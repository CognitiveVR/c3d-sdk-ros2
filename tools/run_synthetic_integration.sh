#!/usr/bin/env bash
# Tier 2: the synthetic integration run, on a host that has no ROS.
#
#   bash tools/run_synthetic_integration.sh
#
# It stands up the fakes, the node and tools/stub_gateway.py, drives a session,
# and then ASSERTS on the bytes that would actually have been transmitted -- the
# JSON streams by reading the captures, and the two binary streams by decoding
# them with tools/capnp_dump.py.
#
# Then it does it twice more, under the two conventions whose position map is a
# REFLECTION, with a camera and a LaserScan on one tilted mount: that is the only
# place the pose a laserscan record carries is distinguishable from the pose every
# other record carries, and tools/verify_scan_plane.py is what tells them apart.
# The first pass runs rep103, where the two rules coincide and cannot be.
#
# The script is its own container entry point: the outer half resolves the same
# image tools/build_and_test.sh uses and re-runs this file inside it with
# `--in-container`. That is why there is no list of files to stage anywhere -- the
# repository is mounted read-only at /src and everything is read from there.
#
# CAPTURE=/some/dir keeps the capture directory afterwards by bind-mounting it;
# without it the captures live and die inside the container.
set -euo pipefail

if [ "${1:-}" != "--in-container" ]; then
  REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
  digest() {
    if command -v shasum >/dev/null 2>&1; then shasum -a 256; else sha256sum; fi
  }
  if [ -z "${IMAGE:-}" ]; then
    # Identical derivation to tools/build_and_test.sh, so the two share one image
    # rather than building a second copy of the same thing.
    IMAGE_INPUTS=("$REPO/docker/Dockerfile.test")
    while IFS= read -r manifest; do IMAGE_INPUTS+=("$manifest"); done < <(
      find "$REPO" -mindepth 2 -maxdepth 2 -name package.xml -not -path '*/.git/*' | sort
    )
    IMAGE="c3d-sdk-ros2-test:$(cat "${IMAGE_INPUTS[@]}" | digest | cut -c1-12)"
  fi
  if [ -n "${REBUILD_IMAGE:-}" ] || ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "=== building $IMAGE (first run takes a few minutes) ==="
    docker build -f "$REPO/docker/Dockerfile.test" -t "$IMAGE" "$REPO"
  fi
  MOUNTS=(-v "$REPO":/src:ro)
  if [ -n "${CAPTURE:-}" ]; then
    mkdir -p "$CAPTURE"
    MOUNTS+=(-v "$CAPTURE":/tmp/c3d_capture)
  fi
  exec docker run --rm "${MOUNTS[@]}" "$IMAGE" \
    bash /src/tools/run_synthetic_integration.sh --in-container
fi

# ---------------------------------------------------------------------------
# Inside the container from here.
# ---------------------------------------------------------------------------
set +u
source /opt/ros/jazzy/setup.bash
set -u

# Its own domain, and LOCALHOST discovery: this run serves /cognitive3d for real,
# and a second one on the same machine would bind the same names.
export ROS_DOMAIN_ID=91
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# The node reads its key from the environment only. A placeholder: the stub records the scheme of
# the Authorization header and never the value.
export C3D_APPLICATION_API_KEY=SYNTHETICTESTKEY

CAPTURE=/tmp/c3d_capture
SPOOL=/tmp/c3d_synth_spool
# The two scan-plane passes share this one, because it is named in their params file.
SCAN_PLANE_SPOOL=/tmp/c3d_scan_plane_spool
LOGS=/tmp/c3d_synth_logs
rm -rf "$SPOOL" "$LOGS"
mkdir -p "$CAPTURE" "$LOGS"
rm -f "$CAPTURE"/*.json "$CAPTURE"/*.bin "$CAPTURE"/*.partial 2>/dev/null || true

echo "=== building the workspace ==="
mkdir -p /tmp/ws/src
find /src -mindepth 2 -maxdepth 2 -name package.xml -not -path "*/.git/*" \
  -exec dirname {} \; | xargs -I{} cp -r {} /tmp/ws/src/
cp -r /src/core /src/tools /src/THIRD_PARTY_NOTICES.md /tmp/ws/src/
touch /tmp/ws/src/tools/COLCON_IGNORE
cd /tmp/ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release > "$LOGS/build.log" 2>&1 || {
  echo "workspace build failed"; tail -40 "$LOGS/build.log"; exit 1; }
set +u
source /tmp/ws/install/setup.bash
set -u

PIDS=()
start() {
  local name="$1"; shift
  "$@" > "$LOGS/$name.log" 2>&1 &
  PIDS+=($!)
}
cleanup() {
  for pid in "${PIDS[@]:-}"; do kill "$pid" 2>/dev/null || true; done
  sleep 1
  for pid in "${PIDS[@]:-}"; do kill -9 "$pid" 2>/dev/null || true; done
  PIDS=()
}
trap cleanup EXIT

# The scan-plane passes' sensor mount, in one place: the harness publishes this
# transform and the verifier rebuilds the ray endpoints from it, so a second copy
# anywhere would be a way for the two to agree about the wrong thing. Roll and
# pitch as well as yaw, because a level scan plane stays level under a rule that
# is wrong about everything else.
MOUNT_FRAME=sensor_mount
MOUNT_XYZ=0.35,-0.12,0.47
MOUNT_RPY_DEG=20,35,55

echo "=== starting the stub gateway and the fakes ==="
start stub_gateway python3 /src/tools/stub_gateway.py 8931 "$CAPTURE"
start tf_publisher python3 /src/tools/fake_tf_publisher.py
start nav_server python3 /src/tools/fake_nav_server.py
# The three the binary streams need. Rates above what the node is configured to
# take, so the rate gate is exercised rather than merely satisfied.
start camera_publisher python3 /src/tools/fake_camera_publisher.py --rate 5.0
start pointcloud_publisher python3 /src/tools/fake_pointcloud_publisher.py --rate 2.0
start laserscan_publisher python3 /src/tools/fake_laserscan_publisher.py --rate 5.0
sleep 6

# doctor against the same graph, before the node holds its name. Only its field of view lines are
# asserted: the rest of its report is about a gateway and a scene this stub does not model.
echo "=== doctor ==="
timeout 90 /tmp/ws/install/cognitive3d_ros/lib/cognitive3d_ros/doctor \
  --ros-args --params-file /src/tools/synthetic_params.yaml > "$LOGS/doctor.log" 2>&1 || true
grep 'field of view' "$LOGS/doctor.log" || echo "(doctor printed no field of view line)"

echo "=== starting the node ==="
start node /tmp/ws/install/cognitive3d_ros/lib/cognitive3d_ros/cognitive3d_node \
  --ros-args --params-file /src/tools/synthetic_params.yaml

for _ in $(seq 1 30); do
  if timeout 10 ros2 lifecycle get /cognitive3d >/dev/null 2>&1; then break; fi
  sleep 1
done
echo "--- configure"; timeout 30 ros2 lifecycle set /cognitive3d configure
# The window the records must fall inside. Read AFTER activate and before
# deactivate, so "every time is wall clock" is checked against this run rather
# than against the century.
RUN_START=$(date +%s)
echo "--- activate";  timeout 30 ros2 lifecycle set /cognitive3d activate
sleep 3
echo "--- driving goals"
timeout 45 python3 /src/tools/fake_goal_driver.py > "$LOGS/goal_driver.log" 2>&1 || true
echo "--- recording"; sleep 10
echo "--- deactivate"; timeout 30 ros2 lifecycle set /cognitive3d deactivate
RUN_END=$(date +%s)
sleep 4
cleanup
trap - EXIT
sleep 1

echo
echo "=== assertions ==="
RUN_START="$RUN_START" RUN_END="$RUN_END" CAPTURE="$CAPTURE" LOGS="$LOGS" python3 - <<'PY'
import glob
import json
import math
import os
import subprocess
import sys

capture = os.environ['CAPTURE']
run_start = int(os.environ['RUN_START'])
run_end = int(os.environ['RUN_END'])
duration = run_end - run_start

failures = []


def check(condition, message):
    print(('  ok   ' if condition else '  FAIL ') + message)
    if not condition:
        failures.append(message)


camera_files = sorted(glob.glob(os.path.join(capture, '*-camera.bin')))
lidar_files = sorted(glob.glob(os.path.join(capture, '*-lidar.bin')))
partials = glob.glob(os.path.join(capture, '*.partial'))
# A leftover `.partial` matches no reader's glob, so the capture would be silently
# short by one -- which is worse than a corrupt file, because a wrong aggregate
# still looks measured.
check(not partials, f'no .partial orphans left behind ({len(partials)} found)')
check(bool(camera_files), f'camera parts captured: {len(camera_files)}')
check(bool(lidar_files), f'lidar parts captured: {len(lidar_files)}')
if not camera_files or not lidar_files:
    sys.exit(1)

document = json.loads(subprocess.check_output(
    [sys.executable, '/src/tools/capnp_dump.py', *camera_files, *lidar_files]))
entries = document['files']
cameras = [e for e in entries if e['stream'] == 'camera']
lidars = [e for e in entries if e['stream'] == 'lidar']

frames = [f for e in cameras for f in e['frames']]
scans = [s for e in lidars for s in e['scans']]

# Rate x duration, with a wide tolerance: the session opens after activate and the
# first part is cut a flush interval later, so an exact count would be a flaky
# test rather than a stricter one.
def within(count, rate, name):
    expected = rate * duration
    check(0.4 * expected <= count <= 1.2 * expected + 4,
          f'{name}: {count} records for {rate} Hz over {duration}s (expected about {expected:.0f})')


front = [f for f in frames if f['cameraId'] == 'front']
zoom = [f for f in frames if f['cameraId'] == 'zoom']
within(len(front), 2.0, 'camera front')
within(len(zoom), 2.0, 'camera zoom')
within(len([s for s in scans if s['sensorId'] == 'scan']), 2.0, 'lidar scan')
within(len([s for s in scans if s['sensorId'] == 'depth']), 1.0, 'lidar depth')

check(all(f['format'] == 'jpeg' for f in frames), 'every frame says format jpeg')
check(all(f['jpegSoi'] for f in frames), 'every frame body starts with a JPEG SOI marker')
check(all(f['width'] <= 160 for f in front),
      f'every front frame is at or below max_width_px 160 (widest {max(f["width"] for f in front)})')
check(len(front) + len(zoom) == len(frames), 'every frame carries a configured name')
# The field of view the parameters set, on every frame, exactly: both are Float32-exact. The
# camera's sibling CameraInfo, which "front" has too, does not override them.
fovs = sorted({(f['hfovDegrees'], f['vfovDegrees']) for f in front})
check(fovs == [(61.0, 45.5)], f'every front frame carries its parameters, 61 x 45.5: {fovs}')
# "zoom" has no parameters, so each frame carries the latest CameraInfo's field of view, and the
# publisher's focal length alternates between half the 320 x 240 image's width and all of it.
zoom_fovs = [
    (2 * math.degrees(math.atan(160 / f)), 2 * math.degrees(math.atan(120 / f)))
    for f in (160.0, 320.0)]
def zoom_level(frame):
    for i, (h, v) in enumerate(zoom_fovs):
        if abs(frame['hfovDegrees'] - h) < 1e-3 and abs(frame['vfovDegrees'] - v) < 1e-3:
            return i
    return None
# A frame taken before the first CameraInfo arrived carries none (0 by 0): the two subscriptions
# connect independently. That is allowed at the start of the stream and nowhere after it.
levels = [zoom_level(f) for f in sorted(zoom, key=lambda f: f['time'])]
unknown = [f for f in zoom if f['hfovDegrees'] == 0.0 and f['vfovDegrees'] == 0.0]
leading = next((i for i, level in enumerate(levels) if level is not None), len(levels))
check(zoom and leading <= 3 and None not in levels[leading:] and len(unknown) == leading,
      f'every zoom frame after the first CameraInfo carries its field of view ({leading} before it): '
      f'{sorted({(round(f["hfovDegrees"], 3), round(f["vfovDegrees"], 3)) for f in zoom})}')
check(set(levels[leading:]) == {0, 1},
      f'the zoom frames carry both zoom levels, frame to frame: {levels}')

# doctor names each camera's source: the parameters for front, the zooming CameraInfo for zoom.
doctor_lines = [line for line in open(os.path.join(os.environ['LOGS'], 'doctor.log'))
                if 'field of view' in line]
check(any('[ok  ] camera/front field of view' in line and
          '61.0 x 45.5 degrees, from camera.front.hfov_degrees' in line for line in doctor_lines),
      'doctor says front takes its field of view from its parameters')
check(any('[ok  ] camera/zoom field of view' in line and 'from /fake/camera/camera_info' in line
          for line in doctor_lines),
      'doctor says zoom takes its field of view from /fake/camera/camera_info')

# The clock rule. Under sim time a stamp starts near zero, and a session whose
# record timestamps do that is accepted with a 200 and never appears -- so this is
# the assertion that would catch ROS time reaching the wire.
times = [f['time'] for f in frames] + [s['time'] for s in scans]
check(all(run_start * 1000 - 5000 <= t <= run_end * 1000 + 5000 for t in times),
      f'every record time is inside the run window ({min(times)} .. {max(times)})')

for stream, group in (('camera', cameras), ('lidar', lidars)):
    parts = [e['header']['part'] for e in group]
    check(parts == sorted(parts) and parts[0] == 1 and len(set(parts)) == len(parts),
          f'{stream} part is monotonic from 1: {parts}')
    check(all(e['header']['sdkVersion'] == group[0]['header']['sdkVersion'] for e in group),
          f'{stream} carries one sdkVersion: {group[0]["header"]["sdkVersion"]}')
    check(all(e['header']['sessionId'] == group[0]['header']['sessionId'] for e in group),
          f'{stream} carries one sessionId: {group[0]["header"]["sessionId"]}')
    check(all(e['header']['formatVersion'] == '1.0' for e in group),
          f'{stream} formatVersion is 1.0')

# The JSON streams join the binary ones on sessionId. This is the whole point of
# sending the same value: rows from six streams describe one session.
gaze = sorted(glob.glob(os.path.join(capture, '*-gaze.json')))
check(bool(gaze), f'gaze parts captured: {len(gaze)}')
if gaze:
    gaze_session = json.load(open(gaze[0]))['body']['sessionid']
    check(gaze_session == cameras[0]['header']['sessionId'],
          f'the camera stream joins the gaze stream on {gaze_session}')
    # A robot has no eyes: a gaze record is a position and a rotation, and a gaze point would be
    # counted by the platform's gaze analytics as somewhere the robot looked.
    records = [r for path in gaze for r in json.load(open(path))['body'].get('data', [])]
    check(bool(records), f'gaze records captured: {len(records)}')
    check(all(sorted(r) == ['p', 'r', 'time'] for r in records),
          'every gaze record is time, p and r, with no gaze point')

laser = [s for s in scans if s['kind'] == 'laserscan']
clouds = [s for s in scans if s['kind'] == 'pointcloud']
check(bool(laser), f'laserscan arms decoded: {len(laser)}')
check(bool(clouds), f'pointcloud arms decoded: {len(clouds)}')
if laser:
    check(all(s['rangeCount'] == 640 for s in laser), 'every scan carries its 640 rays')
    check(all(s['intensityCount'] == 640 for s in laser),
          'intensities are present, because this lidar asked for them')
if clouds:
    check(all(c['pointCount'] <= 100000 for c in clouds),
          f'every cloud is at or under max_points (largest {max(c["pointCount"] for c in clouds)})')
    check(all(c['position'] == 'int16_mm' and c['color'] == 'rgb565' and c['zstdLevel'] == 1
              for c in clouds),
          'every cloud carries the configured position, colour and compression')
    check(all(c['voxelLeafMm'] == 50 for c in clouds), 'the voxel leaf is recorded as 50 mm')
    # The block DECODES: pointCount and the format fields describe it and nothing
    # else does, so a header that disagrees with its own bytes is exactly the
    # failure this decode catches.
    check(all(c['decoded']['consistent'] for c in clouds),
          'every cloud block decodes to the point count its header claims')
    check(all(c['decoded']['points'] == c['pointCount'] for c in clouds),
          'the decoded point count matches the header')

print()
if failures:
    print(f'{len(failures)} assertion(s) FAILED')
    sys.exit(1)
print('synthetic integration: every capture assertion passed')
PY

# ---------------------------------------------------------------------------
# The scan-plane passes.
# ---------------------------------------------------------------------------
#
# A camera and a LaserScan on ONE tf frame, tilted in all three axes, under a
# convention whose position map is a reflection. The laserscan's rays reach the
# wire as the sensor published them, so its record carries the scan plane's
# rotation while the camera's carries the dynamic rule; under rep103 -- the pass
# above -- those are the same numbers, which is why a laserscan call site using
# ScenePose passes everything else in this repository.
scan_plane_pass() {
  local convention="$1"
  local capture="$CAPTURE/scan_plane_$convention"
  rm -rf "$capture"
  mkdir -p "$capture"
  # The spool as well as the capture: a chunk the first pass left behind is uploaded
  # by the second one's recovery pass, into the second one's capture, and the
  # verifier then checks the second convention's rule against the first's records.
  rm -rf "$SCAN_PLANE_SPOOL"

  echo
  echo "=== the scan plane under $convention ==="
  trap cleanup EXIT
  start "$convention-stub_gateway" python3 /src/tools/stub_gateway.py 8932 "$capture"
  # The mount is static under `map` rather than under `base_link`: a sensor that
  # moves with the robot has a world pose only at its own stamp, and this one is
  # the transform that was published, at every stamp.
  start "$convention-tf_publisher" python3 /src/tools/fake_tf_publisher.py \
    --mount-frame "$MOUNT_FRAME" --mount-xyz "$MOUNT_XYZ" --mount-rpy-deg "$MOUNT_RPY_DEG"
  start "$convention-camera_publisher" python3 /src/tools/fake_camera_publisher.py \
    --rate 4.0 --frame "$MOUNT_FRAME"
  start "$convention-laserscan_publisher" python3 /src/tools/fake_laserscan_publisher.py \
    --rate 4.0 --frame "$MOUNT_FRAME"
  sleep 5

  start "$convention-node" \
    /tmp/ws/install/cognitive3d_ros/lib/cognitive3d_ros/cognitive3d_node \
    --ros-args --params-file /src/tools/scan_plane_params.yaml \
    -p pose_convention:="$convention"
  for _ in $(seq 1 30); do
    if timeout 10 ros2 lifecycle get /cognitive3d >/dev/null 2>&1; then break; fi
    sleep 1
  done
  echo "--- configure"; timeout 30 ros2 lifecycle set /cognitive3d configure
  echo "--- activate";  timeout 30 ros2 lifecycle set /cognitive3d activate
  echo "--- recording"; sleep 10
  echo "--- deactivate"; timeout 30 ros2 lifecycle set /cognitive3d deactivate
  sleep 3
  cleanup
  trap - EXIT
  sleep 1

  python3 /src/tools/verify_scan_plane.py --capture "$capture" --convention "$convention" \
    --frame "$MOUNT_FRAME" --lidar scan --camera mount \
    --mount-xyz "$MOUNT_XYZ" --mount-rpy-deg "$MOUNT_RPY_DEG"
}

scan_plane_pass gltf_authored
scan_plane_pass unity

echo
echo "synthetic integration: captures, and the scan plane under gltf_authored and unity"
