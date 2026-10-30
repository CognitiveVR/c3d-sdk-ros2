#!/usr/bin/env bash
# Tier 2: the synthetic integration run, on a host that has no ROS.
#
#   bash tools/run_synthetic_integration.sh
#
# It stands up the fakes, the node and tools/stub_gateway.py, drives a session,
# and then ASSERTS on the bytes that would actually have been transmitted -- the
# JSON streams by reading the captures, and the two binary streams by decoding
# them with tools/capnp_dump.py and every camera chunk again with ffmpeg.
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

# Each camera chunk's stream, extracted by capnp_dump for ffmpeg to decode on its own.
H264="$LOGS/h264"

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

# The camera publisher's rate. The assertions bound the time between two stored frames with it.
CAMERA_PUBLISH_HZ=5.0

echo "=== starting the stub gateway and the fakes ==="
start stub_gateway python3 /src/tools/stub_gateway.py 8931 "$CAPTURE"
start tf_publisher python3 /src/tools/fake_tf_publisher.py
start nav_server python3 /src/tools/fake_nav_server.py
# The three the binary streams need. Rates above what the node is configured to
# take, so the rate gate is exercised rather than merely satisfied.
start camera_publisher python3 /src/tools/fake_camera_publisher.py --rate "$CAMERA_PUBLISH_HZ"
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
RUN_START="$RUN_START" RUN_END="$RUN_END" CAPTURE="$CAPTURE" LOGS="$LOGS" H264="$H264" \
  CAMERA_PUBLISH_HZ="$CAMERA_PUBLISH_HZ" python3 - <<'PY'
import glob
import importlib.util
import json
import math
import os
import subprocess
import sys

capture = os.environ['CAPTURE']
run_start = int(os.environ['RUN_START'])
run_end = int(os.environ['RUN_END'])
duration = run_end - run_start
# camera.<name>.rate_hz in synthetic_params.yaml, the same for front and zoom.
CAMERA_RATE_HZ = 2.0
CAMERA_PUBLISH_HZ = float(os.environ['CAMERA_PUBLISH_HZ'])

failures = []


def check(condition, message):
    print(('  ok   ' if condition else '  FAIL ') + message)
    if not condition:
        failures.append(message)


def publisher(name):
    """One of the fakes, imported for its constants and its argparse defaults, so an expectation
    here cannot drift from what was published: the run passes neither a ray count nor a range.
    Importing runs nothing, because each fake spins only under __main__."""
    spec = importlib.util.spec_from_file_location(name, f'/src/tools/{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


camera_files = sorted(glob.glob(os.path.join(capture, '*-camera.bin')))
lidar_files = sorted(glob.glob(os.path.join(capture, '*-lidar.bin')))
partials = glob.glob(os.path.join(capture, '*.partial'))
# A leftover `.partial` matches no reader's glob, so the capture would be silently
# short by one -- which is worse than a corrupt file, because a wrong aggregate
# still looks measured.
check(not partials, f'no .partial orphans left behind ({len(partials)} found)')
check(bool(camera_files), f'camera chunks captured: {len(camera_files)}')
check(bool(lidar_files), f'lidar blocks captured: {len(lidar_files)}')
if not camera_files or not lidar_files:
    sys.exit(1)

document = json.loads(subprocess.check_output(
    [sys.executable, '/src/tools/capnp_dump.py', '--scan-ranges',
     '--extract-h264', os.environ['H264'], *camera_files, *lidar_files]))
entries = document['files']
cameras = [e for e in entries if e['stream'] == 'camera']
lidars = [e for e in entries if e['stream'] == 'lidar']
front = [c for c in cameras if c['cameraId'] == 'front']
zoom = [c for c in cameras if c['cameraId'] == 'zoom']
scan_blocks = [e for e in lidars if e['sensorId'] == 'scan']
depth_objects = [e for e in lidars if e['sensorId'] == 'depth']
frames = [f for c in cameras for f in c['frames']]
scans = [s for b in scan_blocks for s in b['entries']]
clouds = [c for o in depth_objects for c in o['entries']]


# Rate x duration, with a wide tolerance: the session opens after activate and an
# exact count would be a flaky test rather than a stricter one. So only a gross loss
# shows here; the continuity checks on each camera's frames below are what see one
# lost frame.
def within(count, rate, name):
    expected = rate * duration
    check(0.4 * expected <= count <= 1.2 * expected + 4,
          f'{name}: {count} records for {rate} Hz over {duration}s (expected about {expected:.0f})')


# A chunk and a scan block each span 3 s of real time; the upper bound allows the
# short one that ends the session and one more.
def one_per_three_seconds(count, name):
    check(0.4 * duration / 3 <= count <= duration / 3 + 2,
          f'{name}: {count} over {duration}s, at one per 3 s')


check(len(front) + len(zoom) == len(cameras), 'every chunk carries a configured camera name')
check(len(scan_blocks) + len(depth_objects) == len(lidars),
      'every lidar block carries a configured lidar name')
within(len([f for c in front for f in c['frames']]), CAMERA_RATE_HZ, 'camera front frames')
within(len([f for c in zoom for f in c['frames']]), CAMERA_RATE_HZ, 'camera zoom frames')
within(len(scans), 2.0, 'lidar scan')
within(len(clouds), 1.0, 'lidar depth')
one_per_three_seconds(len(front), 'camera front chunks')
one_per_three_seconds(len(zoom), 'camera zoom chunks')
one_per_three_seconds(len(scan_blocks), 'lidar scan blocks')

# seq counts one sensor's objects from 0, so it is checked per sensor, and a missing
# number is an object lost. Capture order is upload order, and the uploader sends the
# oldest object first.
for name, group in (('camera front', front), ('camera zoom', zoom), ('lidar scan', scan_blocks),
                    ('lidar depth', depth_objects)):
    seqs = [e['seq'] for e in group]
    check(bool(seqs) and seqs == list(range(len(seqs))),
          f'{name} seq runs from 0 with none missing: {seqs}')

# Continuity, frame by frame, across chunk boundaries. Every frame the node poses takes the
# camera's next frame seq whether or not it is stored, so a frame lost anywhere -- dropped as
# backlog, or held by the encoder when a chunk is closed by force -- leaves a gap in seq. A
# frame lost after the last one stored leaves none, so a loss at the session's end does not
# show here.
#
# And in time, which catches a hole no posed frame was taken for. The gate takes the first
# frame at or after its deadline, and its deadline is at most one gate period after the frame
# it took last, so two stored frames are at most 1/rate_hz + 1/publisher rate apart:
# 1/2.0 + 1/5.0 = 0.7 s, here 0.4 or 0.6 s, at a chunk boundary as anywhere else. The bound is
# twice the 0.7 s, which a loss of about three frames in a row at a boundary exceeds and a
# loss of one does not.
boundary_bound = 2 * (1 / CAMERA_RATE_HZ + 1 / CAMERA_PUBLISH_HZ)
for name, group in (('front', front), ('zoom', zoom)):
    chunks = sorted(group, key=lambda c: c['seq'])
    frame_seqs = [f['seq'] for c in chunks for f in c['frames']]
    check(bool(frame_seqs) and frame_seqs == list(range(len(frame_seqs))),
          f'camera {name} frame seq runs from 0 across its chunks with none missing '
          f'({len(frame_seqs)} frames; missing '
          f'{sorted(set(range(max(frame_seqs, default=0) + 1)) - set(frame_seqs))[:8]})')
    gaps = [(b['frames'][0]['time'] - a['frames'][-1]['time']) / 1000.0
            for a, b in zip(chunks, chunks[1:]) if a['frames'] and b['frames']]
    check(bool(gaps) and all(0 < gap <= boundary_bound for gap in gaps),
          f'camera {name}: from each chunk\'s last frame to the next chunk\'s first is at most '
          f'{boundary_bound:.1f} s (largest {max(gaps, default=0.0):.3f} s, '
          f'smallest {min(gaps, default=0.0):.3f} s, over {len(gaps)} boundaries)')

# A chunk is one closed group of pictures, which is what lets it decode on its own.
check(all(c['frames'] and c['frames'][0]['keyframe'] for c in cameras),
      'every chunk opens on a keyframe')
check(all(c['offsetsTile'] for c in cameras),
      "every chunk's frames are its data, back to back from byte 0 to the last")
check(all(c['keyframesMatchNalTypes'] for c in cameras),
      'every frame flagged keyframe carries an IDR slice, and no other frame does')
check(all(f['startsWithAud'] for f in frames),
      f"every frame's access unit opens on a delimiter ({len(frames)} frames)")
check(all(c['width'] <= 160 for c in front),
      f'every front chunk is at or below max_width_px 160 (widest {max(c["width"] for c in front)})')
# The field of view the parameters set, on every chunk, exactly: both are Float32-exact. The
# camera's sibling CameraInfo, which "front" has too, does not override them.
fovs = sorted({(c['hfovDegrees'], c['vfovDegrees']) for c in front})
check(fovs == [(61.0, 45.5)], f'every front chunk carries its parameters, 61 x 45.5: {fovs}')
# "zoom" has no parameters, so each chunk carries the latest CameraInfo's field of view at its
# first frame, and the publisher's focal length alternates between half the 320 x 240 image's
# width and all of it. Which of the two a chunk opens on is not something to assert.
zoom_fovs = [
    (2 * math.degrees(math.atan(160 / f)), 2 * math.degrees(math.atan(120 / f)))
    for f in (160.0, 320.0)]


def zoom_level(chunk):
    for i, (h, v) in enumerate(zoom_fovs):
        if abs(chunk['hfovDegrees'] - h) < 1e-3 and abs(chunk['vfovDegrees'] - v) < 1e-3:
            return i
    return None


# A chunk that opened before the first CameraInfo arrived carries none (0 by 0): the two
# subscriptions connect independently. That is allowed for the first chunk and no other.
levels = [zoom_level(c) for c in sorted(zoom, key=lambda c: c['seq'])]
unknown = [c for c in zoom if c['hfovDegrees'] == 0.0 and c['vfovDegrees'] == 0.0]
leading = next((i for i, level in enumerate(levels) if level is not None), len(levels))
check(zoom and leading <= 1 and None not in levels[leading:] and len(unknown) == leading,
      f'every zoom chunk after the first CameraInfo carries one of its two levels '
      f'({leading} before it): {levels}')

# Each chunk decodes on its own, to exactly the frames it lists. `-vsync passthrough` because a
# raw stream carries no timestamps and ffmpeg's constant-rate default can drop a frame of one.
undecodable = []
for chunk in cameras:
    decoded = subprocess.run(
        ['ffmpeg', '-hide_banner', '-loglevel', 'error', '-f', 'h264', '-i', chunk['extracted'],
         '-vsync', 'passthrough', '-f', 'rawvideo', '-pix_fmt', 'gray', '-'],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    want = len(chunk['frames']) * chunk['width'] * chunk['height']
    if decoded.returncode != 0 or len(decoded.stdout) != want:
        undecodable.append((chunk['cameraId'], chunk['seq'], decoded.returncode,
                            len(decoded.stdout), want, decoded.stderr.decode()[-200:]))
check(not undecodable,
      f'every chunk decodes with ffmpeg to frames x width x height grey bytes '
      f'({len(cameras)} chunks): {undecodable[:3]}')

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
times = [r['time'] for r in frames + scans + clouds]
check(all(run_start * 1000 - 5000 <= t <= run_end * 1000 + 5000 for t in times),
      f'every record time is inside the run window ({min(times)} .. {max(times)})')

for stream, group in (('camera', cameras), ('lidar', lidars)):
    check(all(e['header']['sdkVersion'] == group[0]['header']['sdkVersion'] for e in group),
          f'{stream} carries one sdkVersion: {group[0]["header"]["sdkVersion"]}')
    check(all(e['header']['sessionId'] == group[0]['header']['sessionId'] for e in group),
          f'{stream} carries one sessionId: {group[0]["header"]["sessionId"]}')
    check(all(e['header']['formatVersion'] == '2.0' for e in group),
          f'{stream} formatVersion is 2.0')

# The JSON streams join the binary ones on sessionId. This is the whole point of
# sending the same value: rows from six streams describe one session.
gaze = sorted(glob.glob(os.path.join(capture, '*-gaze.json')))
check(bool(gaze), f'gaze parts captured: {len(gaze)}')
if gaze:
    gaze_session = json.load(open(gaze[0]))['body']['sessionid']
    check(gaze_session == cameras[0]['header']['sessionId'],
          f'the camera stream joins the gaze stream on {gaze_session}')
    check(gaze_session == lidars[0]['header']['sessionId'],
          f'the lidar stream joins the gaze stream on {gaze_session}')
    # A robot has no eyes: a gaze record is a position and a rotation, and a gaze point would be
    # counted by the platform's gaze analytics as somewhere the robot looked.
    records = [r for path in gaze for r in json.load(open(path))['body'].get('data', [])]
    check(bool(records), f'gaze records captured: {len(records)}')
    check(all(sorted(r) == ['p', 'r', 'time'] for r in records),
          'every gaze record is time, p and r, with no gaze point')

check(bool(scan_blocks) and all(b['kind'] == 'scans' for b in scan_blocks),
      f'lidar scan is stored as scan blocks: {len(scan_blocks)}')
check(bool(depth_objects) and all(o['kind'] == 'clouds' for o in depth_objects),
      f'lidar depth is stored as cloud objects: {len(depth_objects)}')
if scan_blocks:
    laserscan = publisher('fake_laserscan_publisher')
    published = laserscan.parse_args([])
    check(all(b['rays'] == published.rays for b in scan_blocks),
          f"every scan block carries the publisher's {published.rays} rays")
    # docs/data-format.md: millimetres when rangeMax is 65 m or less, centimetres above it.
    unit = 'millimetre' if published.range_max <= 65.0 else 'centimetre'
    check(all(b['rangeUnit'] == unit for b in scan_blocks),
          f'every scan block is in {unit}s, for a {published.range_max} m rangeMax')
    check(all(b['encoding'] == 'deltaShuffleZstd' for b in scan_blocks),
          'every scan block is deltaShuffleZstd')
    check(all(len(s['ranges']) == published.rays for s in scans),
          f'every scan decodes to {published.rays} ranges')

    # Which rays the publisher made non-finite, by its own constants and in its own order: a
    # sentinel has to come back as the same kind of nothing, not merely as some kind.
    def published_kind(ray):
        if ray % laserscan.NO_RETURN_EVERY == 0:
            return 'inf'
        if ray % laserscan.INVALID_EVERY == laserscan.INVALID_PHASE:
            return 'nan'
        return 'finite'

    def kind(value):
        if math.isnan(value):
            return 'nan'
        if math.isinf(value):
            return 'inf' if value > 0 else '-inf'
        return 'finite'

    wrong = [(s['seq'], ray, kind(value)) for s in scans for ray, value in enumerate(s['ranges'])
             if kind(value) != published_kind(ray)]
    check(not wrong, f'every ray the publisher made inf or NaN is that after the round trip, and '
                     f'every other ray is finite: {len(wrong)} wrong, first {wrong[:4]}')
    # Half a millimetre either side: a range is stored as the nearest whole millimetre.
    finite = [v for s in scans for v in s['ranges'] if math.isfinite(v)]
    check(bool(finite) and published.range_min - 0.0005 <= min(finite)
          and max(finite) <= published.range_max + 0.0005,
          f'every finite range is inside [{published.range_min}, {published.range_max}] m '
          f'({min(finite, default=None)} .. {max(finite, default=None)})')

    # The publisher draws a smooth sine across the sweep, so neighbouring rays are millimetres
    # apart. Its own message counter is not on the wire, so its formula cannot be evaluated here.
    def steepest_step(ranges):
        values = [v for v in ranges if math.isfinite(v)]
        return max((abs(b - a) for a, b in zip(values, values[1:])), default=0.0)

    steepest = max((steepest_step(s['ranges']) for s in scans), default=math.inf)
    check(steepest < 0.1,
          f'scan by scan, consecutive finite rays differ by under 0.1 m (largest {steepest:.4f} m)')
    check(all(len(s['intensities']) == published.rays for s in scans),
          'intensities are present on every scan, because this lidar asked for them')
    # A ray's intensity does not depend on the publisher's counter, so the publisher's own
    # function gives every one, and each is stored as its nearest whole code.
    _ranges, intensities = laserscan.ranges_and_intensities(
        published.rays, published.range_min, published.range_max, 0)
    off = [(s['seq'], ray, code) for s in scans
           for ray, (code, want) in enumerate(zip(s['intensities'], intensities))
           if abs(code - want) > 0.5 + 1e-3]
    check(not off, f'every intensity is the published one for its ray, to the nearest whole code, '
                   f'{min(intensities):.0f} to {max(intensities):.0f}: '
                   f'{len(off)} off, first {off[:4]}')

if depth_objects:
    pointcloud = publisher('fake_pointcloud_publisher')
    check(all(o['encoding'] == 'planarDeltaShuffleZstd' for o in depth_objects),
          'every cloud object is planarDeltaShuffleZstd')
    check(all(o['zstdLevel'] == 1 for o in depth_objects),
          'every cloud object carries the configured zstd_level 1')
    check(all(o['position'] == 'int16_mm' and o['color'] == 'rgb565' for o in depth_objects),
          'every cloud object carries the configured position and colour')
    check(all(o['voxelLeafMm'] == 50 for o in depth_objects), 'the voxel leaf is recorded as 50 mm')
    check(all(c['pointCount'] <= 100000 for c in clouds),
          f'every cloud is at or under max_points (largest {max(c["pointCount"] for c in clouds)})')
    # Each block DECODES: pointCount and the format fields describe it and nothing
    # else does, so a header that disagrees with its own bytes is exactly the
    # failure this decode catches.
    check(all(c['decoded']['consistent'] for c in clouds),
          'every cloud decodes from its own slice to the point count its entry claims')
    check(all(c['decoded']['points'] == c['pointCount'] for c in clouds),
          'the decoded point count matches the entry')
    # A wrong inverse transform still decodes to the right NUMBER of points, so the positions
    # are checked as well: each is a voxel centroid of points drawn inside the publisher's box
    # with its jitter, give or take the int16 millimetre.
    slack = pointcloud.JITTER_M + 0.001
    outside = [point for c in clouds for point in c['decoded']['first']
               if not all(low - slack <= point[axis] <= high + slack
                          for axis, (low, high) in enumerate(pointcloud.BOX))]
    check(not outside, f"every decoded point reported lies inside the publisher's box "
                       f'{pointcloud.BOX}: {outside[:3]}')

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
