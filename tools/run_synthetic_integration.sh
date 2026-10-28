#!/usr/bin/env bash
# Tier 2: the synthetic integration run, on a host that has no ROS.
#
#   bash tools/run_synthetic_integration.sh
#
# It stands up the fakes, the node and tools/stub_gateway.py, drives a session,
# and then ASSERTS on the bytes that would actually have been transmitted -- the
# JSON streams by reading the captures, and the three binary streams by decoding
# them with tools/capnp_dump.py. The pose stream's blocks are decoded down to
# their joint values.
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
rm -f "$CAPTURE"/*.json "$CAPTURE"/*.bin "$CAPTURE"/*.partial "$CAPTURE"/*.jsonl 2>/dev/null || true

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
# With a camera on a panning gimbal: a moving joint for the pose stream's rig. And a robot
# description, latched before the node starts, which decorates the rig.
start tf_publisher python3 /src/tools/fake_tf_publisher.py --gimbal \
  --robot-description /src/tools/synthetic_robot_description.urdf
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
grep -e 'field of view' -e 'pose stream' "$LOGS/doctor.log" || echo "(doctor printed no field of view or pose stream line)"

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
import hashlib
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

# ---- The pose stream -------------------------------------------------------------------------
# The fake robot's tree under base_link: base_scan, static 0.2 m up and tagged as the depth lidar's
# frame, and camera_link panning on a gimbal with its optical frame under it.
pose_files = sorted(glob.glob(os.path.join(capture, '*-pose.bin')))
check(bool(pose_files), f'pose parts captured: {len(pose_files)}')
if pose_files:
    pose_parts = json.loads(subprocess.check_output(
        [sys.executable, '/src/tools/capnp_dump.py', *pose_files]))['files']
    numbers = [e['header']['part'] for e in pose_parts]
    check(numbers == list(range(1, len(numbers) + 1)),
          f'pose parts are numbered from 1 with none missing: {numbers}')
    if gaze:
        check(all(e['header']['sessionId'] == gaze_session for e in pose_parts),
              'the pose stream joins the gaze stream on its session id')
    check(not any(e['manifests'] and e['blocks'] for e in pose_parts),
          'no pose part carries both manifests and blocks')
    check(bool(pose_parts[0]['manifests']) and not pose_parts[0]['blocks'],
          'the first pose part is a manifest')
    manifest_parts = [e for e in pose_parts if e['manifests']]
    block_parts = [e for e in pose_parts if e['blocks']]
    check(all(e['startUs'] == 0 and e['endUs'] == 0 for e in manifest_parts),
          'a manifest part has no startUs or endUs')
    manifests = [m for e in pose_parts for m in e['manifests']]
    first = manifests[0]
    check(first['manifestId'] == 'ros2-a8ee2656b910c9c7' and first['name'] == 'sim-robot-01',
          f'the rig is named after device_id: {first["manifestId"]} {first["name"]}')
    check(first['mesh'] == '' and first['fileType'] == '' and
          first['riggingMappingType'] == 'meshName' and first['root'] == '',
          'no model, meshName mapping, no root')
    # The description is latched before the node starts, but it and the first tick race, so the
    # first manifest may be tf's alone; the last one is decorated from it.
    tier1 = {'c3d.profile': 'ros2/1', 'ros.robotName': 'sim-robot-01'}
    with open('/src/tools/synthetic_robot_description.urdf', 'rb') as f:
        description_sha = hashlib.sha256(f.read()).hexdigest()
    tier2 = {'c3d.profile': 'ros2/1', 'ros.robotName': 'synthetic_bot',
             'ros.urdfSha256': description_sha, 'c3d.model.format': 'urdf',
             'c3d.model.formatVersion': '1.2'}
    check(all(m['properties'] in (tier1, tier2) for m in manifests),
          f'every manifest carries the ros2/1 object keys, tf\'s alone or with the robot '
          f'description\'s: {[m["properties"] for m in manifests]}')
    check(manifests[-1]['properties'] == tier2,
          f'the last manifest carries the robot description\'s name, hash and version: '
          f'{manifests[-1]["properties"]}')
    last = manifests[-1]
    names = [j['meshName'] for j in last['joints']]
    check(names == ['base_link', 'base_scan', 'camera_link', 'camera_optical_frame'],
          f'the last manifest is the whole rig, parents first: {names}')
    check([j['j'] for j in last['joints']] == list(range(len(names))), 'joints are 0..J-1')
    by_name = {j['meshName']: j['properties'] for j in last['joints']}
    check('ros.parent' not in by_name.get('base_link', {}), 'the root names no parent')
    check(by_name.get('base_scan', {}).get('ros.parent') == 'base_link' and
          by_name.get('camera_link', {}).get('ros.parent') == 'base_link' and
          by_name.get('camera_optical_frame', {}).get('ros.parent') == 'camera_link',
          f'every other joint names its tf parent: {by_name}')
    check(by_name.get('base_scan', {}).get('c3d.kind') == 'lidar',
          'base_scan, the depth lidar\'s configured frame, is tagged lidar')
    check(by_name.get('base_link') == {},
          f'the root carries no joint keys: {by_name.get("base_link")}')
    check(by_name.get('camera_link') == {'ros.parent': 'base_link', 'ros.jointName': 'gimbal_pan',
                                         'ros.jointType': 'revolute', 'ros.limit.lower': -0.8,
                                         'ros.limit.upper': 0.8, 'ros.limit.velocity': 3.0},
          f'camera_link carries its URDF joint and only its bounded limits: '
          f'{by_name.get("camera_link")}')
    check(by_name.get('base_scan', {}).get('ros.jointName') == 'scan_mount' and
          by_name.get('base_scan', {}).get('ros.jointType') == 'fixed' and
          by_name.get('camera_optical_frame', {}).get('ros.jointName') == 'camera_optical_joint',
          'the fixed joints are named and typed')
    check('wheel_left_link' not in by_name, 'a URDF link tf does not publish is no joint')
    # The description's <ros2_control> holds an address and a serial port. Nothing that left the
    # node, and nothing it logged, may carry them: every capture's bytes and every log are searched.
    leaked = []
    for path in sorted(glob.glob(os.path.join(capture, '*')) +
                       glob.glob(os.path.join(os.environ['LOGS'], '*.log'))):
        if os.path.isfile(path) and not path.endswith(('tf_publisher.log', 'build.log')):
            with open(path, 'rb') as f:
                body = f.read()
            if b'192.0.2.55' in body or b'SECRET' in body:
                leaked.append(os.path.basename(path))
    check(not leaked, f'nothing of the description\'s <ros2_control> reached a capture or a log: '
                      f'{leaked}')
    with open(os.path.join(os.environ['LOGS'], 'node.log')) as f:
        node_log = f.read()
    check("robot description read: robot 'synthetic_bot', URDF 1.2" in node_log,
          'the node logs the description it read by name and version')

    blocks = [b for e in block_parts for b in e['blocks']]
    check(bool(blocks), f'pose blocks captured: {len(blocks)}')
    seqs = [b['seq'] for b in blocks]
    check(seqs == list(range(len(seqs))), f'block seq runs 0.. with none missing ({len(seqs)})')
    first_seqs = {m['firstSeq'] for m in manifests}
    reset = {b['seq'] for b in blocks if b['resetBefore']}
    check(reset == {s for s in first_seqs if s < len(blocks)},
          f'resetBefore is set exactly at each manifest send\'s firstSeq: {sorted(reset)}')
    check(blocks[0]['decoded']['keyframe'], 'the first block is a keyframe')
    check(all(b['decoded']['consistent'] for b in blocks), 'every block body decodes whole')
    check(all(b['sampleCount'] == b['decoded']['sampleCount'] and
              b['startUs'] == b['decoded']['startUs'] for b in blocks),
          'every envelope copies its block header')
    for e in block_parts:
        starts = [b['startUs'] for b in e['blocks']]
        ends = [b['decoded']['times'][-1] for b in e['blocks']]
        if e['startUs'] != min(starts) or e['endUs'] != max(ends):
            check(False, f'pose part {e["header"]["part"]} startUs/endUs span its blocks')
            break
    else:
        check(True, 'every block part\'s startUs and endUs span its blocks')
    times = [t for b in blocks for t in b['decoded']['times']]
    check(all(run_start * 1e6 - 5e6 <= t <= run_end * 1e6 + 5e6 for t in times),
          f'every pose sample time is wall clock inside the run ({min(times)} .. {max(times)})')
    check(len(times) >= 5 * duration,
          f'{len(times)} pose samples over {duration}s at 10 Hz')

    # The manifest each block decodes with: the latest whose firstSeq is at or before its seq.
    def in_force(seq):
        chosen = None
        for m in manifests:
            if m['firstSeq'] <= seq:
                chosen = m
        return chosen
    scan_heights, pans, root_x = [], [], []
    for b in blocks:
        m = in_force(b['seq'])
        joints = [j['meshName'] for j in m['joints']]
        if b['decoded']['jointCount'] != len(joints):
            check(False, f'block {b["seq"]} has {b["decoded"]["jointCount"]} joints, its manifest '
                         f'{len(joints)}')
        written = b['decoded']['joints']
        if 'base_scan' in joints and 'p' in written.get(str(joints.index('base_scan')), {}):
            scan_heights += written[str(joints.index('base_scan'))]['p']
        if 'camera_link' in joints and 'r' in written.get(str(joints.index('camera_link')), {}):
            pans += [q[2] for q in written[str(joints.index('camera_link'))]['r']]
        if 'p' in written.get('0', {}):
            root_x += [p[0] for p in written['0']['p']]
    check(bool(scan_heights) and all(abs(p[0]) < 1e-4 and abs(p[1]) < 1e-4 and
                                     abs(p[2] - 0.2) < 1e-4 for p in scan_heights),
          f'base_scan is 0.2 m above base_link in every sample that writes it '
          f'({len(scan_heights)} samples, rep103)')
    check(bool(pans) and max(pans) - min(pans) > 0.2,
          f'camera_link pans on its gimbal: rotation z from {min(pans or [0]):.3f} '
          f'to {max(pans or [0]):.3f}')
    check(bool(root_x) and root_x[-1] > root_x[0] + 1.0,
          f'the root drives forward in the reference frame: x {root_x[:1]} .. {root_x[-1:]}')

    properties = {}
    for path in gaze:
        properties.update(json.load(open(path))['body'].get('properties', {}))
    check(properties.get('c3d.ros.pose.rate_hertz') == 10.0,
          f'c3d.ros.pose.rate_hertz: {properties.get("c3d.ros.pose.rate_hertz")}')
    check(properties.get('c3d.ros.pose.frame_count') == 4,
          f'c3d.ros.pose.frame_count: {properties.get("c3d.ros.pose.frame_count")}')
    check(properties.get('c3d.stream.pose.blockIntervalMs') == 1000,
          f'c3d.stream.pose.blockIntervalMs: {properties.get("c3d.stream.pose.blockIntervalMs")}')
    check(isinstance(properties.get('c3d.stream.pose.heldFrames'), int),
          f'c3d.stream.pose.heldFrames: {properties.get("c3d.stream.pose.heldFrames")}')
    pose_doctor = [line for line in open(os.path.join(os.environ['LOGS'], 'doctor.log'))
                   if 'pose stream' in line]
    check(any('[ok  ] pose stream' in line and '4 frame(s) under base_link' in line and
              'base_link, base_scan, camera_link, camera_optical_frame' in line
              for line in pose_doctor),
          f'doctor names the rig it would send: {pose_doctor}')
    send_log = os.path.join(capture, 'pose_send_log.jsonl')
    lines = [json.loads(line) for line in open(send_log)] if os.path.exists(send_log) else []
    check(len(lines) == len(block_parts),
          f'the send log has one line per block part: {len(lines)} for {len(block_parts)}')

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
  start "$convention-tf_publisher" python3 /src/tools/fake_tf_publisher.py --gimbal \
    --mount-frame "$MOUNT_FRAME" --mount-xyz "$MOUNT_XYZ" --mount-rpy-deg "$MOUNT_RPY_DEG"
  start "$convention-camera_publisher" python3 /src/tools/fake_camera_publisher.py \
    --rate 4.0 --frame "$MOUNT_FRAME"
  start "$convention-laserscan_publisher" python3 /src/tools/fake_laserscan_publisher.py \
    --rate 4.0 --frame "$MOUNT_FRAME"
  sleep 5

  start "$convention-node" \
    /tmp/ws/install/cognitive3d_ros/lib/cognitive3d_ros/cognitive3d_node \
    --ros-args --params-file /src/tools/scan_plane_params.yaml \
    -p pose_convention:="$convention" -p pose.enabled:=true -p pose.block_interval_s:=1.0
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

  # The pose stream under a convention whose position map is a reflection, read off the wire at
  # its two static joints: base_scan 0.2 m up, and camera_optical_frame's fixed turn under
  # camera_link. Every joint takes the position map and the Dynamic Object rotation rule, written
  # out here rather than taken from the SDK.
  CAPTURE_DIR="$capture" CONVENTION="$convention" python3 - <<'PY'
import glob
import json
import os
import subprocess
import sys

convention = os.environ['CONVENTION']
files = sorted(glob.glob(os.path.join(os.environ['CAPTURE_DIR'], '*-pose.bin')))
if not files:
    print(f'  FAIL no pose parts captured under {convention}')
    sys.exit(1)
parts = json.loads(subprocess.check_output([sys.executable, '/src/tools/capnp_dump.py', *files]))
manifests = [m for e in parts['files'] for m in e['manifests']]
blocks = [b for e in parts['files'] for b in e['blocks']]
# Position map M and the Dynamic Object rule, per convention (docs/data-format.md).
position = {'unity': lambda x, y, z: (-y, z, x),
            'gltf_authored': lambda x, y, z: (-x, z, -y)}[convention]
rotation = {'unity': lambda x, y, z, w: (-y, z, x, -w),
            'gltf_authored': lambda x, y, z, w: (x, -z, y, w)}[convention]
want_scan = position(0.0, 0.0, 0.2)
want_optical = rotation(-0.5, 0.5, -0.5, 0.5)
failures = []
seen_scan = seen_optical = 0
for block in blocks:
    joints = None
    for m in manifests:
        if m['firstSeq'] <= block['seq']:
            joints = [j['meshName'] for j in m['joints']]
    written = block['decoded']['joints']
    if 'base_scan' in joints:
        for p in written.get(str(joints.index('base_scan')), {}).get('p', []):
            seen_scan += 1
            if max(abs(a - b) for a, b in zip(p, want_scan)) > 1e-4:
                failures.append(f'base_scan at {p}, want {want_scan}')
    if 'camera_optical_frame' in joints:
        for q in written.get(str(joints.index('camera_optical_frame')), {}).get('r', []):
            seen_optical += 1
            sign = 1 if sum(a * b for a, b in zip(q, want_optical)) >= 0 else -1
            if max(abs(sign * a - b) for a, b in zip(q, want_optical)) > 1e-4:
                failures.append(f'camera_optical_frame turned {q}, want {want_optical}')
if not seen_scan or not seen_optical:
    failures.append(f'samples read: base_scan {seen_scan}, camera_optical_frame {seen_optical}')
for failure in failures[:5]:
    print('  FAIL ' + failure)
if failures:
    sys.exit(1)
print(f'  ok   the pose stream under {convention}: base_scan at {want_scan} in {seen_scan} samples, '
      f'camera_optical_frame turned {want_optical} in {seen_optical}')
PY
}

scan_plane_pass gltf_authored
scan_plane_pass unity

echo
echo "synthetic integration: captures, and the scan plane under gltf_authored and unity"
