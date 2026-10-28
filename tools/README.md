# Development, test and release tools

Three test tiers, in increasing cost. Use the cheapest one that can find the bug you are looking
for. `tools/build_and_test.sh` runs each of them in a container, so a host with no ROS runs the
whole suite; [CONTRIBUTING.md](../CONTRIBUTING.md) covers the selectors and when to run which.

## Unit tests. Every push, in CI

```bash
tools/build_and_test.sh core      # core/ alone, with no ROS in the environment
tools/build_and_test.sh ros       # the ament packages, on Jazzy
tools/build_and_test.sh humble    # core/ and the ament packages, on Humble
```

No ROS graph, no simulator, no network. This is where the coordinate conversion, the wire
conformance, the spool recovery and every translator edge case live. In a sourced ROS workspace,
`colcon test && colcon test-result --verbose` runs the ament half directly.

### The pose encoder against the platform's vectors. By hand

The pose block encoder (`core/include/c3d/pose.hpp`) writes a format the platform owns: its
specification, reference implementation and conformance vectors are in Cognitive3D's
`c3d-stream-formatter` repository, which is not public. The vectors are the acceptance test and
are not in this repository, so CI skips the core tests that read them. With a checkout of the
formatter, point the core tests at its vectors:

```bash
POSE_VECTORS=~/cvr/c3d-stream-formatter/vectors/pose tools/build_and_test.sh core
```

`PoseVectors.*` then encode every stored vector and compare each uncompressed body with the
reference's byte for byte, cut the six `generated/` recipes, and check the tests' own decoder
against `bad/`. The `PoseBlock*` tests need no vectors and run everywhere.

The formatter's own judge is `verify_encoder`, which takes a Python function. It drives this
encoder through `c3d_pose_encode_vector` (`core/tools/`, built with the core tests): one
`input.json` on stdin, every block on stdout as a 4-byte little-endian length and its bytes. To
run it, build the command into a host directory with the test image, then hand that command to
the checker:

```bash
IMAGE=$(docker images c3d-sdk-ros2-test --format '{{.Repository}}:{{.Tag}}' | head -n1)
mkdir -p /tmp/c3d-pose
docker run --rm -v "$PWD":/src:ro -v /tmp/c3d-pose:/b "$IMAGE" bash -c \
  'cmake -S /src/core -B /b >/dev/null && cmake --build /b -j"$(nproc)" --target c3d_pose_encode_vector >/dev/null'

FORMATTER=~/cvr/c3d-stream-formatter
PYTHONPATH=$FORMATTER PYTHONDONTWRITEBYTECODE=1 \
uv run --no-project --with numpy --with zstandard --with lz4 -- python3 -c '
import json, shlex, struct, subprocess, sys
from c3d_stream_formatter.pose.vectors import verify_encoder
def encode(inp):
    out = subprocess.run(shlex.split(sys.argv[1]), input=json.dumps(inp).encode(),
                         capture_output=True, check=True).stdout
    blobs, i = [], 0
    while i < len(out):
        (n,) = struct.unpack_from("<I", out, i); blobs.append(out[i + 4:i + 4 + n]); i += 4 + n
    return blobs
problems = verify_encoder(sys.argv[2], encode)
print("\n".join(problems) or "encoder pass rule: PASS"); sys.exit(bool(problems))
' "docker run --rm -i -v /tmp/c3d-pose:/b $IMAGE /b/tools/c3d_pose_encode_vector" \
  "$FORMATTER/vectors/pose"
```

It checks every stored vector and the `generated/` recipes; each problem names its vector. On a
Linux host with the core dependencies installed, build the command natively and pass its path
instead of the `docker run`.

### The pose upload queue and the upload check. By hand

A session uploads pose blocks through an upload queue whose rules are the platform's as well
(the formatter's `docs/pose/spec.md` §10). Two checks come from the formatter. The queue
vectors, `vectors/pose/queue/`, drive the queue class alone on a fake clock:
`PoseQueueVectors.EveryQueueCasePasses` runs every case whenever `POSE_VECTORS` is set, as above.
The upload check judges what a whole session posted. `PoseSessionTest.PoseEndToEnd` drives two
objects through manifests, re-sends, timer sends, cap sends and `End`, and with `POSE_CAPTURE`
writes every body it posted, as `part_<n>.bin`, and the send log, as `send_log.json`:

```bash
POSE_VECTORS=~/cvr/c3d-stream-formatter/vectors/pose POSE_CAPTURE=/tmp/c3d-pose-uploads \
  tools/build_and_test.sh core
uv run --no-project --with ~/cvr/c3d-stream-formatter --with pycapnp -- \
  python -m c3d_stream_formatter.pose.uploads /tmp/c3d-pose-uploads \
  --send-log /tmp/c3d-pose-uploads/send_log.json --interval-ms 1000
```

It ends `11 bodies, 0 problems, 0 warnings: PASS`. `--interval-ms` is the test session's
`pose_block_interval_s`. The session writes its send log (`Config::pose_send_log_path`) one
JSON line per block part; the test wraps the lines in the JSON list the checker reads.

## The Python suites. Every push, in CI

```bash
for suite in tools/test_*.py; do python3 "$suite"; done
```

Standard library only, with no dependency step, so an operator verifying a session never needs a
virtual environment. CI finds the suites by glob, so a new `tools/test_*.py` runs without being
listed anywhere.

## The capture tier. Every PR, in CI

`run_synthetic_integration.sh` stands up the fakes, the node and the stub gateway, drives a
session, and then asserts on what would actually have been transmitted:

- `stub_gateway.py` stands in for `data.cognitive3d.com`. It answers 200 with the
  `cvr-request-time` header the client checks for and writes every payload to
  disk as `<NNNNNN>-<stream>.json`, so a test can assert on the real wire bytes.
  It records `auth_present`, `auth_scheme`, `auth_credential_present` and
  **never the Authorization header's value**: that value is a live API key, and
  a capture directory is something people attach to bug reports.

  A body announced as `application/octet-stream` (the camera and lidar routes)
  lands as **two** files instead: the bytes as `<NNNNNN>-<stream>.bin`, and the
  usual `.json` record beside it naming that file and its size. The dispatch is
  on the content type rather than on the route, because that is what the real
  gateway dispatches on: a JSON body announced as octet-stream is a defect at the
  sender, and this tool should show it rather than hide it by re-reading the URL.
  The body is written and renamed **before** the record that names it, so a
  reader that found a record found a complete body.

  Captures are written to `<name>.partial` and renamed into place, so a reader
  globbing `*-<stream>.json` never sees a half-written file; SIGTERM and SIGINT
  are caught so a capture already in flight finishes its rename rather than
  becoming an orphan. On SIGKILL, or a write error, a `.partial` can still be
  left behind. Treat one as a failure, not as noise: it means a capture is
  missing, and a set that is silently short by one is worse than a corrupt file.
  Covered by `test_stub_gateway.py`.
- `fake_tf_publisher.py` publishes a moving `map`→`odom`→`base_link` chain with a
  deliberate relocalisation jump, and with `--mount-frame` a static tilted sensor
  mount under `map` for the scan-plane passes below.
- `fake_nav_server.py` is a scripted `NavigateToPose` server, plus battery, joint
  and diagnostics publishers with a fault that appears and clears.
- `fake_goal_driver.py` drives goals and cancels one mid-flight.
- `fake_camera_publisher.py`, `fake_pointcloud_publisher.py` and
  `fake_laserscan_publisher.py` feed the two binary streams, each faster than the
  node is configured to take so the rate gate is exercised rather than merely
  satisfied. They publish the awkward cases on purpose: a raw and a JPEG image, a
  scan carrying infinities and NaNs, and a cloud whose drawn positions repeat
  inside one voxel leaf, because a downsample pass that merges nothing cannot be
  shown to have run at all.
- `synthetic_params.yaml` and `scan_plane_params.yaml` configure the node for the
  session and for the scan-plane passes.
- `verify_scan_plane.py` is the scan-plane passes' verdict; the section below is
  what it checks.
- `capnp_dump.py` decodes the `.bin` captures into one JSON document on stdout,
  `{"files": [...]}` with an entry per input, so an assertion reads the lot in a
  single `json.load`. It is the **one** tool in `tools/` with a third-party
  dependency: a hand-written packed Cap'n Proto reader does not survive a schema
  change, and the schemas are vendored copies (`core/capnp/README.md`). The
  dependencies are declared inline (PEP 723), so

  ```bash
  uv run tools/capnp_dump.py capture/000012-camera.bin
  ```

  resolves them into a throwaway environment and the interpreter every other tool
  runs on stays clean. Everything that does not need the schemas (the planar
  point decoder, the color decoders, the name-to-stream rule) is a plain
  function there, and `test_capnp_dump.py` covers that half unconditionally and
  skips the rest when `pycapnp` is absent. That is what keeps the
  standard-library-only CI job honest rather than merely green.

  `--scan-ranges` adds each `LaserScan`'s ranges and intensities to the document.
  Off by default because a 640-ray sweep is most of the document and almost no
  assertion needs it; a reader placing a ray does.

**Why a fake action server rather than a simulator.** A simulator hands you
`SUCCEEDED` almost every time. An objective translator's bugs live in `ABORTED`,
`CANCELED`, preemption, and goals that vanish from the status array without ever
reaching a terminal state. Those are only reachable by scripting the server, so
this tier is not a poor substitute for a simulator; it is the better tool for
this particular job.

The script is its own container entry point: it resolves the same image
`build_and_test.sh` uses, re-runs itself inside it, and builds the workspace there,
so a host with no ROS runs this tier without installing anything. There is no list
of files to stage anywhere: the repository is mounted read-only and everything is
read from it. It asserts on the decoded payloads itself (record counts against the
configured rates, every time inside the run's own window, `part` monotonic from 1
per stream, the JSON and binary streams carrying one `sessionId`, and each point
block decoding to the count its own header claims) rather than leaving a directory
for a human to read. It runs on Jazzy only.

```bash
tools/build_and_test.sh synthetic                                  # how CI runs it
bash tools/run_synthetic_integration.sh                            # captures die with the container
CAPTURE=/tmp/c3d_capture bash tools/run_synthetic_integration.sh   # keep them
```

With a capture directory kept, the same bytes are readable by hand:

```bash
uv run tools/capnp_dump.py /tmp/c3d_capture/*-camera.bin | head -40
python3 - <<'PY'
import json, glob, collections
c = collections.Counter()
for f in glob.glob('/tmp/c3d_capture/*-events.json'):   # not *events*: that also matches an in-flight .partial
    for r in json.load(open(f))['body'].get('data', []): c[r['name']] += 1
print(c)
PY
```

### The scan-plane passes

The first pass runs `pose_convention: rep103`, where every axis map is the identity, so it cannot
tell the pose a `LaserScan` record carries from the pose a camera frame carries: under `rep103`
they are the same numbers. Under `unity` and `gltf_authored` the position map is a REFLECTION and
they are not: a laserscan's rays reach the wire as the sensor published them, so its record
carries the scan plane's rotation and a reader rebuilds a ray as
`rotate(pose.q, (r cos t, r sin t, 0)) + pose.position` ([data format](../docs/data-format.md)).

So the tier drives the session twice more, under `gltf_authored` and then `unity`, with the camera
and the laserscan publishing on ONE tf frame: a static `map` → `sensor_mount` from
`fake_tf_publisher.py --mount-frame`, rotated in roll, pitch AND yaw, because a level scan plane
stays level under a rule that is wrong about everything else. Its parent is the reference frame
rather than `base_link` so that the sensor's world pose is the transform that was published and
nothing else; a mount that moved with the robot would be knowable only at each record's own stamp.

`verify_scan_plane.py` then decodes both streams with `capnp_dump.py --scan-ranges` and checks, per
convention:

- the session reports the `pose_convention` the pass claims. Session properties ride the gaze
  stream, so this is read off the wire rather than trusted from the params file;
- every record places the sensor at the mapped mount position;
- the laserscan rotation is not the camera rotation on that same frame, by at least 5 degrees;
- **and the one that matters:** every sampled ray, rebuilt by the reader rule from the record
  alone, lands within 1 mm of where the published transform puts it. The expectation is computed
  from a rotation matrix written out from roll, pitch and yaw and mapped into scene axes by the
  convention's own position map, with no SDK code anywhere in it;
- as a control, that the camera's rotation would have put those same rays meters away, so a pass
  cannot come from the two rules having quietly become the same thing.

This is the only check in the tree on the CALL SITE rather than on the conversion: `colcon test`
pins the rule in `cognitive3d_ros/test/test_media.cpp` and cannot see which rule
`SessionRunner::RecordLaserScan` calls, because the generated Cap'n Proto readers are private to
`core/` and no ament test can decode a payload. Swapping that call for `ScenePose` passes the whole
ament suite and fails here.

`verify_scan_plane.py` refuses `--convention rep103` outright: a pass that cannot tell the two
rules apart would be green for a reason that has nothing to do with the rules.

With `CAPTURE` set, each pass keeps its own `scan_plane_<convention>/` subdirectory of it.

## Onboarding tools

A new integration cannot proceed without the output of these two. Both install with the
`cognitive3d_ros` package and run as `ros2 run cognitive3d_ros <tool>`, for example
`ros2 run cognitive3d_ros fetch_application_key.py`; from a checkout they run as shown here.

```bash
# an SDF world -> glTF files to upload as a scene. Needs Gazebo and assimp; no Cognitive3D code.
tools/export_world_gltf.sh my_world /tmp/scene

# your project's application key, read rather than transcribed. Needs an organization key.
uv run tools/fetch_application_key.py                 # masked listing
uv run tools/fetch_application_key.py --env-line      # C3D_APPLICATION_API_KEY=... to append
```

`export_world_gltf.sh` drives Gazebo's own `ColladaWorldExporter` and repairs the COLLADA
`<asset>`/`<up_axis>` block that it builds and never attaches
([gz-common#930](https://github.com/gazebosim/gz-common/issues/930)). That repair is the only
non-obvious step and it is the one that is unit-tested, because it is the one that can be wrong:

```bash
uv run tools/test_collada_zup_repair.py
uv run tools/test_fetch_application_key.py
```

The repair lives in `collada_zup_repair.py` rather than inline in the shell script for exactly that
reason: it can be exercised on a synthetic document, including the case no current exporter
produces (an XML declaration), which is what separates anchoring on `<COLLADA` from anchoring on
the first `>` in the file.

`gltf_bbox.py` reports orientation by walking the node hierarchy, because accessor `min`/`max` are
PRE-transform: assimp puts the Z-up correction on the root node's matrix, so a raw bounding box
calls a correctly repaired scene "on its side". Its upright verdict is a heuristic about
room-shaped geometry and says so: a near-cubic world is reported inconclusive rather than judged.

The two session verifiers, `verify_gaze_contract.py` and `verify_dynamics_level.py`, are
diagnostics that read a stored session back through an unversioned API the dashboard uses, which
may change without notice. [Doctor and verification](../docs/doctor-and-verification.md) covers
when to run them.

## Release tools

- `bump_version.sh <x.y.z>` rewrites every version declaration; `--check <x.y.z>` verifies them.
- `release/build_debs.sh <humble|jazzy> <out-dir>` builds the debs with bloom inside that distro's
  `ros-base` image, and `release/smoke_test.sh` installs them into a fresh one and checks what a
  robot would get. `release/rosdep.yaml` supplies the rosdep keys for this repository's own
  packages.
- `release/changelog_section.sh <x.y.z>` prints that version's CHANGELOG section, which becomes
  the release notes.
- `sync_capnp.sh` refreshes the vendored schemas from the canonical copies, for maintainers who
  have them (`core/capnp/README.md`).

## A note on Python in containers

Several ROS images ship a `python3` that is not the interpreter the `rclpy` C
extension was built for, which makes every Python ROS node die with
`No module named 'rclpy._rclpy_pybind11'`. Check with:

```bash
ls /opt/ros/$ROS_DISTRO/lib/python3.*/site-packages/rclpy/_rclpy_pybind11*.so
```

and match the `cpython-3XX` tag against `python3 --version`.
