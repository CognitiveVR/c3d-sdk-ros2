# Doctor and verification

`doctor` checks your configuration, credentials, network path and live ROS graph without opening a
session or uploading anything. Run it on the robot before the first launch, and again after every
change to your params file. Most integration mistakes produce no error at run time: the platform
accepts the upload with HTTP 200, and the session is empty, misplaced or filed under another
project. `doctor` is where those mistakes become visible.

After a session, the two verifiers read back what the platform stored, and
`fetch_application_key.py` removes the step where a wrong application key is usually typed.

## Run doctor

Stop the node first, then run `doctor` on the robot with the same params file, the same
environment and the same ROS domain the node uses:

```bash
ros2 run cognitive3d_ros doctor --ros-args --params-file <path/to/params.yaml>
```

`doctor` constructs the real node with `dry_run: true` and `auto_start_session: false`, configures
it, and reports what the node resolved: it cannot disagree with the node about your configuration.
It then watches tf, the topics and every configured camera and lidar for a few seconds, and usually
finishes within 20 seconds.

`doctor` checks the build of the node that the launch file runs. With `cognitive3d_ros_nav2`
installed, that is the Nav2 build, so `ros2 run cognitive3d_ros doctor` runs that package's
`doctor` with the same arguments. Its first check, `build`, names the package and every message type
the node translates; the Nav2 build lists `nav2_msgs/msg/BehaviorTreeLog`. `build` describes the
build the launch file runs, not a component you load into a container yourself. For that, read the
loaded node's configure log: its line `translating N message type(s)` names the same types.

If the node runs in a namespace, give `doctor` the same one, for example
`--ros-args -r __ns:=/robot1 --params-file <path/to/params.yaml>`. The namespace decides which
actions and standard topics are discovered, and the default `device_id`. When the robot's tf tree
is under the namespace too, add the remaps the node runs with, `-r /tf:=tf -r /tf_static:=tf_static`,
or `doctor` reads the global `/tf`.

Paste the whole output into any support request, on the Cognitive3D
[Discord](https://discord.gg/x38sNUdDRH) or to `support@cognitive3d.com`. Its first line names the
SDK version, the ROS 2 distro and the RMW. The output is safe to share as it is: it masks the
application key and never prints the organization key.

## Credentials that enable more checks

`doctor` reads `C3D_APPLICATION_API_KEY` like the node does, and fails without it. Two more
environment variables let it ask the platform about your project:

| variable | enables |
| -- | -- |
| `C3D_ORG_API_KEY` | `scene/project`: whether the scene has the version `scene_version` names, and which project it belongs to. The organization key every check below authenticates with, and `mesh/files` too. A developer key is refused. |
| `C3D_PROJECT_ID` | `key belongs to project`: whether the application key is one of this project's keys. `scene/project` compares the scene's project with it. With `robot_dynamic_object: true`, also `mesh/exists` and `object id/registered`. |

`doctor` sends the organization key only to the API host your environment names
([`C3D_DATA_HOST` and `C3D_API_HOST`](configuration.md#environment-variables), or production), and
only when `protocol` is `https`. Of the application key it prints only the last four characters,
once, on the `application key` line, and it never prints the organization key.

## Read the output

Each line is a check: a severity, the check's name and what it found. A line starting with `->`
under it says what to do. This abridged run found nothing fatal:

```text
cognitive3d_ros 1.1.0 doctor (ROS 2 jazzy, rmw_fastrtps_cpp)

  [ok  ] build                        cognitive3d_ros's node, the one the launch file runs while cognitive3d_ros_nav2 is not installed; translating 9 message type(s): action_msgs/msg/GoalStatusArray, ...
  [ok  ] configuration                accepted; scene <your-scene-uuid> version <your-scene-version>, device <your-device-id>
  [ok  ] application key              set (ending ...9y8z)
  [ok  ] wall clock                   set
  [ok  ] key/project (NOT CHECKED)    set C3D_PROJECT_ID to the project you expect
  [ok  ] scene/project (NOT CHECKED)  set C3D_ORG_API_KEY to check that the scene is in your project and has version <your-scene-version>
  [note] spool directory              /home/robot/.local/state/cognitive3d is writable, and holds 1 session(s) left open by a previous process
         -> Nothing to do -- the node closes these with an abnormalEnd reason at configure. Mentioned so an unexpected count is visible rather than silent.
  [ok  ] robot pose                   map -> base_link resolves
  [ok  ] gaze frame                   not configured; the gaze pose is the robot body's

  OK, with notes. A session opened now will record a spatial timeline.
```

| severity | meaning |
| -- | -- |
| `ok` | Nothing to do. |
| `note` | Worth reading, and not fatal: something may be wrong, or a check you asked for could not answer. |
| `FAIL` | Fix this before you open a session. |

A check that did not run carries `(NOT CHECKED)` in its name, so it cannot be read as a pass. It is
`ok` when nothing asked for it, such as `key/project` with no `C3D_PROJECT_ID` set or
`scene/project` with no `C3D_ORG_API_KEY`, and a `note` when you asked for it and it got no answer.
Read the name as well as the severity.

When the node refuses to configure, `doctor` reports the refusal and stops there: every other check
depends on a resolved configuration.

## Exit codes

| code | meaning |
| -- | -- |
| `0` | No check failed. Notes do not change the exit code. |
| `1` | At least one check failed, including a command line that does not parse, a node that cannot be constructed, and a refused configuration. |

The exit code cannot tell a check that passed from one that did not run, so a script that gates on
an optional check should also look for `(NOT CHECKED)` in the output. To stop a start-up script
when `doctor` fails:

```bash
ros2 run cognitive3d_ros doctor --ros-args --params-file <path/to/params.yaml> || exit 1
```

## Checks

`doctor` prints the checks in this order. The remedy it prints for each one is more specific than
this table.

| check | FAIL when | note when |
| -- | -- | -- |
| `build` | Never. Names the package whose node `doctor` built and every message type it translates. | `cognitive3d_ros_nav2` is installed and its `doctor` could not be started, so this report is on the plain node, which lacks the Nav2 translators. Install `cognitive3d_ros_nav2` at the same version as `cognitive3d_ros`. Or the ament index cannot be read, most often because `AMENT_PREFIX_PATH` is unset, so which build the launch file runs is unknown. Source the ROS 2 setup file. |
| `command line` | The arguments after `--ros-args` do not parse, most often a wrong `--params-file` path. | Never. |
| `parameters` | The node cannot be constructed: a value of the wrong type (`scene_version: 1`), or an empty list (`cameras: []`). Names the parameter and the fix. | Never. |
| `another node` | A node with `doctor`'s name, `/cognitive3d` by default, is already running. Watched for the whole run, because `doctor` also serves `~/start_session` once configured, and a session opened by the other node could be left unended. | Never. |
| `configuration` | The node refuses to configure. Every problem is listed at once. | Never. On success it names the scene, the scene version and the device. |
| `application key` | `C3D_APPLICATION_API_KEY` is not set. | Never. |
| `wall clock` | The system clock reads earlier than 2026-01-01, so it has not been set. The node refuses every session start until it is, because a session stamped from this clock is accepted and never appears. A clock that is set but stale, as on a board that restores its last saved time at boot, does not fail this check: see [An unset wall clock](sessions.md#an-unset-wall-clock). | Never. |
| `environment` | Never. Names the environment and both hosts, and any `gateway` override. | Never. |
| `gateway` | The data host is unreachable; answers 401 or 403 to a probe that carries no credentials, so something in front of it requires them; or answers 200 without the `cvr-request-time` header, which is a captive portal or an intercepting proxy, not the platform. Never for a 404, which is the data host's answer to this probe: the line says `reachable (HTTP 404, the expected answer to this probe)`. | It answers 429 or a 5xx. Data spools and is retried. |
| `key belongs to project` | The application key is not one of `C3D_PROJECT_ID`'s keys. The session would be stored, complete and with no error, in whichever project the key belongs to. | The check could not answer. Named `key/project (NOT CHECKED)` when it could not run: no `C3D_ORG_API_KEY`, `protocol` not `https`, or an application key too short to look up. Without `C3D_PROJECT_ID` it has that name and is `ok`. |
| `scene/project` | The scene belongs to another project than `C3D_PROJECT_ID`: the platform would create an empty shadow scene with this id in your project and record the session against it. Or the scene has no version numbered `scene_version`; when the value is the scene's numeric version id, it names the version number to use instead. Or the platform shows no scene with this id to the organization key that `key belongs to project` has just proved against `C3D_PROJECT_ID`: the id does not exist, or belongs to another organization. | `C3D_PROJECT_ID` is not set, so the scene's project, which it names, was not compared with yours. Or the organization key sees no scene with this id, and with no `C3D_PROJECT_ID` the key itself is unproven. Or the platform did not answer, refused the organization key, or answered without a readable scene. Named `scene/project (NOT CHECKED)` when `protocol` is not `https`. Without `C3D_ORG_API_KEY` it has that name and is `ok`. |
| `spool directory` | The spool directory is not writable, or its `.lock` file is a link or not a plain file. | Another process holds the directory, named by its process id: every other node given this directory is refused at configure. Or it holds sessions a previous process left open, which the next configure closes. |
| `robot pose` | Neither `reference_frame` nor `fallback_reference_frame` resolves to `robot_frame`. Without a pose the session has no spatial timeline. | Only the fallback resolves. The path is then drawn from wherever the robot booted. `doctor` waits up to 10 seconds for tf, and gives `reference_frame` its `reference_frame_grace_s`. |
| `gaze frame` | `gaze_frame` is set and does not resolve. The node would record the robot's pose instead, with no error in the session. | Never. |
| `pose channel` | Never. | The frame in force is `odom` or `map` and its samples carry no height change and no tilt. The standard producers of these frames are planar, so height, roll and pitch stay zero even on a ramp, and on a flat floor that looks correct. See [Coordinates](coordinates.md). |
| `clock` | `use_sim_time` is `true` and nothing publishes `/clock`. | `/clock` is published and `use_sim_time` is `false`. |
| `pose/convention` | `pose_convention` is not set. It cannot check that the value matches your scene. | Never. |
| `robot dynamic object (NOT CHECKED)` | Never. Printed when `robot_dynamic_object` is `false`, in place of the next three checks. | Never. |
| `mesh/exists` | `robot_mesh` is not an object of `C3D_PROJECT_ID`. Session Replay would show no robot. A scene in another project also produces this; `scene/project` says whether that is the cause. | It could not check: it needs `C3D_PROJECT_ID`, `C3D_ORG_API_KEY` and `protocol: https`, or the platform did not answer. A pass confirms the project, not the scene version. |
| `object id/registered` | Never. | `robot_object_id` is not a registered object id in the project. Session Replay still shows the robot, but the uploaded object's initial position, rotation, scale and thumbnail never apply. Also a note when it could not check, as for `mesh/exists`. |
| `mesh/files` | `robot_mesh` is not an object of the scene version; it has no files; or its files do not include `<robot_mesh>.gltf`. Session Replay would show no robot. Or the scene has no version numbered `scene_version` and `scene/project` could not read the scene to say so. | It could not check: it needs `C3D_ORG_API_KEY` and `protocol: https`, or the platform did not answer, or the scene has no version numbered `scene_version`, which `scene/project` reports. |
| `topics` | Never. | The allowlist is empty (it says what the session still records), or allowlisted topics are not on the graph (it names every one). |
| `topics/qos` | Never. | The publishers of an allowlisted topic disagree on durability or reliability, or the subscription the node would request is incompatible with one of them; it names each publisher node with its reliability and durability, and the override that fixes it, such as `topic_config.joint_states.qos.durability: volatile`. The node chooses a topic's QoS from the publishers it has discovered when it subscribes, so publishers that disagree connect on one start and not on the next. See [Ingest and QoS](recording.md#ingest-and-qos). When the publishers disagree and your override already asks for the weaker value, the check is `ok` and names the topic, its publishers and the override that reaches every one. Named `topics/qos (NOT CHECKED)` when no allowlisted topic is on the graph. |
| `actions` | Never. | An action server on the graph will not be recorded; it names each one and why. An action that is not recorded emits no events. Nav2's planner, smoother and controller actions are listed apart, as left out by design: they run on every replan. It also lists the battery and diagnostics topics taken by type. |
| `shared series` | Never. | Printed only when two recorded topics of one odometry, IMU, joint state or laser scan type would record into the same series. Keep one topic of each of these types. |
| `sensor/<name>` | The topic of a [described sensor](configuration.md#described-sensors) is of another type than its `kind` reads: `std_msgs/msg/Float32` for a `cone_rangefinder`, `std_msgs/msg/Int32MultiArray` for a `binary_array`. The readings would be recorded under other series names, or none, and the description would name a series that never exists. The node logs the same error when it subscribes the topic, once the session is open. | The topic has no publisher, so the sensor records nothing until one starts. A pass names the kind, the topic, its type and the series the readings are recorded under. |
| `camera encoder` | Cameras are configured and the binary named by `ffmpeg_path` cannot run, does not answer its version probe, or lists no `libx264` encoder. No camera can record, and configure refuses for the same reason. The remedy is `sudo apt install ffmpeg`, or `ffmpeg_path` set to a binary that runs. | Never. A pass names the binary and its version line. See [Camera and lidar](camera-and-lidar.md#how-a-camera-is-encoded). |
| `camera/<name>`, `lidar/<name>` | The topic's type is not one the stream takes; there is no frame to pose it in; its frame does not resolve; or a real sample cannot be prepared (a camera's) or encoded (a lidar's). Each of these makes the stream record nothing. | The topic is not on the graph, or published nothing while `doctor` watched, which a QoS mismatch also causes. A pass names the type, QoS and frame. For a camera it says what one frame prepares to, an estimated rate and the chunk length the node would choose, such as `one frame prepares to 640x480; about 150 KB/s estimated ... chunks of 3 s, about 450.0 KB each`, from the resolution, `crf` and `rate_hz`, capped by `max_bitrate_kbps` when set: an estimate from measurements on recorded footage, not a measurement of this camera. It also names the bitrate cap the encoder would run with: `max_bitrate_kbps`, the default cap where 28 frames of that size do not fit in `replay_max_mb`, or none. It is FATAL when not even a 1-second chunk fits under 6 MiB, because the node would refuse the camera at its first frame. For a lidar it gives the bytes per second of one real message at `rate_hz`. |
| `camera/<name> field of view` | Never. | The camera has no field of view source, so its chunks carry none and Session Replay draws no frustum for it: `hfov_degrees` and `vfov_degrees` are unset, and its CameraInfo topic is switched off, is not on the graph, is not a `sensor_msgs/msg/CameraInfo`, published nothing while `doctor` watched, or carries no usable intrinsics, as a driver with no calibration publishes. A pass names the angles and where they come from. |
| `lidar/<name> optical frame` | Never. | A point cloud posed against an optical frame carries its points in the body frame, so it would be stored on its side. Set `lidar.<name>.frame`. See [Camera and lidar](camera-and-lidar.md). |
| `camera/lidar bandwidth` | Never. | The configured cameras and lidars add up to more than 2 MiB per second. Nothing downstream limits it. |

## What doctor does not check

- Whether `pose_convention` matches how your scene was authored. See [Coordinates](coordinates.md).
- Remote variables and A/B tests. See [Remote variables](remote-variables.md).
- A `series` the topic's type ignores. The node warns when it subscribes the topic, and its
  configure log prints the plan it resolved; see
  [Configuration](configuration.md#per-topic-rule-keys).
- That the platform stores a session. `doctor` opens none; verify one after a run.

## Wait for the session to appear

A session takes roughly 1 to 3 minutes to appear on the Cognitive3D dashboard after its last data
is accepted. Poll rather than check once: a check made right after the session ends reports a false
negative. The node spools data and uploads it in the background, so on a slow or paused uplink the
last data is accepted later than the session's end. [Sessions](sessions.md) explains the spool.

## Verify a stored session

The two verifiers read a session back from the platform and check what was stored, not what the
node believes it sent. They are diagnostics on an unversioned API: they read through an API the
Cognitive3D dashboard uses, which may change without notice.

Both read `C3D_PROJECT_ID` and `C3D_ORG_API_KEY` from the environment, and
[`C3D_DATA_HOST` and `C3D_API_HOST`](configuration.md#environment-variables) for a stack other
than production; `--project <id>` overrides `C3D_PROJECT_ID` for either. Both exit `0` on `PASS` and `1` on `FAIL`, including a missing variable or a
failed request. Take the session id from the `~/start_session` response, the node's log, or the
dashboard.

### Check the gaze contract

`verify_gaze_contract.py` checks every stored gaze sample against what this SDK sends: a position
and a unit rotation, and no gaze point `g`. Pass the session's `pose_convention`, which the session
records as its `c3d.ros.pose_convention` property:

```bash
ros2 run cognitive3d_ros verify_gaze_contract.py <session-id> --pose-convention gltf_authored
```

It fails when a sample carries a gaze point, or when more than `--max-unusable-fraction` (default
`0.01`) of the samples have no usable position or rotation.

It also reads two things from the rotations, and reports them without failing the run unless asked:

- **Level:** the angle from the rotation's up axis to world up. `--require-level` fails the run
  when it is more than `--max-up-degrees` (default `5.0`).
- **Heading:** the angle from the rotation's forward axis to the direction the robot traveled, on
  the horizontal. `--require-heading` fails the run when the 95th percentile is more than
  `--max-heading-degrees` (default `25.0`), when more than `--max-reversing-share` (default
  `0.25`) of the segments travel backwards, or when there are fewer than
  `--minimum-forward-segments` (default `20`) forward segments of at least
  `--minimum-travel-meters` (default `0.10`), so drive the robot.

Pass both for a ground robot whose gaze frame is its body. A `gaze_frame` that legitimately pitches
or rolls fails the level check, and one that turns away from the path, such as a camera on a
pan-tilt head, fails the heading check.

Take `--pose-convention` from the session's `c3d.ros.pose_convention` property. `unity` and
`gltf_authored` sessions are checked alike, so the flag cannot tell those two apart, but `rep103`
mistaken for either of them, or either of them for `rep103`, reads wrong, and not the same way in
both directions:

- A `rep103` session checked as `unity` or `gltf_authored` reads every sample's up axis as its
  forward axis: the heading check reads 90 degrees off on every segment it finds.
- A `unity` or `gltf_authored` session checked as `rep103` reads by the direction the robot drove:
  as a correct heading, as reversing, or with no segments at all.

Either way, the level angle follows the robot's heading rather than any tilt, and without
`--require-heading` and `--require-level` the run passes.

### Check the Dynamic Object's orientation

`verify_dynamics_level.py` checks a session recorded with `robot_dynamic_object: true`. A default
session stores no Dynamic Object, and the tool exits `FAIL no dynamics samples stored`, which
reports the configuration rather than a fault.

```bash
ros2 run cognitive3d_ros verify_dynamics_level.py <session-id>
```

It checks two things in the frame Session Replay draws in:

- **Level:** every stored rotation keeps the mesh's up axis within `--max-degrees` (default `1.0`)
  of world up.
- **Heading:** the mesh faces its own path. It needs at least `--minimum-forward-segments` (default
  `20`) forward segments of at least `--minimum-travel-meters` (default `0.10`), so drive the
  robot. It fails when the 95th percentile is more than `--max-heading-degrees` (default `25.0`)
  off, or when more than `--max-reversing-share` (default `0.25`) of the segments render the mesh
  traveling backwards.

It reads rotations only, never heights. A drive that is mostly reversing fails like a mesh turned
around; give the drive a turn instead of raising the bound. On a slope, a correct climb and a
broken up axis fail alike, so a FAIL there is a prompt to look in Session Replay.

### What the verifiers cannot see

- Both compare a session with itself. A planar pose source renders perfectly level and along its
  own path, so both pass on a session whose height and tilt are always zero. `doctor`'s
  `pose channel` check is what says whether the source can carry them.
- Neither compares the session with your scene. A path in the wrong place, from a wrong
  `pose_convention` or a pose recorded in `odom`, passes both.
- Neither checks sensors, events, objectives, cameras or lidars.

## Check Session Replay

Open the session in Session Replay and check, against what the robot did:

- The path lies on the floor plan, where the robot drove.
- The robot, or its gaze line, faces the direction of travel.
- The sensor series you allowlisted are present under their own names, not only the `c3d.*` series.
- The events and action outcomes you expect are there.
- Each camera and lidar shows frames.

## Fetch your application key

A wrong application key is the mistake the platform never reports: a key from another project
uploads with HTTP 200 into that project, and the session never appears in yours.
`fetch_application_key.py` reads the key from the platform instead of a transcription.

It reads `C3D_PROJECT_ID` and `C3D_ORG_API_KEY`, and the host pair for a stack other than
production. Without arguments it prints the project's application keys, masked, and marks the one
it chose: the most recently created valid key. Any valid key of a project routes to that project.

```bash
export C3D_PROJECT_ID=<your-project-id>
export C3D_ORG_API_KEY=<your-organization-key>
ros2 run cognitive3d_ros fetch_application_key.py
```

`--env-line` prints `C3D_APPLICATION_API_KEY=<key>` and nothing else, unmasked, for an environment
file the node's process loads:

```bash
ros2 run cognitive3d_ros fetch_application_key.py --env-line >> <path/to/cognitive3d.env>
```

It exits `1` when the platform refuses the organization key (a developer key is refused), when the
project has no valid application key, or when the API does not answer. Keep the environment file
out of version control.
