---
name: integrate-cognitive3d-ros2
description: Use when adding Cognitive3D telemetry (the cognitive3d_ros node, the Cognitive3D SDK for ROS 2) to an existing ROS 2 robot or workspace - surveying the live robot, choosing its parameters, wiring it into launch, running doctor and checking the first session. Not for developing the SDK itself.
---

# Integrate the Cognitive3D SDK for ROS 2

This procedure adds the Cognitive3D node to a robot that already works: its own tf tree, topic
names and launch files. It targets SDK 1.0.0, and every link below points at that version's docs.

Most mistakes in this integration produce no error. The platform accepts the session with HTTP 200,
and it is empty, misplaced or filed under another project. So measure the robot before deciding
anything, and verify the result against the platform rather than against what was configured.

## Operating rules

- Measure the live robot, and take every value from what you measured. Never copy a topic, frame or
  convention from an example, this file's template included. If the robot, or its simulator, is not
  running, say so and stop: a params file written against a guessed graph looks configured and is
  wrong.
- Never read, print, store or commit a credential. Ask the human to export the keys, and refer to
  them by variable name only: `C3D_APPLICATION_API_KEY`, `C3D_ORG_API_KEY`. A key never goes in a
  params file, a launch file or a report.
- Check the installed version first, with `ros2 pkg xml -t version cognitive3d_ros`. If it is not
  1.0.x, tell the human, and follow the docs at that version's tag instead of these links.
- Do not move the robot yourself. Ask the human to drive it, or get their explicit go-ahead.
- Report every `doctor` FAIL and note to the human verbatim, never as a summary.
- Add the node to the robot's own application launch, not to a vendor bringup the human may later
  update. Change nothing else in their stack.

## Step 1: Survey the robot

Run these with the same `ROS_DOMAIN_ID` and RMW the node will use, from a shell where ROS 2 is
sourced:

```bash
ros2 pkg xml -t version cognitive3d_ros
ros2 pkg prefix cognitive3d_ros_nav2            # fails when the Nav2 package is not installed
printenv ROS_DISTRO RMW_IMPLEMENTATION ROS_DOMAIN_ID
ros2 node list
ros2 topic list -t
ros2 action list
ros2 run tf2_tools view_frames --wait-time 15   # writes frames_<time>.gv, a readable text graph
timeout 15 ros2 run tf2_ros tf2_echo map base_link
ros2 param get <a-node-from-the-list> use_sim_time
ros2 topic info --verbose <each-topic-to-record>   # its publishers, and the QoS each offers
```

Discovery and tf take time to fill, and a slow simulation stretches both. Read a name missing from
the first answer as not seen yet, not as absent:

- On a large graph, such as Nav2 with a simulator, the first `ros2 node list` and
  `ros2 topic list -t` can show a fraction of it. Run both again about 20 seconds later, and use the
  second answer.
- `tf2_echo` prints `Invalid frame ID "map" … frame does not exist` while its listener fills,
  even when the frame exists. A transform prints as `At time …` and `- Translation: […]`. Only
  a run that prints no transform in its 15 seconds means there is no `map`, and doctor's
  `robot pose` check in Step 5 is the authority: `map -> <robot_frame> resolves`, or a note that it
  falls back to `odom`.
- When `use_sim_time` is true, measure the simulator's real-time factor from `/clock`, with two
  readings about 30 seconds apart. The simulated seconds over the wall-clock seconds is the factor:

  ```bash
  sim0=$(ros2 topic echo --once /clock --field clock.sec | head -n 1); wall0=$(date +%s)
  sleep 30
  sim1=$(ros2 topic echo --once /clock --field clock.sec | head -n 1); wall1=$(date +%s)
  echo "$((sim1 - sim0)) simulated seconds in $((wall1 - wall0)) wall-clock seconds"
  ```

Answer each question below from the output, and write the answer down with the command that gave
it. The hand-off report in Step 7 needs both.

| question | what it decides |
| -- | -- |
| Which distro, RMW and SDK version? | Whether these links apply; the report header |
| Is there a `map` frame, and what publishes it? | `reference_frame`. Without one the path is drawn from wherever the robot booted |
| What is the robot body frame called (`base_link`, `base_footprint`, a namespaced name), and which is the parent? | `robot_frame` |
| Which odometry, IMU, joint state, scan and plain sensor topics exist, and with what types? | `topics`: one topic per type for odometry, IMU, joint states and scans |
| Does a topic to record have more than one publisher, and do they differ in durability or reliability? | A `topic_config.<key>.qos` override for that topic |
| Is the robot in a namespace, is its tree on `/tf` or under the namespace, and is more than one robot on the graph? | The node's namespace and tf remaps, and `ROS_DOMAIN_ID` isolation |
| Is there a forward-facing camera, with a body frame and an `_optical_frame`? | `gaze_frame` and `gaze_frame_convention` |
| Which action servers run, and are any not Nav2's or the iRobot Create 3's? | `action_names`, only for custom actions |
| Is Nav2 running, and is `cognitive3d_ros_nav2` installed? | Which executable runs; `/behavior_tree_log` in `topics` |
| Is `use_sim_time` true, and what real-time factor does the simulator run at, measured from `/clock`? | `use_sim_time`, and `sim_time_on_wire` when the factor stays below 1 |
| Does the robot run in a container, under systemd or in a component container? Can you recreate that container, or only `docker exec` into it? | How the node is started and stopped, and where the spool lives |

## Step 2: Decide

Make each decision from the survey, in one or two sentences, and record why.

- **The scene.** Ask the human where it came from and for its scene id and version number (the
  `versionNumber`, not the `versionId`). It must belong to the same project as the application key;
  with `C3D_ORG_API_KEY` set, doctor's `scene/project` check confirms the version number in Step 5,
  and with `C3D_PROJECT_ID` set as well, the project. See [Scenes](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/scenes.md).
- **The pose convention.** `gltf_authored` for a scene exported from ROS or Gazebo as glTF,
  including with `export_world_gltf.sh`; `unity` for a scene authored in Unity. Ask, never infer: a
  wrong choice is invisible in every check except Session Replay. See
  [Choosing a pose convention](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/coordinates.md#choosing-a-pose-convention).
- **The frames.** `reference_frame` stays `map` when the survey found one. `robot_frame` is the
  frame the body is built around, normally `base_link` even where a `base_footprint` exists, as
  either parent or child; see
  [Choosing the body frame](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/coordinates.md#choosing-the-body-frame).
  With no `map`, once doctor's `robot pose` check in Step 5 confirms it, say so to the human as a
  finding: see
  [When the robot has no map frame](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/coordinates.md#when-the-robot-has-no-map-frame).
- **The gaze frame.** Leave `gaze_frame` empty unless a forward camera's viewpoint matters. A body
  frame or an `_optical_frame` both work; check the convention the node logs at configure. See
  [The gaze channel](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/coordinates.md#the-gaze-channel).
- **The allowlist.** List the topics the survey found, by their real names. Leave out `/tf`,
  `/tf_static`, images and point clouds. Batteries, diagnostics and the Nav2 and Create 3 actions
  under the node's namespace need no entry. Give each `Float32` or `Int32MultiArray` topic a
  `series` that names its unit. See
  [The allowlist](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/recording.md#the-allowlist).
- **Sensor descriptions.** For an allowlisted rangefinder (`Float32`) or one-bit array
  (`Int32MultiArray`), a `sensors` entry records its kind, cone, range and mount as session
  properties, so Session Replay can draw it. Session Replay draws a cone only for a
  `cone_rangefinder` named `ultrasonic`: name the rangefinder that, or it is recorded and drawn with
  no cone. Never write `c3d.` keys into `session_properties`: configure refuses them. Provisional.
  See [Described sensors](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/configuration.md#described-sensors).
- **QoS.** For a topic whose publishers differ in durability, as `/joint_states` often does under
  ros2_control (`joint_state_broadcaster` transient-local, `joint_state_publisher` volatile), set
  `topic_config.<key>.qos.durability: volatile`; for one whose publishers differ in reliability,
  `qos.reliability: best_effort`. Otherwise the node can subscribe before it sees the second
  publisher and never receive from it. See
  [Ingest and QoS](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/recording.md#ingest-and-qos).
- **Actions.** Set `action_names` only for a custom action. It replaces the built-in list, so keep
  the defaults the robot uses beside it, and never set it to `[]`. See
  [Actions](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/recording.md#actions).
- **Sessions.** Keep `auto_start_session: true` unless a mission or task orchestrator owns the
  session boundaries. If the project runs A/B tests, the orchestrator passes a new UUID as
  `participant_id` for every run. Leave `session_name` unset: set as a parameter, it names every
  session alike, and a per-run name goes in the `~/start_session` request. See
  [Sessions](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/sessions.md#starting-a-session-from-another-node).
- **Cameras and lidars.** Off unless the human asks. Each is a name with its own topic, sized by
  `doctor`. A camera's field of view lets Session Replay draw its frustum. It comes from the
  driver's CameraInfo when that is calibrated; when `doctor`'s `camera/<name> field of view` says
  there is no source, set `camera.<name>.hfov_degrees` and `vfov_degrees` from the datasheet or a
  measurement, both or neither. See [Camera and lidar](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/camera-and-lidar.md).
- **The robot's Dynamic Object.** Leave `robot_dynamic_object` off. Session Replay draws the robot
  from the pose timeline, and turning it on needs a mesh upload first.
- **The device id.** A stable name unique to this robot.

## Step 3: Write the params file

Copy `params.template.yaml` from this skill's directory to `~/cognitive3d/params.yaml`, the path
the SDK's docs use, unless the human chooses another, and replace every placeholder with a value
from Step 2. Delete each optional block that does not apply rather than leaving it empty: an empty
YAML list stops the node at startup. Keep the `/**` key and the quotes around `scene_version`. [Configuration](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/configuration.md)
documents every parameter.

## Step 4: Wire it into launch

The shipped launch file picks the Nav2 build of the node when `cognitive3d_ros_nav2` is installed,
and `autostart:=true` configures and activates the node:

```bash
ros2 launch cognitive3d_ros cognitive3d.launch.py params_file:=<path/to/params.yaml> autostart:=true
```

From the robot's own Python launch file, include it rather than declaring the node yourself:

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

cognitive3d = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("cognitive3d_ros"), "launch", "cognitive3d.launch.py"])),
    launch_arguments={"params_file": "<path/to/params.yaml>", "autostart": "true"}.items(),
)
```

- The node needs `C3D_APPLICATION_API_KEY` in its own environment: exported in the launching shell,
  `Environment=` or `EnvironmentFile=` in a systemd unit, or `-e` on a container.
- Under systemd or in a container, follow
  [Running in a container or under systemd](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/sessions.md#running-in-a-container-or-under-systemd):
  an explicit `spool_dir` on persistent storage, the node in the container's main process tree,
  and `--network=host --ipc=host`. When the robot's stack is a container you can only
  `docker exec` into, follow
  [A robot stack in a container you cannot recreate](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/sessions.md#a-robot-stack-in-a-container-you-cannot-recreate):
  stop the node with SIGINT to the launch process, never by stopping the container, and tell the
  human where the spool lives and whether it survives the container being replaced.
- As a composable component, load it into `component_container_isolated` or a container of its
  own. Never into a shared `component_container`: its one single-threaded executor stalls every
  component in it while the node fetches remote variables at session start (up to
  `remote_variables_timeout_s`), drains uploads at cleanup and shutdown, or waits on a camera or
  lidar frame's transform. Not `component_container_mt` either, where a lifecycle transition can
  wait seconds behind busy ingest. On a Nav2 robot the component is
  `cognitive3d_ros_nav2::Cognitive3DNode`; `cognitive3d_ros::Cognitive3DNode` has no behavior tree
  translator.
- On a namespaced robot, run one node per robot in that robot's namespace: wrap the include in
  `GroupAction([PushRosNamespace("<robot-namespace>"), cognitive3d])`, from `launch.actions` and
  `launch_ros.actions`. `PushRosNamespace` does not move `/tf`: when the survey found the tree under
  the namespace (`/<robot-namespace>/tf`), add `SetRemap("/tf", "tf")` and
  `SetRemap("/tf_static", "tf_static")` to the group, as the robot's own nodes are remapped. Give
  `doctor` the same namespace and remaps (`-r __ns:=/<robot-namespace> -r /tf:=tf
  -r /tf_static:=tf_static`), and use `/<robot-namespace>/cognitive3d` wherever this file says
  `/cognitive3d`.

## Step 5: Run doctor

Stop the node, ask the human to export `C3D_APPLICATION_API_KEY`, `C3D_PROJECT_ID` and
`C3D_ORG_API_KEY`, and run `doctor` with the same params file, environment, domain and namespace
the node uses:

```bash
ros2 run cognitive3d_ros doctor --ros-args --params-file <path/to/params.yaml>
ros2 run cognitive3d_ros doctor --ros-args -r __ns:=/<robot-namespace> --params-file <path/to/params.yaml>
```

It exits `1` on any FAIL. Fix every FAIL and run it again. A check whose name ends in
`(NOT CHECKED)` did not run, even when it reads `ok`: without `C3D_PROJECT_ID` nothing checked that
the key belongs to the project, and without `C3D_ORG_API_KEY` the `scene/project` check, which
catches a scene from another project and a version id given as `scene_version`, does not run. A
`topics/qos` note names the QoS override to add. On a Nav2 robot, `doctor` checks the Nav2 build of
the node the launch file runs and says so; for a component you load yourself, its configure log line
`translating N message type(s)` must name `nav2_msgs/msg/BehaviorTreeLog`. Read every note to the
human verbatim. See
[Doctor and verification](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/doctor-and-verification.md#checks).

## Step 6: Record and verify the first session

1. Ask the human to drive a short route along a wall or corridor they can recognize in the scene,
   with at least one turn, while the node is active. The activation log line names the session id.
   Warnings that allowlisted topics are not on the graph, or that there is no pose yet, are normal in
   the first seconds. What says recording works is the report line the node logs every 10 seconds:
   `frame=` names the reference frame and `poses=` rises. See
   [The report line](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/sessions.md#the-report-line); its `events=` counts only events
   published on `~/events`, never objectives.
2. End the session with `ros2 lifecycle set /cognitive3d deactivate`, and leave the node running
   for a minute so the spool uploads.
3. Wait 1 to 3 minutes, then check what the platform stored, with the session's own convention:

   ```bash
   ros2 run cognitive3d_ros verify_gaze_contract.py <session-id> --pose-convention <the-pose-convention>
   ```

   For a ground robot with no `gaze_frame`, add `--require-level --require-heading`, and drive
   the robot forward for at least two meters first.

   Run `verify_dynamics_level.py <session-id>` only when `robot_dynamic_object` is true. Both
   verifiers are diagnostics on an unversioned API, and both compare the session with itself.
4. Ask the human to check Session Replay: the path runs along the wall they drove, the robot faces
   its direction of travel, the allowlisted sensors appear under their own names, and the expected
   objectives are there. Only this catches a wrong pose convention or an `odom` origin, and no tool
   reads sensors, events or objectives back from a stored session. See
   [Check Session Replay](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/doctor-and-verification.md#check-session-replay).

## Step 7: Hand off

Give the human a report in this shape, with no credential in it:

```text
Cognitive3D integration: SDK <version>, ROS 2 <distro>, <rmw>, domain <id>
Params file: <path>; launched by: <how>

Decisions, each with the measurement behind it:
- scene <scene_id> version <n>, from <source>; project checked by <doctor scene/project, or how>
- pose_convention <value>, because the scene was <how authored>
- reference_frame <value> (map present: <yes/no>), robot_frame <value>
- gaze_frame <value or none>
- topics <list>, each seen in `ros2 topic list -t` as <type>; QoS overrides <list and why, or none>
- actions: recorded <list>; not recorded <list and why>, from doctor's actions check
- sessions <automatic or orchestrated>; cameras, lidars, robot_dynamic_object <values>

doctor: exit <code>. Every FAIL and note, verbatim:
<lines>

First session <id>: verify_gaze_contract <PASS or FAIL>; Session Replay checked by <who>: <what>

Not verifiable from the data:
- height and tilt, when the pose source is planar (doctor's pose channel check)
- sensors, events and objectives, until a person checks the dashboard
- the pose convention and the map origin, until a person checks Session Replay
```

## Silent failures to keep in mind

Each is explained at the link. If the docs are unreachable, the line is the rule.

- A params file not keyed `/**` applies nothing to a node of another name. See [the params file is ignored](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/troubleshooting.md#the-node-ignores-the-params-file).
- An application key or scene id from another project files the session elsewhere, or in an empty scene. See [an empty world](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/troubleshooting.md#the-session-is-in-an-empty-world-or-the-scene-looks-wrong).
- The wrong `pose_convention` turns the whole path 90 degrees, and no tool reports it. See [the robot is in the wrong place](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/troubleshooting.md#the-robot-is-in-the-wrong-place).
- With no `map`, the path is drawn from the robot's boot pose. See [no map frame](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/coordinates.md#when-the-robot-has-no-map-frame).
- Height and tilt read zero on a standard mobile stack; a constant zero proves nothing. See [planar poses](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/coordinates.md#planar-poses).
- A missing or misnamed topic leaves only the `c3d.*` series, which still look like sensors. See [sensors are missing](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/troubleshooting.md#sensors-are-missing).
- An action that is not enrolled records nothing, which looks like an action that never ran. See [actions](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/recording.md#actions).
- Objectives keyed on `<action>.started` match nothing on Nav2 or Create 3; use `.executing`. See [objectives](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/recording.md#objectives).
- An unresolved `gaze_frame` or a wrong `gaze_frame_convention` misaims the ray with no error. See [the gaze is misaimed](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/troubleshooting.md#the-gaze-points-at-the-floor-or-90-degrees-off).
- A point cloud whose header names an optical frame can be stored on its side. See [the wrong header frame](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/camera-and-lidar.md#a-point-cloud-whose-header-names-the-wrong-frame).
- A refused key pauses uploads and keeps the data; nothing appears until it is fixed. See [uploads stall or pause](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/troubleshooting.md#uploads-stall-or-pause).
- A container without `--ipc=host` subscribes and receives nothing. See [run in a container](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/install.md#run-in-a-container).
- For A/B tests: the variable's own probability at 0, and a new `participant_id` per run. See [A/B test arms are wrong](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/troubleshooting.md#ab-test-arms-are-wrong).

For anything else, start from [Troubleshooting](https://github.com/CognitiveVR/c3d-sdk-ros2/blob/v1.0.0/docs/troubleshooting.md).
