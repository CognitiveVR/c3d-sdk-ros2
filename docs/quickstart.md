# Quickstart

This page takes a robot that already runs ROS 2 from an installed SDK to a first session on the
Cognitive3D dashboard. Every command runs on the robot, or on a machine on the robot's ROS graph,
in a shell where ROS 2 is sourced:

```bash
source /opt/ros/jazzy/setup.bash      # or humble
```

## Before you start

- The SDK is installed: [Install](install.md).
- You have a Cognitive3D project, and access to its keys on the Cognitive3D dashboard.
- The robot is running and publishes tf. `ros2 topic list` in your shell shows its topics.

Cognitive3D uses three kinds of key, and they are not interchangeable:

| key | what it is for | where the SDK reads it |
| -- | -- | -- |
| application key | Uploading sessions. The node runs on it. | `C3D_APPLICATION_API_KEY`, read by the node and `doctor` |
| developer key | Uploading scenes and robot meshes, in the Upload Web App | not read by the SDK |
| organization key | Reading your project back from the platform | `C3D_ORG_API_KEY`, read by `doctor`, the verifiers and `fetch_application_key.py` |

## Step 1: Get your application key

Read the key from the platform rather than copying it by hand. A key from another project uploads
without an error, and the session lands in that project instead of yours. With your project's id and
an organization key exported, `fetch_application_key.py` writes the key to a file only you can read:

```bash
export C3D_PROJECT_ID=<your-project-id>
export C3D_ORG_API_KEY=<your-organization-key>
(umask 077 && ros2 run cognitive3d_ros fetch_application_key.py --env-line > ~/cognitive3d.env)
```

Load the file into every shell that runs the node or `doctor`. The node reads the key from its
environment only, never from a params file:

```bash
set -a && . ~/cognitive3d.env && set +a
```

If you copy the key from the dashboard instead, export it as `C3D_APPLICATION_API_KEY` the same
way. Keep `C3D_PROJECT_ID` and `C3D_ORG_API_KEY` exported in this shell either way: `doctor` uses
them in Step 4 to check that the key belongs to your project, and the verifier uses them in Step 6.
The node itself never reads them. [Doctor and verification](doctor-and-verification.md#fetch-your-application-key)
describes the tool.

## Step 2: Choose the scene and the pose convention

Every session is recorded against one version of one scene. You need its scene id, a UUID, and its
scene version, a number: the `sceneId` and `versionNumber` the Upload Web App reports, or the values
the Cognitive3D dashboard shows for a scene your project already has. [Scenes](scenes.md) covers
getting a scene, including exporting a Gazebo world. Use a scene from the same project as the
application key: the platform accepts a scene id from another project and records the session into
an empty scene. With `C3D_ORG_API_KEY` exported, `doctor` checks the scene's project and version
number in Step 4; see
[Confirm the scene belongs to your project](scenes.md#confirm-the-scene-belongs-to-your-project).

Then choose `pose_convention` from how the scene was authored:

| the scene was | `pose_convention` |
| -- | -- |
| exported from ROS or Gazebo as glTF, including with `export_world_gltf.sh` | `gltf_authored` |
| authored in Unity | `unity` |

It has no default. A wrong choice is invisible in the session, and Step 6 is where you check it.
[Coordinates](coordinates.md#choosing-a-pose-convention) explains the choice.

## Step 3: Write the params file

Copy the minimal params file the package installs:

```bash
mkdir -p ~/cognitive3d
cp "$(ros2 pkg prefix cognitive3d_ros)/share/cognitive3d_ros/config/params.minimal.yaml" ~/cognitive3d/params.yaml
```

Without its comments, the copy reads:

```yaml
/**:
  ros__parameters:
    scene_id: "REPLACE-WITH-YOUR-SCENE-UUID"
    scene_version: "1"
    device_id: "my-robot"
    pose_convention: "gltf_authored"
    topics:
      - "/odom"
      - "/joint_states"
      - "/scan"
```

Edit `~/cognitive3d/params.yaml` and replace every value in it with your own:

- Keep the `/**` key. A file keyed by a node name configures no node of any other name.
- Set `scene_id` to your scene's UUID. Configure refuses anything else, the file's placeholder
  included.
- Set `scene_version` to your scene's version number, quoted: `"3"`, not `3`.
- Set `device_id` to a name unique to this robot. It is how the robot appears on the dashboard.
- Set `pose_convention` to the value you chose in Step 2.
- Replace `topics` with your robot's own topics, from `ros2 topic list -t`. The three in the file
  are common names, not a promise that your robot publishes them. The node records nothing from
  your robot that is not listed, apart from batteries, diagnostics and the Nav2 and iRobot Create 3
  actions, which it finds by itself. Leave out `/tf` and `/tf_static`.
  [Recording](recording.md#the-allowlist) says what each message type records.
- Do not add an empty list such as `cameras: []`: it stops the node at startup.

For example, for a robot that runs an EKF, records into version 3 of a scene exported from Gazebo,
and has no laser scanner, the edited file reads:

```yaml
/**:
  ros__parameters:
    scene_id: "<your-scene-uuid>"
    scene_version: "3"
    device_id: "warehouse-amr-07"
    pose_convention: "gltf_authored"
    topics:
      - "/odometry/filtered"
      - "/joint_states"
      - "/imu/data"
```

The node reads the robot's pose from `map` to `base_link`, or from `odom` while `map` does not
exist. If your robot's body frame has another name, set `robot_frame`; on a robot with both
`base_link` and `base_footprint`, keep `base_link` (see
[Choosing the body frame](coordinates.md#choosing-the-body-frame)). `doctor` checks the frames in
the next step. [Configuration](configuration.md) lists every parameter.

A robot in a simulator also needs `use_sim_time: true`, and `sim_time_on_wire: true` while the
simulator runs slower than real time; see [Simulation time](sessions.md#simulation-time). A node in
a container or under systemd needs a `spool_dir` that outlives it; see
[Running in a container or under systemd](sessions.md#running-in-a-container-or-under-systemd).

## Step 4: Run doctor

`doctor` checks your configuration, credentials, network path and live ROS graph without opening a
session or uploading anything:

```bash
ros2 run cognitive3d_ros doctor --ros-args --params-file "$HOME/cognitive3d/params.yaml"
```

Fix every `FAIL` and run it again until it exits `0`. Read every `note` as well: most describe
something that would produce a session that uploads and is wrong. A check marked `(NOT CHECKED)`
did not run, whatever its severity. With the keys from Step 1 exported, `doctor` also checks that
the application key and the scene belong to your project.
[Doctor and verification](doctor-and-verification.md#read-the-output) explains the output, and
[Troubleshooting](troubleshooting.md) is indexed by what goes wrong.

## Step 5: Launch the node and record a session

Start the node with your params file:

```bash
ros2 launch cognitive3d_ros cognitive3d.launch.py params_file:="$HOME/cognitive3d/params.yaml" autostart:=true
```

The node is a lifecycle node, and it records nothing until it is configured and activated.
`autostart:=true` does both as soon as it starts. Activation opens a session, and the node logs its
id, which is the session's start in Unix seconds and the device id:

```text
[INFO] [...] [cognitive3d]: activated. Sampling poses at 30.0 Hz into session <seconds>_<your-device-id>.
```

In the first seconds after activation the node can warn `<n> allowlisted topic(s) not present on
the graph` and `no pose yet`, while it discovers the graph and its tf listener fills. Both clear
within a few seconds. The node then logs a report every 10 seconds, and its session line says
whether recording works:

```text
  session <seconds>_<your-device-id>  poses=636 frame=map  recorded=4380 dropped=0  uploaded=3 spooled=1
```

`frame=` names the reference frame the poses are in: `map`, or `odom` on a robot without a map
frame. `poses=` rises from one report to the next. A warning that repeats after the first 30
seconds is a real problem, and so is `poses=` standing still; see
[Troubleshooting](troubleshooting.md). [The report line](sessions.md#the-report-line) explains every
field.

Without `autostart:=true`, configure and activate the node yourself, or from a lifecycle manager:

```bash
ros2 lifecycle set /cognitive3d configure
ros2 lifecycle set /cognitive3d activate
```

To name the session, so your team can find it on the dashboard, launch without `autostart:=true`
and set `session_name` between the two transitions. Name the task and the run:

```bash
ros2 lifecycle set /cognitive3d configure
ros2 param set /cognitive3d session_name "patrol/run-001"
ros2 lifecycle set /cognitive3d activate
```

The name applies to every session the node opens until you set another; see
[Automatic sessions](sessions.md#automatic-sessions).

Drive the robot a few meters along a wall or corridor you can recognize in the scene, with at least
one turn. Then end the session from a second shell:

```bash
ros2 lifecycle set /cognitive3d deactivate
```

Deactivating ends the session and stops recording, and the node keeps uploading what it recorded.
Give it a minute, or until it logs `spool drained: … none remaining`, before you stop the launch
with Ctrl+C; anything still waiting uploads the next time the node starts. [Sessions](sessions.md)
covers the lifecycle, starting sessions from another node, and running the node under systemd or in
a container.

## Step 6: Find the session and check it

A session takes 1 to 3 minutes to appear on the Cognitive3D dashboard after its data is uploaded.
Open your project, find the session by its id or by your device, and open it in Session Replay.
Check that:

- The path runs along the wall or corridor you drove. A path turned 90 degrees about the scene's
  origin means the wrong `pose_convention`. A path of the right shape in the wrong place usually
  means it was recorded in `odom`; see [When the robot has no map frame](coordinates.md#when-the-robot-has-no-map-frame).
- The robot faces the direction it drove.
- The sensor series you allowlisted are there under their own names, not only the `c3d.*` series
  the node records about itself.

[Check Session Replay](doctor-and-verification.md#check-session-replay) has the full checklist.

Then check what the platform stored, with the same `C3D_PROJECT_ID` and `C3D_ORG_API_KEY` as in
Step 1, and the session's own pose convention:

```bash
ros2 run cognitive3d_ros verify_gaze_contract.py <session-id> --pose-convention gltf_authored
```

It prints `PASS` and exits `0` when every stored gaze sample has a usable position and rotation and
no gaze point, and reports whether the rotations stay level and face the way the robot drove. It
compares the session with itself, so it cannot tell whether the path lies in the right place:
Session Replay is the only check for that. The verifier is a diagnostic on an unversioned API,
described in [Verify a stored session](doctor-and-verification.md#verify-a-stored-session). No tool
reads a stored session's sensor series, events or objectives back yet: check those on the
Cognitive3D dashboard or in Session Replay.

If the session does not appear, see [The session never appears](troubleshooting.md#the-session-never-appears).

## Next steps

- Record more of the robot: sensor series, events and action objectives in [Recording](recording.md),
  and camera frames and lidar scans in [Camera and lidar](camera-and-lidar.md).
- Let a mission or task orchestrator open and close sessions, with `auto_start_session: false` and
  the session services in [Sessions](sessions.md#starting-a-session-from-another-node).
- Run A/B tests on the robot's behavior with [Remote variables](remote-variables.md).
- Run the node unattended: [Running in a container or under systemd](sessions.md#running-in-a-container-or-under-systemd).
