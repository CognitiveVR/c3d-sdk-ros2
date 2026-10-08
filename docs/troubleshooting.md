# Troubleshooting

Most integration mistakes produce no error. The platform accepts the upload with HTTP 200, and the
session is empty, misplaced, filed under another project or never shown. This page is indexed by
what you see. Each entry lists the likely causes, how to check for each, and the page that explains
it and the fix.

## Start with doctor

Run `doctor` on the robot, with the node stopped and the same params file, environment, ROS domain
and namespace the node uses:

```bash
ros2 run cognitive3d_ros doctor --ros-args --params-file <path/to/params.yaml>
```

It opens no session and uploads nothing, and it catches most of what follows. Fix every `FAIL`, and
read every `note`. A check marked `(NOT CHECKED)` did not run: set `C3D_PROJECT_ID` and
`C3D_ORG_API_KEY` to run the checks that ask the platform about your project.
[Doctor and verification](doctor-and-verification.md#checks) describes every check.

## The node stops at startup

| likely cause | check | fix |
| -- | -- | -- |
| A value of the wrong type, such as `scene_version: 1` unquoted | `cognitive3d_node:` on stderr, or doctor's `parameters` check, names the parameter | Quote strings and give each value the type in its row: [How parameters are loaded](configuration.md#how-parameters-are-loaded) |
| An empty list, such as `cameras: []` or `action_names: []` | The same message names the parameter | Leave an unused list out. To record no actions by name, set `action_discovery: false`: [Actions](recording.md#actions) |
| The launch file has no `params_file` | `params_file:=<path> is required`, or `params_file '…' is not a file` | Pass the path to your copy of `params.minimal.yaml`: [Configuration](configuration.md) |

## Configure is refused

The node logs `configuration rejected:` and the reason, lists every missing required value at
once, and stays unconfigured. doctor's `configuration` check prints the same reason.

| likely cause | check | fix |
| -- | -- | -- |
| `C3D_APPLICATION_API_KEY` is not in the node's environment | The refusal names the variable. A shell variable that is not exported, or a key set for your login shell but not for a systemd unit or container, never reaches the node | [Environment variables](configuration.md#environment-variables); for services, [Running in a container or under systemd](sessions.md#running-in-a-container-or-under-systemd) |
| `scene_id`, `scene_version` or `pose_convention` is not set, `scene_id` is not a UUID (the placeholder still in the file), or `scene_version` is not a number of 1 or more | The refusal names each one | [Required parameters](configuration.md#required-parameters) |
| The params file sets `application_key`, to any value, `""` included | The refusal names `application_key` and `C3D_APPLICATION_API_KEY` | Remove the line and export the key instead: [Environment variables](configuration.md#environment-variables) |
| `C3D_ENVIRONMENT` is set to anything but `prod`, or only one of `C3D_DATA_HOST` and `C3D_API_HOST` is set | The refusal names the variable | Unset both for production: [Environment variables](configuration.md#environment-variables) |
| Another process holds `spool_dir`, such as a second node or an earlier instance still running | `spool_dir … is already in use by process <pid>`; doctor's `spool directory` check | Give every node its own `spool_dir`: [One process per spool directory](sessions.md#one-process-per-spool-directory) |
| Under systemd, neither `XDG_STATE_HOME` nor `HOME` names a directory, so the spool has no default | The refusal names `spool_dir` | Set `spool_dir` explicitly: [Running in a container or under systemd](sessions.md#running-in-a-container-or-under-systemd) |
| A `session_properties` key in `c3d.`, the SDK's own namespace | The refusal names the key | Rename it: [What every session records](sessions.md#what-every-session-records) |
| A QoS value the node does not apply, such as `best-effort` for `best_effort`, or a `qos.depth` of 0 | The refusal names the parameter and the values it takes | [The allowlist and per-topic rules](configuration.md#the-allowlist-and-per-topic-rules) |
| A `camera.<name>.*`, `lidar.<name>.*` or `sensor.<name>.*` key under a name `cameras`, `lidars` or `sensors` does not list, or with a misspelled field, such as `hfov_deg` for `hfov_degrees` | The refusal names the key, and says either that the list does not name it or that it is not a parameter, followed by the fields there are | Add the name to its list or remove the block, or correct the field: [Cameras and lidars](configuration.md#cameras-and-lidars), [Described sensors](configuration.md#described-sensors) |
| A `topic_config` key for a topic not in `topics`, such as a battery topic taken by its type; a key spelled otherwise than its entry, such as `navigate_to_pose_action_status` for `/navigate_to_pose/_action/status`; or a misspelled field, such as `rate` for `rate_hz` | The refusal names the key, and says either that it matches no entry in `topics`, followed by the keys `topics` gives, or that it is not a parameter, followed by the fields there are | Add the topic to `topics`, or correct the key or the field: [Per-topic rule keys](configuration.md#per-topic-rule-keys) |
| `sim_time_on_wire: true` without `use_sim_time: true`, `protocol: http` on a robot, or a value out of range, such as a jump threshold of 0 or a NaN | The refusal names the parameter | The parameter's row in [Configuration](configuration.md) |

## The node ignores the params file

Every parameter takes its default while the run looks configured.

| likely cause | check | fix |
| -- | -- | -- |
| The file is keyed by a node name, and the node runs under another | The file's top-level key is not `/**`; doctor's `configuration` line names the scene and device in force | Key the file `/**`: [How parameters are loaded](configuration.md#how-parameters-are-loaded) |
| A value was changed with `ros2 param set` while the node was configured | The set was rejected: `<name> is read at configure, so a set now would apply only at the next one` | Clean up, set the value and configure again; only `session_name`, `session_properties` and `uploads_paused` can be set on a configured node: [How parameters are loaded](configuration.md#how-parameters-are-loaded) |

## No session starts

| likely cause | check | fix |
| -- | -- | -- |
| The node was never activated | `ros2 lifecycle get /cognitive3d` says `unconfigured` or `inactive` | Configure and activate it, or launch with `autostart:=true`: [Lifecycle](sessions.md#lifecycle) |
| `auto_start_session: false`, and nothing calls `~/start_session` | The parameter in your params file | [Automatic sessions](sessions.md#automatic-sessions) |
| `~/start_session` is refused | The response's `ok`, `session_id` and `message` | [The refusal contract](sessions.md#the-refusal-contract) |
| The system clock is unset, as on a board with no real-time clock before NTP | `no session was started: the wall clock reads …`; activation fails; doctor's `wall clock` check | Wait for time synchronization, and under systemd order the unit after it: [An unset wall clock](sessions.md#an-unset-wall-clock) |
| `dry_run: true` is left in the params file | The report line says `[dry run: nothing is uploaded]` | Remove it: [Diagnostics and limits](configuration.md#diagnostics-and-limits) |

## The session never appears

A session takes 1 to 3 minutes to appear on the Cognitive3D dashboard after its data is uploaded,
and a session the robot is still uploading appears later than that.

| likely cause | check | fix |
| -- | -- | -- |
| It is still being uploaded | The report line's `spooled=` count is above 0 | Wait, and poll rather than check once: [Wait for the session to appear](doctor-and-verification.md#wait-for-the-session-to-appear) |
| Uploads are paused or failing | The report line and the `cognitive3d: uploads` status on `~/diagnostics` | [Uploads stall or pause](#uploads-stall-or-pause) |
| The node stopped before the upload finished | The shutdown log line says how many chunks were left | They upload the next time the node is configured: [Shutdown and crashes](sessions.md#shutdown-and-crashes) |
| The application key belongs to another project, so the session is in that project | doctor's `key belongs to project` check, with `C3D_PROJECT_ID` set | Read the key from the platform: [Fetch your application key](doctor-and-verification.md#fetch-your-application-key) |
| The node uploaded to another stack | doctor's `environment` check names both hosts | Unset `C3D_DATA_HOST`, `C3D_API_HOST` and `gateway` for production: [Environment variables](configuration.md#environment-variables) |
| The board restored a stale saved clock at boot, so the session is dated in the past | The session's date on the dashboard; `timedatectl` said the clock was not synchronized when the session began | Start the node only once the clock is synchronized: [An unset wall clock](sessions.md#an-unset-wall-clock) |

## The session is in an empty world, or the scene looks wrong

| likely cause | check | fix |
| -- | -- | -- |
| The scene id belongs to another project, so the platform made an empty scene for it in yours | doctor's `scene/project` check, with `C3D_ORG_API_KEY` set; or the scene list of your project on the dashboard | [Confirm the scene belongs to your project](scenes.md#confirm-the-scene-belongs-to-your-project) |
| `scene_version` holds the version id, not the version number | doctor's `scene/project` check says so; the Upload Web App reports both | Use `versionNumber`: [Scene id and scene version](scenes.md#scene-id-and-scene-version) |
| The glTF still references a buffer that is not named `scene.bin` | `gltf_bbox.py scene.gltf` reports `WILL UPLOAD EMPTY` | [Prepare the files](scenes.md#prepare-the-files) |
| A primitive has no material, so it renders pink, or a texture is referenced by a path, so it renders untextured | The glTF's `materials` and `images` | [Prepare the files](scenes.md#prepare-the-files) |
| A Gazebo export lies on its side | The export's orientation verdict | [Export a Gazebo world](scenes.md#export-a-gazebo-world) |

## The robot is in the wrong place

| likely cause | check | fix |
| -- | -- | -- |
| The wrong `pose_convention`: the path is turned 90 degrees about the scene's origin | Drive along a wall you can recognize and look in Session Replay. No tool can check this | [Choosing a pose convention](coordinates.md#choosing-a-pose-convention) |
| The pose was recorded in `odom`, whose origin is wherever the robot booted: the path has the right shape in the wrong place | The session's `robot.reference_frame_unavailable` event; doctor's `robot pose` note | [When the robot has no map frame](coordinates.md#when-the-robot-has-no-map-frame) |
| The `map` frame's origin and axes differ from the scene's | Where the robot's start lands in the scene | [Reference frames](coordinates.md#reference-frames) |
| The localizer corrected itself with a jump | `robot.localisation_jump` events and `c3d.localisation_jumps` | [Localization jumps](coordinates.md#localization-jumps) |

## The robot model is sideways, rolling or missing

These apply only with `robot_dynamic_object: true`. By default Session Replay draws the robot from
the pose timeline and needs no mesh.

| likely cause | check | fix |
| -- | -- | -- |
| The mesh's forward axis is not the robot frame's +X: the model drives sideways or backwards along a correct path | Session Replay; `verify_dynamics_level.py` | Set `mesh_yaw_offset_deg`: [Aligning the robot model](coordinates.md#aligning-the-robot-model) |
| The model renders turns as rolls | The same session's path is also turned 90 degrees | The wrong `pose_convention`: [Choosing a pose convention](coordinates.md#choosing-a-pose-convention) |
| The mesh needs a roll or pitch, which `mesh_yaw_offset_deg` cannot correct | The URDF's visual origin | Bake the rotation into the mesh: [Prepare the robot mesh](scenes.md#prepare-the-robot-mesh) |
| The object was created and its mesh files were never stored | doctor's `mesh/files` check | Make the second request: [Upload the robot mesh](scenes.md#upload-the-robot-mesh) |
| A corrected mesh was uploaded under the same mesh name, which never replaces the files | The model is unchanged | Upload it under a new name: [Mesh files are permanent per mesh name](scenes.md#mesh-files-are-permanent-per-mesh-name) |
| The scene has a new version, and objects belong to a scene version | doctor's `mesh/exists` and `mesh/files` checks | Upload the mesh to the new version: [Upload the robot mesh](scenes.md#upload-the-robot-mesh) |

## Height and tilt are always zero

The standard producers of `map` and `odom` on a mobile robot are planar, so a ramp is recorded
flat and every verifier still passes. A constant zero height is evidence of nothing. doctor's
`pose channel` check reports whether the frame in force carried any height or tilt.
[Planar poses](coordinates.md#planar-poses) explains how to record them.

## The gaze points at the floor, or 90 degrees off

| likely cause | check | fix |
| -- | -- | -- |
| The gaze viewpoint is the robot body's, which is usually at floor level | `gaze_frame` is not set | Set `gaze_frame` to a forward-facing camera frame: [The gaze channel](coordinates.md#the-gaze-channel) |
| `gaze_frame` does not resolve, so every sample falls back to the body | `c3d.gaze_frame_failures`; doctor's `gaze frame` check fails | [Optical and body frames](coordinates.md#optical-and-body-frames) |
| The wrong `gaze_frame_convention`: the gaze rotation is 90 degrees off | The configure log line naming the frame and the convention in force | [Optical and body frames](coordinates.md#optical-and-body-frames) |
| The verifier was given the wrong `--pose-convention`. A `rep103` session checked as `unity` or `gltf_authored` reads 90 degrees off on every segment; a `unity` or `gltf_authored` session checked as `rep103` reads as a correct heading, as reversing, or with no segments, by the direction driven. Either way the level angle follows the heading | The session's `c3d.ros.pose_convention`, which is the `--pose-convention` to pass | [Check the gaze contract](doctor-and-verification.md#check-the-gaze-contract) |

## Sensors are missing

A session with no sensors from your robot still shows the node's own `c3d.*` series, so it looks as
though it has sensors.

| likely cause | check | fix |
| -- | -- | -- |
| The topic is not in `topics`, or is listed under another name, such as `/odom` on a robot that publishes `/odometry/filtered` | doctor's `topics` check names allowlisted topics missing from the graph; compare with `ros2 topic list`, and on a large graph list again 20 seconds later: the first list can be incomplete | [The allowlist](recording.md#the-allowlist) |
| The topic's type has no translator | The report line for the topic reads `no-xlat` | [What each message type records](recording.md#what-each-message-type-records) |
| The topic is an image, point cloud or other bulk type, which the allowlist refuses | The report line reads `REFUSED` | Record it as a camera or lidar: [Camera and lidar](camera-and-lidar.md) |
| The node runs in a namespace, and the battery or diagnostics topic is outside it | The topic's full name | Name it in `topics`: [Namespaces](recording.md#namespaces) |
| Two topics of one odometry, IMU, joint-state or laser-scan type share one series | doctor's `shared series` check; a warning at subscribe | Allowlist one topic per type: [One topic per type](recording.md#one-topic-per-type-for-odometry-imu-joint-states-and-scans) |
| A `series` name you set never appears, because the topic's type names its own series | A warning at subscribe that the `series` is ignored | Only `Float32` and `Int32MultiArray` topics read it: [Series names](recording.md#series-names) |
| A steady value is stored only every 2 seconds | The series has fewer points than the rate | Expected: [Rates](recording.md#rates) |
| A publisher's QoS does not match the subscription, often because two publishers of one topic disagree, as on `/joint_states` under ros2_control | doctor's `topics/qos` check; the node logs a QoS incompatibility naming the policy and the override; the report line marks the topic `[QOS INCOMPATIBLE]` | Set the override it names: [Ingest and QoS](recording.md#ingest-and-qos) |

## Events or objectives are missing

| likely cause | check | fix |
| -- | -- | -- |
| The objective is keyed on `<action>.started`, which Nav2 and Create 3 goals never record | The session's events include `<action>.executing` | Key the start of a goal on `.executing`: [Objectives](recording.md#objectives) |
| The action is not enrolled: a custom action, an `action_names` list that dropped a default, or a server outside the node's namespace | doctor's `actions` check, and the `action servers on the graph` log line | [Actions](recording.md#actions) |
| The goal finished before the session opened, or was already running when it opened | When the goal was sent | Start the session before sending the goal: [Objectives](recording.md#objectives) |
| Events from `~/events` are published best-effort, which does not connect | `ros2 topic info --verbose` on the events topic | Publish reliably: [Publishing your own events](recording.md#publishing-your-own-events) |
| Events were published with no session open, or while the node was inactive | `DROPPED=` on the report line counts the first; the second is not counted | [Publishing your own events](recording.md#publishing-your-own-events) |
| One source exceeded the event rate cap | `c3d.events_suppressed` | [The event rate cap](recording.md#the-event-rate-cap) |
| An event arrives without some of its properties, because their keys were in `c3d.` | `c3d.event_properties_dropped`; a warning naming each key | Rename them: [Publishing your own events](recording.md#publishing-your-own-events) |
| Nav2 behavior tree events need `cognitive3d_ros_nav2`, `/behavior_tree_log` in `topics`, and in a component container `cognitive3d_ros_nav2::Cognitive3DNode` rather than the base component | Started by the launch file: doctor's `build` line names `nav2_msgs/msg/BehaviorTreeLog`. In a component container: the loaded node's own configure log line `translating N message type(s)` names it. doctor reports the launch file's build, not the component a container loads | [Nav2](recording.md#nav2) |

## No camera frames or lidar scans

| likely cause | check | fix |
| -- | -- | -- |
| The topic's type is not one the sensor takes | doctor's `camera/<name>` or `lidar/<name>` check | [What gets refused](camera-and-lidar.md#what-gets-refused) |
| The sensor's frame does not resolve, so every record is dropped | `c3d.camera.<name>.pose_failures` or `c3d.lidar.<name>.pose_failures`; doctor's sensor check | [Poses and frames](camera-and-lidar.md#poses-and-frames) |
| A depth image, or an encoding the node cannot read | `c3d.camera.<name>.refused` | Record the color topic: [Record a camera](camera-and-lidar.md#record-a-camera) |
| The publisher's QoS does not match | The node logs a QoS incompatibility naming the policy | [QoS](camera-and-lidar.md#qos) |
| Each record is larger than the platform stores, and is dropped after the upload succeeds | doctor's sensor check notes a record over 900 KiB | [Bandwidth](camera-and-lidar.md#bandwidth) |
| The spool evicted them at its cap, which evicts camera and lidar data first | `c3d.chunks_evicted`; a `cognitive3d: uploads` warning | [The size cap](sessions.md#the-size-cap) |

## A point cloud is on its side, or a scan is drawn rotated

| likely cause | check | fix |
| -- | -- | -- |
| The cloud's header names an optical frame its points are not in, as Gazebo's depth camera does | doctor's `lidar/<name> optical frame` note | Set `lidar.<name>.frame`: [A point cloud whose header names the wrong frame](camera-and-lidar.md#a-point-cloud-whose-header-names-the-wrong-frame) |
| A tool reading the stored data posed a scan by the camera rule | The scan is right under `rep103` and wrong under the other conventions | [A laser scan's pose is the scan plane's](camera-and-lidar.md#a-laser-scans-pose-is-the-scan-planes) |

## A/B test arms are wrong

| likely cause | check | fix |
| -- | -- | -- |
| The variable's own probability is not 0, so its base value wins a share of every draw | The variable's `remoteVariableProbability` on the dashboard | [Set up a test](remote-variables.md#set-up-a-test) |
| Every run of one robot gets the same arm, because `participant_id` is empty and the draw uses `device_id` | `c3d.participant.id` repeats across sessions | Pass a new UUID per run: [Choosing the participant id](remote-variables.md#choosing-the-participant-id) |
| Two sticky tests in one project always agree | Their arms across sessions | [Two platform behaviors to plan around](remote-variables.md#two-platform-behaviors-to-plan-around) |
| The test is disabled, which looks exactly like no test | `remote_variables_json` lacks the variables you expect | [Two platform behaviors to plan around](remote-variables.md#two-platform-behaviors-to-plan-around) |
| The fetch failed | A `c3d.remote_variables.unavailable` event with its `status` | [What the node does at session start](remote-variables.md#what-the-node-does-at-session-start) |
| The robot never applies the arm, because activation opened the session and nothing read the response | `auto_start_session: true` | [Applying the arm](remote-variables.md#applying-the-arm) |

## Uploads stall or pause

| likely cause | check | fix |
| -- | -- | -- |
| The platform refuses the application key, so every upload is paused with everything kept | `[UPLOADS PAUSED: credentials refused]` on the report line; `cognitive3d: uploads` is ERROR; `c3d.upload_auth_paused` is 1 | Restart the node with the right key in its environment; nothing spooled is lost: [Outages and refused keys](sessions.md#outages-and-refused-keys) |
| No network, a captive portal or a proxy in the way | doctor's `gateway` check | [Outages and refused keys](sessions.md#outages-and-refused-keys) |
| `uploads_paused: true` | `ros2 param get /cognitive3d uploads_paused` | `ros2 param set /cognitive3d uploads_paused false`: [Pausing uploads](sessions.md#pausing-uploads) |
| The chunks were recorded for another host, or for another project: another application key and another `scene_id`, as when a robot moved projects | `c3d.chunks_held`; a log line that the backlog was recorded under another application key, for another scene | Configure a node with the old key and scene to drain them, or delete them: [Outages and refused keys](sessions.md#outages-and-refused-keys) |
| One stream's chunks keep failing on the gateway's side while the others upload | `[STREAMS DEFERRED: <streams>]` on the report line; `c3d.upload_deferred_streams` | The stream resumes by itself once the gateway accepts it again: [Outages and refused keys](sessions.md#outages-and-refused-keys) |
| The uplink is slower than the data | `spooled=` keeps rising | Lower the camera and lidar rates, or pause uploads and drain later: [What it costs](camera-and-lidar.md#what-it-costs) |

## Part of a session is missing

| likely cause | check | fix |
| -- | -- | -- |
| No transform from the reference frame to `robot_frame`, so the session has no path | `c3d.pose_failures`; doctor's `robot pose` check | [The pose timeline](coordinates.md#the-pose-timeline) |
| A component loaded in a namespace without the robot's tf remaps reads the global `/tf`, which holds another robot's tree or none | `ros2 topic info -v /<namespace>/tf` does not list the node | Load it with `-r /tf:=tf -r /tf_static:=tf_static`: [Running in a container or under systemd](sessions.md#running-in-a-container-or-under-systemd) |
| The spool reached `spool_max_mb` and evicted data, which never uploads | `c3d.chunks_evicted`; the eviction log line | Size the spool for the longest run without a network: [The size cap](sessions.md#the-size-cap) |
| The process was killed, losing the last seconds in memory, and the session was closed as `abnormalEnd` | The end event's `Reason`; `c3d.recovered` | [Shutdown and crashes](sessions.md#shutdown-and-crashes) |
| The robot's transform stopped advancing, so poses were skipped | `c3d.stale_poses` | [Stale transforms](coordinates.md#stale-transforms) |
| The node waited for `map` at the start of the session | Nothing is recorded for up to `reference_frame_grace_s` | [Reference frames](coordinates.md#reference-frames) |
| The remote-variables fetch held the start of the session | A gap of up to `remote_variables_timeout_s` at the start | [What the node does at session start](remote-variables.md#what-the-node-does-at-session-start) |

## A simulated session is too long or splits in two

| likely cause | check | fix |
| -- | -- | -- |
| Under `use_sim_time`, the session runs on the wall clock, so a simulation whose real-time factor is below 1 stretches it | The session's length against the simulated time | Set `sim_time_on_wire: true`: [Simulation time](sessions.md#simulation-time) |
| The simulation reset, and ROS time stepped backwards | An end reason of `ros_time_jumped_backwards` | Expected: [Clock jumps](sessions.md#clock-jumps) |

## Two robots record each other

| likely cause | check | fix |
| -- | -- | -- |
| Both robots are on one ROS graph | `ros2 node list` shows the other robot's nodes | Give each robot its own `ROS_DOMAIN_ID`, and do not use `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` for this: [Robots on a shared network](sessions.md#robots-on-a-shared-network) |
| Both robots use one `device_id` | `c3d.deviceid` on their sessions | Set a unique `device_id`: [Identity and scene](configuration.md#identity-and-scene) |

## The node misbehaves in a container or under systemd

| likely cause | check | fix |
| -- | -- | -- |
| The container has its own network or IPC namespace, so the node subscribes and receives nothing | `--network=host` and `--ipc=host` on the container | [Run in a container](install.md#run-in-a-container) |
| Nothing configures and activates the node, so it records nothing | `ros2 lifecycle get /cognitive3d` | Launch with `autostart:=true`: [Lifecycle](sessions.md#lifecycle) |
| The node was started with `docker exec`, and is killed outright when the container stops | Sessions end as `abnormalEnd` | Start it from the entrypoint: [Running in a container or under systemd](sessions.md#running-in-a-container-or-under-systemd). In a robot container you cannot recreate, stop it with SIGINT first: [A robot stack in a container you cannot recreate](sessions.md#a-robot-stack-in-a-container-you-cannot-recreate) |
| The spool is inside the container, and is lost with it | `spool_dir` is not on a volume | [Running in a container or under systemd](sessions.md#running-in-a-container-or-under-systemd), or a mount the container already has: [A robot stack in a container you cannot recreate](sessions.md#a-robot-stack-in-a-container-you-cannot-recreate) |
| Under systemd with `autostart:=true`, the unit started before the clock was synchronized, activation failed once, and nothing activated the node again | The unit's journal at boot shows `no session was started: the wall clock reads …`; `ros2 lifecycle get /cognitive3d` says `inactive` | Order the unit after time synchronization: [Running in a container or under systemd](sessions.md#running-in-a-container-or-under-systemd) |
| A driver or another node in the same `component_container` stalls when a session starts or the node cleans up | Which container loads the component, and what else it holds | Load the node into `component_container_isolated` or a container of its own: [Running in a container or under systemd](sessions.md#running-in-a-container-or-under-systemd) |
| Lifecycle transitions and services are slow in `component_container_mt` while the robot's topics keep the node busy | Which container loads the component | Use `component_container_isolated`, or a container of the node's own: [Running in a container or under systemd](sessions.md#running-in-a-container-or-under-systemd) |

## Getting help

Ask on the Cognitive3D [Discord](https://discord.gg/x38sNUdDRH), or email
`support@cognitive3d.com`. Include the full `doctor` output, which names the SDK version, the ROS 2
distro and the RMW and masks the application key; how you installed the SDK; and whether the node
runs in a container. Never include a key: refer to it by its environment variable, or by its last
four characters as `doctor` prints them.
