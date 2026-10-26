# Configuration

This page lists every parameter the node (`cognitive3d_node`) reads, and the environment variables
beside them. Most mistakes here produce no error at run time: the platform accepts the upload with
HTTP 200, and the session is empty, misplaced or never shown. Each row says what goes wrong
silently, and how to avoid it. Run [`doctor`](doctor-and-verification.md) after every change to
your params file.

Start from the minimal file the package installs, not from a copy of the reference file:

```bash
cp "$(ros2 pkg prefix cognitive3d_ros)/share/cognitive3d_ros/config/params.minimal.yaml" <path/to/params.yaml>
```

`cognitive3d.yaml`, in the same directory, lists every parameter at its default with a short
comment. It is a reference, not a starting point. [Install](install.md) covers installing the
package and its tools.

## How parameters are loaded

- **Key the file `/**`.** A params file keyed by a node name applies only to a node of that name.
  Under any other name the node takes every default, and the run looks configured.

  ```yaml
  /**:
    ros__parameters:
      scene_id: "<your-scene-uuid>"
      scene_version: "<your-scene-version>"
      pose_convention: "gltf_authored"
  ```

- **Pass the file when you start the node.** The launch file requires it:
  `ros2 launch cognitive3d_ros cognitive3d.launch.py params_file:=<path/to/params.yaml>`. With
  `ros2 run`, pass `--ros-args --params-file <path/to/params.yaml>` to `cognitive3d_node`, and the
  same to `doctor`.
- **Parameters are read at configure.** A change takes effect at the next configure:
  `ros2 lifecycle set /cognitive3d cleanup`, set the value, then `configure`. While the node is
  configured, `ros2 param set` is rejected for every parameter but three, with a message saying
  so: [`uploads_paused`](#uploads) takes effect at once, and `session_name` and
  `session_properties` from the next session the node starts.
- **Give each value the type in its row.** Quote a string that looks like a number:
  `scene_version: "1"`, not `scene_version: 1`. A double parameter also accepts a whole number
  (`rate_hz: 2`). An integer parameter needs a whole number with no decimal point. A value of the
  wrong type stops the node at construction, and `doctor`'s `parameters` check names the
  parameter and the fix.
- **Leave a list you do not use out of the file.** ROS 2 cannot type an empty YAML list, so a
  line such as `cameras: []` stops the node at construction. Every list defaults to empty, except
  [`action_names`](#discovery).
- **Configure refuses a value it cannot use**, naming the parameter, and leaves the node
  unconfigured. A number that must be positive, or 0 or more, refuses NaN as well. Missing
  required values are reported together, in one refusal.
- **Topic names resolve like ROS 2 names.** A relative name in `topics` or in a sensor's `topic`
  resolves against the node's namespace, and `~/name` resolves under the node itself.

## Environment variables

The node reads its credentials and hosts from its own process environment, never from a ROS
parameter. A ROS parameter is readable by anything on the ROS graph and is echoed into launch logs
and parameter dumps. A params file that sets `application_key`, to any value including `""`,
refuses configure, and `ros2 param set` of it is rejected. The node does not declare the name, so
`ros2 param get` cannot read a key left there.

| variable | read by | what it does |
| -- | -- | -- |
| `C3D_APPLICATION_API_KEY` | node, `doctor` | Required. The application key the node uploads with. Configure refuses without it. Export it in the environment the node runs in; a shell variable that is not exported never reaches a child process. |
| `C3D_ENVIRONMENT` | node, `doctor`, tools | Leave it unset. The only accepted value is `prod`, matched exactly; any other value, including an empty one, refuses configure. |
| `C3D_DATA_HOST`, `C3D_API_HOST` | node, `doctor`, tools | For a Cognitive3D stack other than production, such as one Cognitive3D runs for you. Set both or neither, as bare host names with an optional port: `data.<your-stack>` and `api.<your-stack>`, with no scheme or path. One without the other, an empty one, or either beside `C3D_ENVIRONMENT` refuses configure. Unset, the hosts are production. |
| `C3D_PROJECT_ID` | `doctor`, tools | Your project's id. Lets `doctor` check that the application key belongs to it. The verifiers and `fetch_application_key.py` require it. |
| `C3D_ORG_API_KEY` | `doctor`, tools | An organization key. Lets `doctor` ask the platform about your project; the verifiers and `fetch_application_key.py` require it. The node never reads it. |
| `XDG_STATE_HOME`, `HOME` | node | Locate the default [`spool_dir`](#the-spool). |

The node discovers topics on its ROS domain. To keep two robots on one network apart, give each its
own `ROS_DOMAIN_ID`.

## Required parameters

Configure refuses a node without these three values, and without `C3D_APPLICATION_API_KEY`.

| parameter | type | default | unit | what it does |
| -- | -- | -- | -- | -- |
| `scene_id` | string | `""` | — | The id of the Cognitive3D scene the session is recorded against: `<your-scene-uuid>`, 8-4-4-4-12 hexadecimal digits in either case. Any other value refuses configure, the `params.minimal.yaml` placeholder included. Use a scene from the same project as the application key. The platform does not refuse a scene id from another project: it creates an empty scene for it in the key's project. See [Scenes](scenes.md). |
| `scene_version` | string | `""` | — | The scene's version number, as a quoted string of digits, 1 or greater: `"1"`. An unquoted `1` is an integer and stops the node at construction. Text such as `"latest"`, and `"0"`, refuse configure. |
| `pose_convention` | string | `""` | — | How poses are converted for your scene: `gltf_authored` for a scene exported from ROS or Gazebo as glTF, `unity` for a scene authored in Unity, `rep103` only for debugging. It has no default. A wrong choice is invisible in the session: the path lands off the floor plan and nothing reports it. See [Coordinates](coordinates.md). |

## Identity and scene

`device_id` names the robot. The other parameters here apply only when the robot is also recorded
as a Dynamic Object, which is off by default: Session Replay draws the robot from the pose timeline,
and no mesh has to be uploaded. [Scenes](scenes.md) covers uploading a robot mesh.

| parameter | type | default | unit | what it does |
| -- | -- | -- | -- | -- |
| `device_id` | string | `""` | — | The robot's stable identity: the device on the Cognitive3D dashboard, and the identity remote variables are drawn for when a session has no `participant_id`. Empty resolves at configure to `ros-` and the node's namespace segments joined by `-` (`/robot1` is `ros-robot1`, `/fleet/robot1` is `ros-fleet-robot1`), or to `ros-unnamed` in the root namespace. At most 64 bytes of valid UTF-8. Set it to `<your-device-id>`: robots left at `ros-unnamed` share one identity. |
| `robot_dynamic_object` | bool | `false` | — | Also record the robot as a Dynamic Object. When `true`, the mesh named by `robot_mesh` must be uploaded to the scene version, or Session Replay shows no robot while the session records normally. |
| `robot_mesh` | string | `"robot_placeholder"` | — | The mesh name of the uploaded Dynamic Object. Read only when `robot_dynamic_object` is `true`. |
| `robot_display_name` | string | `"Robot"` | — | The Dynamic Object's display name. Read only when `robot_dynamic_object` is `true`. |
| `robot_object_id` | string | `"robot"` | — | The id the Dynamic Object is registered and sampled under. It must match the id of the object you uploaded. Empty refuses configure, even when `robot_dynamic_object` is `false`. |
| `mesh_yaw_offset_deg` | double | `0.0` | degrees | A yaw applied to the Dynamic Object only, never to the gaze rotation, for a mesh whose forward axis is not +X. Yaw only: bake any other rotation into the mesh before uploading it. |
| `dynamic_up_offset_meters` | double | `0.0` | meters | Raises the Dynamic Object along the scene's up axis, for a mesh whose origin is not where the model should sit. |

## Poses and the gaze channel

The node samples the robot's pose from tf into the gaze channel, the session's spatial timeline.
[Coordinates](coordinates.md) explains the frames, the missing `map` frame, and why height and tilt
read zero on most mobile robots.

| parameter | type | default | unit | what it does |
| -- | -- | -- | -- | -- |
| `sample_rate_hz` | double | `30.0` | Hz | Pose samples per second, from 1/86400 (one a day) to 1000. A transform whose stamp stops advancing for 10 sample periods, and at least 5 seconds, is skipped as stale and counted in `c3d.stale_poses`. |
| `reference_frame` | string | `"map"` | — | The frame poses are read in when it resolves. Empty uses `fallback_reference_frame` alone. |
| `fallback_reference_frame` | string | `"odom"` | — | The frame used when `reference_frame` does not resolve. A session that starts on it records `robot.reference_frame_unavailable`, and every switch records `robot.reference_frame_changed`. A path recorded in `odom` starts wherever the robot booted, not where it is in the scene. |
| `reference_frame_grace_s` | double | `5.0` | seconds | How long the node waits for `reference_frame` once the fallback resolves, before it settles on the fallback. Nothing is recorded while it waits. `doctor` waits for the same time. A finite number, 0 or more: `0` adopts the fallback at once. |
| `robot_frame` | string | `"base_link"` | — | The robot body frame whose pose is sampled. |
| `relocalisation_jump_m` | double | `0.5` | meters | A step in `reference_frame` larger than this, between poses about 0.1 seconds apart, records a `robot.localisation_jump` event. Must be positive. Below a `sample_rate_hz` of 10 the poses are consecutive samples, each covering more ground, so raise it in proportion. |
| `relocalisation_jump_deg` | double | `45.0` | degrees | The same, for a turn. Must be positive. |
| `gaze_frame` | string | `""` | — | A frame the gaze channel reports instead of the robot body: its position, and its rotation with its forward axis as the rotation's forward. Empty uses `robot_frame`. A name that does not resolve falls back to the robot pose with no error in the session; it is counted in `c3d.gaze_frame_failures`, and `doctor`'s `gaze frame` check fails on it. |
| `gaze_frame_convention` | string | `"auto"` | — | Which axis of `gaze_frame` is forward: `body_forward_x` for a REP-103 body frame, `optical_forward_z` for a camera optical frame, or `auto`, which picks `optical_forward_z` when a `_` or `/` separated part of the name is `optical` (any case) and the next part is not `flow`, and logs its choice. A wrong choice turns the gaze rotation 90 degrees off with no error. |

## The allowlist and per-topic rules

The node subscribes to nothing from your robot that is not in `topics`, apart from the types and
actions it takes by [discovery](#discovery) and the [cameras and lidars](#cameras-and-lidars) you
name. [Recording](recording.md) explains what each message type records.

| parameter | type | default | unit | what it does |
| -- | -- | -- | -- | -- |
| `topics` | string[] | `[]` | — | The allowlist: topic names, or a prefix ending in `*` (`/robot1/*`). A `*` anywhere else refuses configure, because the entry would match nothing. `~name` refuses configure; write `~/name`. A `topic_config` key for a topic not listed here refuses configure: see [Per-topic rule keys](#per-topic-rule-keys). |
| `topic_config.<key>.rate_hz` | double | `-1.0` | Hz | Samples per second, applied by the translator. 0 or less takes the message type's default: 30 Hz for odometry, joint states, IMU, `Float32` and `Int32MultiArray`, and 1 Hz for every other type, batteries and laser scans included. The diagnostics and action status translators do not read it. |
| `topic_config.<key>.series` | string | `""` | — | The sensor series the topic's values are recorded under. Set it to state a unit: `ultrasonic.distance_centimeters`. Empty derives the name from the topic (`/ultrasonic` records `ultrasonic.value`). Read only for `std_msgs/msg/Float32` and `std_msgs/msg/Int32MultiArray` topics; other types name their own series and ignore it, and the node warns when it subscribes one. Ignored, with a warning, on a `*` entry. |
| `topic_config.<key>.ingest` | string | `""` | — | `refuse`, `metadata_once`, `derive_metrics`, `downsample` or `full_ingest`; any other value refuses configure. Empty takes the message type's default. Only `refuse` and `full_ingest` have an effect, described below. |
| `topic_config.<key>.qos.reliability` | string | `""` | — | `reliable` or `best_effort`. Empty negotiates with the publishers. |
| `topic_config.<key>.qos.durability` | string | `""` | — | `volatile` or `transient_local`. Empty negotiates with the publishers. |
| `topic_config.<key>.qos.history` | string | `""` | — | `keep_last` or `keep_all`. Empty keeps the last `qos.depth` messages. |
| `topic_config.<key>.qos.depth` | int | `-1` | messages | The queue depth: 1 or more, or `-1` for the deepest depth the publishers offer, and at least 10. `0` refuses configure. |

With an empty `topics` and discovery at its defaults, a session still records the pose timeline,
the node's own `c3d.*` series, battery and diagnostics topics, the actions discovery enrolls, and
any cameras and lidars. It records nothing else from your robot: no odometry, IMU, scans or joint
states. Because the `c3d.*`
series appear on the dashboard as sensor data, a session with a broken allowlist still looks like it
has sensors. Check the names against `ros2 topic list` before you write them: `/odom` is a
convention, and a robot running an EKF often publishes `/odometry/filtered` and no `/odom` at all.
`/tf` and `/tf_static` need no entry: the pose channel reads tf through its own listener, which
follows the node's namespace and remaps.

`ingest` has two values with an effect:

- `refuse` creates no subscription for the topic.
- `full_ingest`, set on an exact topic name, lets a bulk type be subscribed. The bulk types (images,
  point clouds, multi-echo scans, occupancy grids and octomaps) are otherwise refused, even when
  listed. No translator records a bulk type, so such a topic is only counted; record images and
  point clouds with [`cameras` and `lidars`](#cameras-and-lidars).

`metadata_once`, `derive_metrics` and `downsample` are currently not implemented and behave
identically: the topic is subscribed and its translator records it as it would with no `ingest`
set, at `rate_hz` where the translator reads it. `full_ingest` on a type that is not bulk does the
same. The log says so. The configure plan prints `metadata_once`, `derive_metrics` and
`downsample` as `(configured, has no effect)`. A `subscribed` line prints `ingest=` only for a value
that has an effect on that topic, or one you set, which it marks `(configured, has no effect)` when
it has none; a message type's default that does nothing is not printed. A topic's `~/diagnostics`
status gives `ingest` by the same rule, and `refuse` for a topic the node refuses.

A QoS value other than the ones listed refuses configure, naming the parameter and the values it
takes.

### Per-topic rule keys

A `topic_config` key is not a topic name. It is the entry as written in `topics`, with every run
of `/` and `*` collapsed to one `_`, and a leading or trailing `_` removed.

| entry in `topics` | `topic_config` key |
| -- | -- |
| `/odom` | `odom` |
| `/sensors/ultrasonic` | `sensors_ultrasonic` |
| `/navigate_to_pose/_action/status` | `navigate_to_pose__action_status` |
| `/robot1/*` | `robot1`, the same key as `/robot1`, so do not list both |

A topic's `topic_config` is read only when the topic has an entry in `topics`. A battery,
diagnostics or action status topic the node takes by [discovery](#discovery) has no key of its own
until you list it, and a topic only a `*` entry covers is configured by that entry's key:
`/robot1/imu` under `/robot1/*` by `topic_config.robot1`. List `/robot1/imu` as well to give it
`topic_config.robot1_imu`; the longer entry wins.

Configure refuses, naming the key, a `topic_config` key that no entry in `topics` gives, and one
whose field is not one of an entry's: `ingest`, `rate_hz`, `series`, `qos.reliability`,
`qos.durability`, `qos.history` or `qos.depth`. The first refusal lists the keys `topics` gives, and
the second the fields. Either key would otherwise be read by nothing while the run looked
configured.

At configure, the node logs the plan for every entry, with the `ingest`, `rate` and `series` it
resolved and whether each value was configured or taken from the message type.
`series=(from topic name)` on a topic you named a series for means the series is under another
entry's key.

## Discovery

The node takes two kinds of topic without an allowlist entry, because their names vary between
robots while their types do not. In a namespace, the node takes them only under its own namespace,
so run one node per robot, in that robot's namespace. In the root namespace it takes them from the
whole graph. An entry in `topics` is taken wherever it is.

| parameter | type | default | unit | what it does |
| -- | -- | -- | -- | -- |
| `auto_subscribe_standard_types` | bool | `true` | — | Subscribe every `sensor_msgs/msg/BatteryState` and `diagnostic_msgs/msg/DiagnosticArray` topic by its type. The node's own `~/diagnostics` is excluded. |
| `action_discovery` | bool | `true` | — | Record every action server whose name is in `action_names`. An action that is not enrolled records nothing, which afterwards looks the same as an action that never ran; `doctor`'s `actions` check lists the enrolled and the skipped. `false` records only the actions whose status topic is in `topics`. |
| `action_names` | string[] | the list below | — | The action names `action_discovery` enrolls, matched by the last segment of the action's name, so one entry covers every namespace the node discovers in. Setting it replaces the built-in list rather than adding to it. `action_names: []` stops the node at construction; to record no actions by name, set `action_discovery: false`. |
| `discovery_period_s` | double | `5.0` | seconds | How often the node reads the ROS graph again for topics, actions, cameras and lidars that appear late. From 0.001 to 86400. |
| `warn_on_missing_topics` | bool | `true` | — | Log, at most every 30 seconds, one line naming every allowlisted topic that is not on the graph. Entries ending in `*` are not checked. |

The built-in `action_names` cover Nav2 and the iRobot Create 3: `navigate_to_pose`,
`navigate_through_poses`, `follow_waypoints`, `spin`, `backup`, `drive_on_heading`, `wait`,
`assisted_teleop`, `dock_robot`, `undock_robot`, `dock`, `undock`, `drive_distance`, `drive_arc`,
`rotate_angle`, `navigate_to_position` and `wall_follow`. [Recording](recording.md) explains why the
planner and controller actions are left out, and which action events to build objectives on.

## Sessions

[Sessions](sessions.md) covers the lifecycle, the session services, time under simulation, and what
every session records about itself.

| parameter | type | default | unit | what it does |
| -- | -- | -- | -- | -- |
| `auto_start_session` | bool | `true` | — | Activating the node opens a session and deactivating ends it. `false`: activation opens none, and another process starts and ends sessions with `~/start_session` and `~/end_session`. `~/start_session` is refused while the node is not active. |
| `session_name` | string | `""` | — | The session's name. A `~/start_session` request with a `session_name` overrides it. Empty names the session by its id. |
| `session_properties` | string[] | `[]` | — | Extra session properties, each a `key=value` string. Values are typed: `4` is a number and `true` a boolean. A key in `c3d.` is the SDK's: it refuses configure, naming the key, and a `ros2 param set` that adds one is rejected. `c3d.remote_variable.*` is allowed only while `remote_variables` is `false`. A malformed entry is skipped with a warning. |
| `use_sim_time` | bool | `false` | — | The standard ROS 2 parameter, read once at configure. Session timestamps stay on wall-clock time unless `sim_time_on_wire` is also set. The launch file's `use_sim_time:=` argument overrides the file only when you give it. |
| `sim_time_on_wire` | bool | `false` | — | Advance session timestamps with ROS time from each session's wall-clock start, so a simulation that runs slower than real time is not stretched. Requires `use_sim_time: true`; configure refuses it without. |

## The spool

The node writes every record to an on-disk queue, the spool, before it uploads anything, so a lost
network loses nothing. [Sessions](sessions.md) describes how the spool behaves offline and after a
power loss.

| parameter | type | default | unit | what it does |
| -- | -- | -- | -- | -- |
| `spool_dir` | string | `""` | — | The spool's directory. Empty resolves at configure to `$XDG_STATE_HOME/cognitive3d`, or else `$HOME/.local/state/cognitive3d`; configure refuses when neither names an absolute directory. One process per directory: configure refuses a directory another process holds, naming its process id, and a directory it cannot write. Give every node its own, including nodes in one component container. |
| `spool_max_mb` | int | `1024` | MiB | The spool's size cap, shared by every stream. A whole number from 1 to 1073741824. At the cap the spool evicts data, lowest-priority streams first, and that data never reaches the platform. Size it for the longest time the robot runs without a network. |
| `marker_heartbeat_s` | double | `5.0` | seconds | How often a live session refreshes its marker in the spool. Must be positive. |
| `orphan_stale_after_s` | double | `30.0` | seconds | How long a session's marker may go unrefreshed before the next configure closes that session as ended abnormally. Must exceed `marker_heartbeat_s` by more than 0.05 seconds. |

## Uploads

An upload is judged by whether it is moving, not by how long it takes, so a large object on a slow
uplink is not abandoned while it progresses. [Sessions](sessions.md) describes what happens to data
the platform refuses.

| parameter | type | default | unit | what it does |
| -- | -- | -- | -- | -- |
| `uploads_paused` | bool | `false` | — | Record and spool as normal, and upload nothing, for a run whose uplink cannot carry the data. Applied at once, also at runtime: `ros2 param set /cognitive3d uploads_paused false` resumes uploads. Size `spool_max_mb` for the whole paused run, or the spool evicts at the cap. |
| `http_connect_timeout_s` | int | `10` | seconds | How long one upload may take to connect. A positive whole number, no larger than `http_timeout_s`. |
| `http_low_speed_limit_bytes_per_s` | int | `1024` | bytes per second | An upload that moves fewer bytes per second than this for `http_low_speed_time_s` is abandoned and retried. A positive whole number. |
| `http_low_speed_time_s` | int | `60` | seconds | The window for `http_low_speed_limit_bytes_per_s`. A positive whole number, shorter than `http_timeout_s`. |
| `http_timeout_s` | int | `300` | seconds | The ceiling for one upload request. A positive whole number. The default carries a 2 MiB upload, the size at which a point-cloud object closes and well above a typical camera chunk or scan block, at about 7 KB per second. A single larger cloud, a camera chunk at a high resolution and a low `crf`, or a slow uplink needs a higher `http_timeout_s`. |
| `retry_min_s` | double | `60.0` | seconds | The wait after a failed upload. It doubles with each failure up to `retry_max_s`, and resets on success. Must be positive, and no larger than `retry_max_s`. |
| `retry_max_s` | double | `240.0` | seconds | The longest wait between retries. While the platform refuses the application key, uploads pause and try again this often. |

## Cameras and lidars

Both streams are off until a name appears in `cameras` or `lidars`. A name is a label you choose,
not the topic: it reaches the session as the sensor's id and names its `c3d.camera.<name>.*` or
`c3d.lidar.<name>.*` counters. [Camera and lidar](camera-and-lidar.md) covers poses, bandwidth and
what is refused.

| parameter | type | default | unit | what it does |
| -- | -- | -- | -- | -- |
| `cameras` | string[] | `[]` | — | The camera names, each configured under `camera.<name>.*`. A name must be non-empty, carry no `.` or `/`, and appear once. A `camera.<name>.<field>` key whose name is not listed here, or whose field is not one of those below, refuses configure, naming the key. |
| `camera.<name>.topic` | string | `""` | — | A `sensor_msgs/msg/Image` or `sensor_msgs/msg/CompressedImage` topic. Required for every listed name: empty refuses configure. A topic of another type is refused when it appears on the graph, not at configure; `doctor` checks it. |
| `camera.<name>.rate_hz` | double | `30.0` | Hz | Frames accepted per second, counted on arrival. They are encoded into one H.264 chunk per 3 seconds of real time; the rate does not set a chunk's length. Must be positive. |
| `camera.<name>.preset` | string | `"ultrafast"` | — | x264's speed preset, `ultrafast` or `veryfast`. `veryfast` spends more CPU for fewer bytes. |
| `camera.<name>.crf` | int | `28` | — | The constant rate factor, 24 to 32; lower is higher quality and more bytes. |
| `camera.<name>.max_bitrate_kbps` | int | `0` | kilobits per second | A cap on the video's bitrate, at most 1,000,000 (1 Gbit/s). `0` sets no cap. Must not be negative. |
| `camera.<name>.max_width_px` | int | `640` | pixels | Wider frames are scaled down to this width, keeping the aspect ratio. `0` never resizes. |
| `camera.<name>.frame` | string | `""` | — | The tf frame each frame's pose is looked up for. Empty uses the message's `header.frame_id`. A frame whose pose cannot be looked up is dropped, so a frame that never resolves records nothing. |
| `camera.<name>.hfov_degrees` | double | `0.0` | degrees | The camera's horizontal field of view, a full angle, sent with every chunk. `0` is unset. Set it with `vfov_degrees` or not at all; above 0 and below 180. Set, it wins over the CameraInfo. See [A camera's field of view](camera-and-lidar.md#a-cameras-field-of-view). |
| `camera.<name>.vfov_degrees` | double | `0.0` | degrees | The vertical field of view, as for `hfov_degrees`. |
| `camera.<name>.camera_info_topic` | string | `"auto"` | — | The `sensor_msgs/msg/CameraInfo` topic each chunk's field of view is computed from, as of its first frame, while `hfov_degrees` and `vfov_degrees` are unset. `"auto"` is the image topic's sibling, `/cam/camera_info` for `/cam/image_raw` or `/cam/image_raw/compressed`; `""` reads none. |
| `camera.<name>.qos.reliability` | string | `""` | — | As for `topic_config.<key>.qos.reliability`. |
| `camera.<name>.qos.durability` | string | `""` | — | As for `topic_config.<key>.qos.durability`. |
| `camera.<name>.qos.history` | string | `""` | — | As for `topic_config.<key>.qos.history`. |
| `camera.<name>.qos.depth` | int | `-1` | messages | As for `topic_config.<key>.qos.depth`. |
| `lidars` | string[] | `[]` | — | The lidar names, each configured under `lidar.<name>.*`, with the same rules as `cameras`. A `lidar.<name>.<field>` key whose name is not listed here, or whose field is not one of those below, refuses configure, naming the key. A lidar has no `qos.*` fields. |
| `lidar.<name>.topic` | string | `""` | — | A `sensor_msgs/msg/LaserScan` or `sensor_msgs/msg/PointCloud2` topic. Required for every listed name. |
| `lidar.<name>.rate_hz` | double | `30.0` | Hz | Scans accepted per second, counted on arrival. A `LaserScan` is stored in one block per 3 seconds of scans; point clouds are packed into objects that close at 2 MiB or 5 seconds. Must be positive. |
| `lidar.<name>.frame` | string | `""` | — | As for a camera. Also set it when a point cloud's header names a frame its points are not in, which `doctor`'s optical frame check detects. |
| `lidar.<name>.intensities` | bool | `false` | — | `LaserScan` only. Record intensities too, which doubles a scan's size. |
| `lidar.<name>.pointcloud.voxel_leaf_m` | double | `0.05` | meters | `PointCloud2` only. The downsampling cell size, from 0 to 65; `0` keeps every point. |
| `lidar.<name>.pointcloud.position` | string | `"int16_mm"` | — | `int16_mm` or `float32`. `int16_mm` reaches 32.767 meters from the sensor's frame; a point beyond that is dropped and counted in `c3d.lidar.<name>.dropped_range`. Use `float32` for a long-range lidar. |
| `lidar.<name>.pointcloud.color` | string | `"none"` | — | `none`, `rgb565`, `rgb332` or `rgb888`. |
| `lidar.<name>.pointcloud.intensity` | string | `"none"` | — | `none` or `uint8`. |
| `lidar.<name>.pointcloud.max_points` | int | `100000` | points | A cloud still larger than this after downsampling is dropped whole, and counted in `c3d.lidar.<name>.dropped_oversize`. At least 1. |
| `lidar.<name>.pointcloud.zstd_level` | int | `3` | — | The compression level, from 0 to 22; `0` sends the points uncompressed. |
| `ffmpeg_path` | string | `"ffmpeg"` | — | The camera encoder binary, searched on `PATH` unless it contains a `/`. With a camera configured, configure refuses when it cannot run or lists no `libx264` encoder. |

## Described sensors

**Provisional.** These parameters may be replaced by a robot manifest read from the robot's URDF in
a later release.

A plain sensor's readings arrive as a `std_msgs/msg/Float32` or `std_msgs/msg/Int32MultiArray`, and
neither type says what the sensor is. A name in `sensors` describes one: the node records the
description as `c3d.ros.sensor.<name>.*` session properties at every session start, and Session
Replay draws the sensor from them. A description records nothing from the topic itself; list the
topic in `topics` as well, which configure checks. The series the description names is the one the
topic records under: its `topic_config.<key>.series`, or the name derived from the topic. Session
Replay draws the cone of a `cone_rangefinder` named `ultrasonic`, and the state lanes of every
`binary_array`. [Data format](data-format.md#session-properties) lists the properties.

| parameter | type | default | unit | what it does |
| -- | -- | -- | -- | -- |
| `sensors` | string[] | `[]` | — | The sensor names, each configured under `sensor.<name>.*`. A name must be non-empty, carry no `.` or `/`, and appear once. A `sensor.<name>.<field>` key whose name is not listed here, or whose field is not one of those below, refuses configure, naming the key. |
| `sensor.<name>.kind` | string | `""` | — | `cone_rangefinder`, a `Float32` distance with a fixed cone, or `binary_array`, an `Int32MultiArray` of one-bit elements. Required; any other value refuses configure. A field of the other kind left set refuses configure too. |
| `sensor.<name>.topic` | string | `""` | — | The topic the readings are recorded from. Required, and it must be recorded through `topics`. |
| `sensor.<name>.frame` | string | `""` | — | The sensor's own tf frame. Empty records none. |
| `sensor.<name>.mount_parent` | string | `""` | — | The frame the mount is given in. Empty records none. |
| `sensor.<name>.mount_measured` | bool | `false` | — | Whether the mount was measured rather than estimated. |
| `sensor.<name>.cone_half_angle_deg` | double | `0.0` | degrees | `cone_rangefinder` only. Half the cone's full angle, above 0 and below 90. Required. |
| `sensor.<name>.units` | string | `""` | — | `cone_rangefinder` only. The unit the topic publishes in; the series is recorded unconverted. `meters`, `metres`, `m`, `centimeters`, `centimetres`, `cm`, `millimeters`, `millimetres` or `mm`. Required. |
| `sensor.<name>.range_min_m` | double | `0.0` | meters | `cone_rangefinder` only. The shortest distance the sensor reports, recorded in centimeters. `0` records none. Below `range_max_m` when both are set. |
| `sensor.<name>.range_max_m` | double | `0.0` | meters | `cone_rangefinder` only. The longest distance, recorded in centimeters. `0` records none. |
| `sensor.<name>.mount_xyz_m` | double[] | `[]` | meters | `cone_rangefinder` only. The sensor's position in `mount_parent`: `[x, y, z]`. Empty records none. |
| `sensor.<name>.mount_yaw_deg` | double | `0.0` | degrees | `cone_rangefinder` only. The sensor's yaw in `mount_parent`, recorded in radians with `mount_xyz_m`. Set without `mount_xyz_m`, it refuses configure. |
| `sensor.<name>.element_count` | int | `0` | elements | `binary_array` only. How many elements the array carries, from 1 to 32, each recorded as its own series. Required. |
| `sensor.<name>.meaning_0` | string | `""` | — | `binary_array` only. What an element's `0` means, recorded verbatim. Empty records none. |
| `sensor.<name>.meaning_1` | string | `""` | — | `binary_array` only. What an element's `1` means. |

## Remote variables

[Remote variables](remote-variables.md) covers setting up a test on the dashboard and keeping each
run's draw independent.

| parameter | type | default | unit | what it does |
| -- | -- | -- | -- | -- |
| `remote_variables` | bool | `true` | — | Fetch remote variables and A/B test arms once per session, and record them as `c3d.remote_variable.*` session properties. A failed fetch records a `c3d.remote_variables.unavailable` event. `false` skips the fetch, and lets `~/start_session` supply `c3d.remote_variable.*` properties itself. |
| `remote_variables_timeout_s` | double | `3.0` | seconds | The fetch's timeout; it delays the start of a session by at most this long. Rounded up to whole seconds and held between 1 and 60, with a warning when it is outside. Must be positive, even with `remote_variables: false`. |

## Diagnostics and limits

| parameter | type | default | unit | what it does |
| -- | -- | -- | -- | -- |
| `report_period_s` | double | `10.0` | seconds | How often the node logs its report and publishes `~/diagnostics`. From 0.001 to 86400. |
| `event_rate_limit_hz` | double | `5.0` | events per second | The most events each source may record per second: each topic is one source, and everything published on `~/events` is one more. Excess events are dropped and counted in `c3d.events_suppressed`. `0` removes the cap; a negative value refuses configure. Sensor series are not limited. |
| `dry_run` | bool | `false` | — | Subscribe, translate and count, but open no session and upload nothing. `doctor` always sets it. |

## Development parameters

These exist for capturing uploads locally while developing the SDK. Leave both at their defaults on
a robot.

| parameter | type | default | unit | what it does |
| -- | -- | -- | -- | -- |
| `gateway` | string | `""` | — | Sends telemetry to this host instead of the data host the environment names, for a local capture stub. It moves only the data host; metadata always comes from the environment. A value naming another Cognitive3D environment's data host refuses configure. |
| `protocol` | string | `"https"` | — | `https`, or `http` for a capture stub. `http` sends the application key unencrypted, so configure refuses it unless `gateway` names a host that is not a Cognitive3D host. |
