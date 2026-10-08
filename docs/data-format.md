# Data format

This page describes what a session recorded by the node stores on the Cognitive3D platform: the
session itself, the pose timeline, events, sensor series, session properties, and the camera and
lidar records. It is a reference for people who read stored sessions or build tools on them. The
node writes all of it; nothing here has to be built by hand.

## Sessions

A session is identified by its session id, `<unix seconds>_<device id>`: the session's start time
in whole seconds and the node's `device_id`. Every record of every stream carries it, and it is
what joins them. Between one configure and the next, each new session id is at least one second
later than the last one the node issued, so sessions started in quick succession do not share an
id; the start time in the id can therefore run a few seconds ahead of the wall clock, and the
session's records move with it (see [Timestamps](#timestamps)). Across a
cleanup and configure, or a restart, two sessions begun within one second can share an id: see
[Starting a session from another node](sessions.md#starting-a-session-from-another-node). Give
every robot its own `device_id`: two robots that share one can produce the same session id.

A session belongs to one scene version: the `scene_id` and `scene_version` the node was configured
with. Its first event is `c3d.sessionStart`, at the session's start time. Its last is
`c3d.sessionEnd`, with two properties: `Reason`, why it ended, and `sessionlength`, its length in
seconds. [Ending a session](sessions.md#ending-a-session) lists the reasons the node records. A session the node could not end, because the robot lost power for example, is closed the
next time a node is configured with the same spool, with `Reason` set to `abnormalEnd` and
`c3d.recovered` set to `true`.

## Timestamps

Every timestamp is Unix epoch time on the robot's wall clock. A session whose start was moved
ahead of the wall clock, to keep its id unique, records on the wall clock moved by the same amount,
so no record but a camera or lidar frame stamped earlier precedes its `c3d.sessionStart`. The pose
timeline, events and sensor series store seconds, as a number with five decimal places. Camera and lidar records store whole
milliseconds.

The clock rule is that ROS time never reaches the platform. A record the node takes itself, such as
a pose sample or a sensor sample, is stamped with the wall-clock time it was taken. A record placed
by a message's header stamp, such as a camera frame, a lidar scan or a stamped event, is stamped
with the wall-clock time now, less the message's age on the ROS clock. A stamp of zero, a stamp more
than 1 second ahead of the ROS clock, and a stamp more than an hour behind it are each placed at the
time the message arrived instead. An event stamped before its session started is placed at the
session's start.

Under `use_sim_time`, the same rule keeps the simulator's clock, which starts near zero, off the
platform. Under `sim_time_on_wire`, timestamps are the session's wall-clock start plus the ROS time
elapsed since then, so records are spaced by simulated time and the session still lands at its real
date. Every session records which rule it used, in `c3d.ros.use_sim_time` and
`c3d.ros.sim_time_on_wire`. [Sessions](sessions.md) describes both settings.

## Coordinates and units

Positions are in meters, as `[x, y, z]`. Rotations are quaternions, as `[x, y, z, w]` with `w`
last. The pose timeline, events and sensor series round values to five decimal places; camera and
lidar records store 32-bit floats.

Every position and rotation is in the session's reference frame, converted by the session's pose
convention, which the session records in `c3d.ros.pose_convention`. With `(x, y, z)` a ROS position
and `(x, y, z, w)` a ROS quaternion, in REP-103 axes:

| convention | position | pose timeline rotation | Dynamic Object, camera and point cloud rotation |
| -- | -- | -- | -- |
| `unity` | `(-y, z, x)` | `(-y, z, x, -w)` | `(-y, z, x, -w)` |
| `gltf_authored` | `(-x, z, -y)` | `(s(x + y), -s(z + w), s(y - x), s(w - z))`, with `s` = √½ | `(x, -z, y, w)` |
| `rep103` | `(x, y, z)`, unchanged | unchanged | unchanged |

A `LaserScan` record's rotation follows its own rule; see
[Reading a sensor pose](#reading-a-sensor-pose). [Coordinates](coordinates.md) explains how to
choose a convention and what each one is for.

## The pose timeline

The pose timeline is what Session Replay draws the robot from. It holds one record per pose
sample, taken `sample_rate_hz` times a second (10 by default):

| field | what |
| -- | -- |
| `time` | When the sample was taken |
| `p` | The position of the gaze frame: `gaze_frame` when one is set, otherwise `robot_frame` |
| `r` | The rotation of that frame |

A record has no gaze point, `g`: a robot has no eyes, and the platform counts every gaze point it
is sent in its gaze analytics.

Under `unity` and `gltf_authored`, the frame's forward axis is the local +Z axis of the stored
rotation `r` and its up axis the local +Y; under `rep103`, local +X and +Z. When the gaze frame is an optical frame (Z forward), the node turns it to a body
frame (X forward) before converting it, and records which it did in
`c3d.ros.gaze_frame_convention`.

A sample is skipped when tf cannot answer, when the robot's transform has stopped advancing, and
while the node waits for its preferred reference frame; [Coordinates](coordinates.md) covers each.
The first two are counted in the `c3d.pose_failures` series, and a stopped transform in
`c3d.stale_poses` as well. Samples held back while the node waits for its reference frame are not
counted.

## Events

An event is a named point in time and space:

| field | what |
| -- | -- |
| `name` | The event's name |
| `time` | When it happened |
| `point` | Where the robot was, in scene coordinates |
| `properties` | Named values: strings, numbers and booleans |

An event recorded before the node has a pose has its `point` at the origin and the property
`c3d.position_unknown` set to `true`. [Recording](recording.md) lists the events the node records
and how to publish your own.

## Sensor series

A sensor series is a name and a list of `[time, value]` pairs, with every value a number. A series
is change-gated: a sample is stored when it differs from the last stored value by more than 0.001,
or when 2 seconds or more have passed since the last stored sample. A value that does not change is
therefore stored at most every 2 seconds. [Recording](recording.md) names the series the node
records from your topics.

The node also records its own health into every session, every 5 seconds of session time (5
simulated seconds under `sim_time_on_wire`), as series named `c3d.*`: counts of failed pose
samples, dropped records and events, and the state of the spool and the uploads.
[Health series](recording.md#health-series) lists each one, and whether it is a running count or a
current value. Each configured camera and lidar adds its own counters; see
[Camera and lidar](camera-and-lidar.md#what-the-session-records).

## The robot's Dynamic Object

With `robot_dynamic_object: true`, the session also registers the robot as a Dynamic Object, under
`robot_object_id`, with `robot_mesh` as its mesh and `robot_display_name` as its name. Each pose
sample then adds a snapshot of the robot frame: the object id, `time`, a position `p` and a rotation
`r`, the rotation by the Dynamic Object rule in the table above and with `mesh_yaw_offset_deg`
applied. With the default, `robot_dynamic_object: false`, the session has no Dynamic Object data at
all.
[Scenes](scenes.md#the-robot-as-a-dynamic-object) describes the setup.

## Session properties

Session properties describe the whole session. The node sets these on every session:

| property | value |
| -- | -- |
| `c3d.version` | The SDK version, such as `1.0.0` |
| `c3d.app.name` | `cognitive3d_ros` |
| `c3d.app.version` | The SDK version |
| `c3d.app.engine` | `ROS2` |
| `c3d.app.engine.version` | The ROS distro, such as `humble` or `jazzy` |
| `c3d.app.sdktype` | `ros` |
| `c3d.deviceid` | `device_id` |
| `c3d.device.os` | The host operating system's name, from `/etc/os-release` |
| `c3d.participant.id` | The `participant_id` given to `~/start_session`, otherwise `device_id` |
| `c3d.sessionname` | The name given to `~/start_session`, otherwise `session_name`, otherwise the session id |
| `c3d.ros.distro` | `ROS_DISTRO`, or `unknown` |
| `c3d.ros.rmw` | The RMW implementation, such as `rmw_fastrtps_cpp` |
| `c3d.ros.domain_id` | `ROS_DOMAIN_ID`, or `unset` |
| `c3d.ros.discovery_range` | `ROS_AUTOMATIC_DISCOVERY_RANGE`, or `unset`. Not recorded on Humble, which has no such variable |
| `c3d.ros.namespace`, `c3d.ros.node` | The node's namespace and name |
| `c3d.ros.pose_convention` | `gltf_authored`, `unity` or `rep103` |
| `c3d.ros.use_sim_time`, `c3d.ros.sim_time_on_wire` | Which clock rule the session used |
| `c3d.ros.robot_dynamic_object` | Whether the session has the robot's Dynamic Object |

Some properties are present only when a feature is on:

| property | present when | value |
| -- | -- | -- |
| `c3d.ros.gaze_frame` | `gaze_frame` is set | The gaze frame |
| `c3d.ros.gaze_frame_convention` | `gaze_frame` is set | `body_forward_x` or `optical_forward_z`, as resolved |
| `c3d.ros.cameras`, `c3d.ros.camera.<name>.*`, `c3d.ros.lidars`, `c3d.ros.lidar.<name>.*` | cameras or lidars are configured | See [Camera and lidar](camera-and-lidar.md#what-the-session-records) |
| `c3d.ros.sensors`, `c3d.ros.sensor.<name>.*` | `sensors` is set | See [Described sensors](#described-sensors) |
| `c3d.remote_variable.<name>` | remote variables are fetched | See [Remote variables](remote-variables.md) |

Your own `session_properties` and the `properties_json` of `~/start_session` are stored beside
these, typed as numbers, booleans or strings.

### Described sensors

**Provisional**, like the [`sensors`](configuration.md#described-sensors) parameters. Each name in
`sensors` is recorded at every session start, and `c3d.ros.sensors` lists the names,
comma-separated. A number whole in its unit is an integer, and any other a double, as the same
text in `session_properties` would be typed; `mount_yaw_radians` is always a double.

| property | kind | value |
| -- | -- | -- |
| `c3d.ros.sensor.<name>.kind` | both | `cone_rangefinder` or `binary_array` |
| `c3d.ros.sensor.<name>.topic` | both | The topic, resolved |
| `c3d.ros.sensor.<name>.series` | both | The series the readings are recorded under. For a `binary_array`, one per element, comma-separated: `line.0,line.1,line.2,line.3` |
| `c3d.ros.sensor.<name>.frame`, `.mount_parent` | both | The frames, when set |
| `c3d.ros.sensor.<name>.mount_measured` | both | `true` or `false` |
| `c3d.ros.sensor.<name>.units` | `cone_rangefinder` | The series' unit, such as `centimeters` |
| `c3d.ros.sensor.<name>.cone_half_angle_degrees` | `cone_rangefinder` | The cone's half-angle |
| `c3d.ros.sensor.<name>.range_min_centimeters`, `.range_max_centimeters` | `cone_rangefinder` | The range limits, when set, in centimeters whatever `units` says |
| `c3d.ros.sensor.<name>.mount_xyz_meters` | `cone_rangefinder` | Text, `x,y,z` in meters with two to five decimals: `0.10,0.00,0.05`. When set |
| `c3d.ros.sensor.<name>.mount_yaw_radians` | `cone_rangefinder` | The mount's yaw, with `mount_xyz_meters` |
| `c3d.ros.sensor.<name>.element_count` | `binary_array` | The number of elements |
| `c3d.ros.sensor.<name>.meaning_0`, `.meaning_1` | `binary_array` | What a `0` and a `1` mean, when set |

## Camera frames

A camera frame record is one picture and the pose it was taken from:

| field | type | what |
| -- | -- | -- |
| `cameraId` | text | The camera's name from `cameras` |
| `time` | UInt64 | When the frame was taken, in epoch milliseconds, by the clock rule |
| `seq` | UInt32 | The camera's frame counter: 0 for the first frame stored in the session, then 1, 2 and so on |
| `frameId` | text | The tf frame the pose is for |
| `pose` | 7 x Float32 | `px py pz qx qy qz qw`; see [Reading a sensor pose](#reading-a-sensor-pose) |
| `format` | text | `jpeg` |
| `width`, `height` | UInt16 | The stored image's size in pixels, after any scaling |
| `data` | bytes | The JPEG image |
| `hfovDegrees`, `vfovDegrees` | Float32 | The camera's horizontal and vertical field of view when the frame was taken, as full angles in degrees, both above 0 and below 180. Both are 0 when it was not recorded |

Camera and lidar records upload in parts, each naming its session id, device id, scene id, scene
version number and SDK version. Parts are numbered from 1 per stream per session, and the platform
keeps the part number with every record, so a missing or repeated part number shows a part that was
lost or delivered twice.

## Lidar scans

A lidar scan record is one scan and the pose it was taken from:

| field | type | what |
| -- | -- | -- |
| `sensorId` | text | The lidar's name from `lidars` |
| `time` | UInt64 | When the scan was taken, in epoch milliseconds, by the clock rule |
| `seq` | UInt32 | The lidar's scan counter, from 0 in each session |
| `frameId` | text | The tf frame the pose is for |
| `pose` | 7 x Float32 | `px py pz qx qy qz qw`; see [Reading a sensor pose](#reading-a-sensor-pose) |

followed by exactly one of a laser scan or a point cloud.

### Laser scan

A laser scan is stored as the sensor published it, in its own units and frame:

| field | type | what |
| -- | -- | -- |
| `angleMin`, `angleMax` | Float32 | The first and last ray's angle, in radians |
| `angleIncrement` | Float32 | The angle between rays, in radians |
| `timeIncrement` | Float32 | The time between rays, in seconds |
| `scanTime` | Float32 | The time between scans, in seconds |
| `rangeMin`, `rangeMax` | Float32 | The sensor's valid range, in meters |
| `ranges` | list of Float32 | One range per ray, in meters. Infinities and NaNs are kept: they are the sensor saying there was no return |
| `intensities` | list of Float32 | One per ray, or empty when the lidar is not configured to store them |

### Point cloud

A point cloud is six fields describing one block of point data:

| field | type | what |
| -- | -- | -- |
| `pointCount` | UInt32 | N, the number of points in the block |
| `position` | text | `int16_mm` or `float32` |
| `color` | text | `none`, `rgb565`, `rgb332` or `rgb888` |
| `intensity` | text | `none` or `uint8` |
| `voxelLeafMm` | UInt16 | The downsampling cube's size in millimeters; 0 when the cloud was not downsampled |
| `zstdLevel` | UInt8 | 0 when the block is stored as it is; above 0, the block is zstd-compressed at that level |
| `data` | bytes | The block |

The fields describe the block as written. A cloud recorded with `color: rgb565` from a sensor with
no color field is stored with `color` set to `none`, while the session property
`c3d.ros.lidar.<name>.pointcloud.color` records the `rgb565` asked for.

To read the block, decompress it when `zstdLevel` is above 0, then read these planes in this order,
little-endian, each N values long:

```text
X[N] Y[N] Z[N]   int16 each when position is "int16_mm" (millimeters: divide by 1000), float32 each when "float32"
C[N]             absent when color is "none"; uint16 for rgb565, uint8 for rgb332, 3 x uint8 for rgb888
I[N]             uint8, 0 to 255; absent when intensity is "none"
```

The points are in scene axes, relative to the sensor, and ordered by position: the sensor's own row
and column layout is not kept. A downsampled point is the average of the points in its cube, its
color the average of theirs.

Color is packed, and unpacks by repeating each channel's high bits into its low bits, so a mid grey
stays mid grey and `0x00` and `0xFF` unpack to exact black and white:

```cpp
// rgb565, one uint16 per point
r = (v >> 11) << 3 | (v >> 13);
g = ((v >> 5) & 63) << 2 | ((v >> 9) & 3);
b = (v & 31) << 3 | ((v >> 2) & 7);

// rgb332, one uint8 per point: red in bits 7-5, green in 4-2, blue in 1-0
r = (v & 0xE0) | ((v & 0xE0) >> 3) | ((v & 0xE0) >> 6);
g = ((v << 3) & 0xE0) | (v & 0x1C) | ((v >> 3) & 0x03);
b = ((v & 0x03) << 6) | ((v & 0x03) << 4) | ((v & 0x03) << 2) | (v & 0x03);
```

`rgb888` is the three bytes `r g b` as they were. A `uint8` intensity is the sensor's intensity
multiplied by 255 when the sensor published it as a float whose values are all 1.0 or less, and
otherwise the sensor's value clamped to 0 to 255.

## Reading a sensor pose

A camera or lidar record's `pose` is a position `(px, py, pz)` and a quaternion `(qx, qy, qz, qw)`:
the pose of `frameId` in the session's reference frame, converted by the session's pose convention.
The position is converted the same way in every record. The rotation follows one of two rules,
depending on what the record's own coordinates are:

| record | the rotation is | in the scene |
| -- | -- | -- |
| camera frame | The sensor's rotation in scene axes, by the Dynamic Object rule | A direction `v` in the frame's own ROS axes points along `rotate(q, M v)` |
| point cloud | The sensor's rotation in scene axes, by the Dynamic Object rule | A stored point `p` is at `rotate(q, p) + position` |
| laser scan | The rotation of the scan plane | A ray is at `rotate(q, (r cos t, r sin t, 0)) + position` |

`M` is the session's position map from [Coordinates and units](#coordinates-and-units). A camera on
an optical frame looks along `v = (0, 0, 1)`, and one on a body frame along `v = (1, 0, 0)`. A stored
point cloud point is already `M` applied to the sensor's own point. For a laser scan, ray `i` has
angle `t = angleMin + i * angleIncrement` and range `r = ranges[i]`; skip a ray whose range is not
finite or lies outside `rangeMin` to `rangeMax`. The point cloud and laser scan rules need nothing
but the record; a camera's viewing direction also needs `M`, which `c3d.ros.pose_convention`
names.

The two rotations differ because a laser scan's rays are stored unconverted. Under `unity` and
`gltf_authored`, the axis map is a reflection, which no rotation can express, so a scan's record
carries the rotation that agrees with the reflected sensor on the scan plane, and the reflection
lands on the axis no ray uses. A laser scan and a camera on the same tf frame at the same moment
therefore have the same position and different rotations. Under `rep103` the two rules agree. A
reader that applies the point cloud rule to a laser scan draws the scan turned out of place.

## Schemas

The camera and lidar records are defined by Cap'n Proto schemas, and the node uploads them as
packed Cap'n Proto messages. The schemas are in the repository, in
`core/capnp/camera_payload.capnp`, `core/capnp/lidar_payload.capnp` and
`core/capnp/common_payload.capnp`, which holds the `Pose` both use. A `Pose` that was never written
reads as the identity rotation at the origin, because `qw` defaults to 1; a pose the node stores is
always a real tf lookup.
