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
sample, taken `sample_rate_hz` times a second (30 by default):

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

## Camera chunks

A camera record is one chunk: one camera's frames for about 3 seconds of real time, or 1 to 3
seconds for a camera whose 3-second chunks would be too large, encoded as H.264
and stored as one object. A chunk is one closed group of pictures, so it decodes on its own, and its
first frame is always a keyframe. The record is a `CameraPayload`:

| field | type | what |
| -- | -- | -- |
| `userId`, `sessionId`, `sceneId`, `sceneVersion`, `formatVersion`, `sessionStartMs`, `sdkVersion` | | The session the chunk belongs to, the scene version number, `"2.0"` and the SDK version |
| `cameraId` | text | The camera's name from `cameras` |
| `seq` | UInt32 | The camera's chunk counter: 0 for the first chunk stored in the session, then 1, 2 and so on |
| `startMs`, `endMs` | UInt64 | The first and the last frame's `time`, in epoch milliseconds |
| `codec` | text | `h264` |
| `codecParams` | bytes | The stream's SPS and PPS as an avcC record |
| `width`, `height` | UInt16 | The stored image's size in pixels, after any scaling |
| `nominalFps` | Float32 | The camera's `rate_hz`: the cap on what is fed to the encoder, not a measurement |
| `frameId` | text | The tf frame the poses are for; the same for every frame in the chunk |
| `hfovDegrees`, `vfovDegrees` | Float32 | The camera's horizontal and vertical field of view at the chunk's first frame, as full angles in degrees, both above 0 and below 180. Both are 0 when it was not recorded |
| `encoderSettings` | text | What the encoder ran with, for diagnosis, such as `x264 ultrafast crf=28 threads=1 gop=3s`, followed by `maxrate=800k` when `max_bitrate_kbps` caps the bitrate at 800 |
| `frames` | list of `FrameMeta` | One per frame, in decode order |
| `data` | bytes | The H.264 stream: every frame's access unit, back to back |

A `FrameMeta` is one frame and the pose it was taken from:

| field | type | what |
| -- | -- | -- |
| `time` | UInt64 | When the frame was taken, in epoch milliseconds, by the clock rule |
| `seq` | UInt32 | The camera's frame counter: 0 for the first frame stored in the session, then 1, 2 and so on. A frame the node dropped leaves no hole in a chunk, but leaves one in `seq` |
| `pose` | 7 x Float32 | `px py pz qx qy qz qw`; see [Reading a sensor pose](#reading-a-sensor-pose) |
| `byteOffset`, `byteLength` | UInt32 | Where this frame's access unit lies inside `data` |
| `keyframe` | Bool | `true` for the chunk's first frame, and for no other |

The stream is Annex-B: each access unit starts with an access unit delimiter and its NAL units carry
start codes. The SPS and PPS are in-band, before every IDR, so the chunk's first access unit carries
them and a chunk is self-contained. A chunk's frames are decoded in the order of
`frames`, which is the order of `time`.

`codecParams` is the avcC record for the stream's parameter sets. It gives a reader the codec
string, `avc1.` followed by the record's profile, constraint and level bytes as six hexadecimal
digits (`avc1.PPCCLL`), and the profile. It is not a decoder configuration for the stored bytes:

- **A WebCodecs reader configures the decoder with the `avc1.PPCCLL` string built from
  `codecParams`, not with the payload's `codec` field, which says only `h264`, and with no
  `description`.** WebCodecs reads
  length-prefixed AVC whenever a `description` is given, and the stored units are Annex-B. A reader
  that wants to pass the avcC record as `description` converts each access unit to length-prefixed
  form before `decode()`.
- **ffmpeg reads a chunk's `data` as a raw stream**, `ffmpeg -f h264 -i chunk.h264`. The stream
  carries no timestamps: take each frame's time from its `FrameMeta`.

Every frame in a chunk has the same size, and a change of image size or of the tf frame ends the
chunk early and starts the next one. A camera whose field of view changes records a different angle
in each chunk, not in each frame. A chunk also ends before its `data` would pass 8 MiB, and the
next chunk starts with that frame, encoded again as a keyframe, so a chunk can be shorter than its
time without a frame missing. A chunk's first frame is never encoded again: a keyframe over 9 MiB is
dropped, with the camera's frames until the next keyframe, and counted in
`c3d.camera_access_units_dropped`.

The platform refuses an object larger than 10 MiB, and the SDK deletes an object the platform
refuses, so it builds none that large: a chunk's `data` closes at 8 MiB, a scan block's codes at
8 MiB, and `max_points` bounds one cloud to 8 MiB, which leaves room for the rest of the payload.

Camera and lidar objects upload one at a time, each naming its session id, device id, scene id,
scene version number and SDK version. The per-sensor `seq` numbers them from 0 per camera or lidar
per session, and the platform keeps it with every object, so a missing or repeated `seq` shows an
object that was lost or delivered twice. A lidar's scan blocks and its cloud objects are numbered
apart, so a lidar whose topic changes message type mid-session can repeat a `seq` without anything
being lost or delivered twice.

## Lidar blocks

A lidar record is one block of one sensor's scans or clouds. It is a `LidarPayload`:

| field | type | what |
| -- | -- | -- |
| `userId`, `sessionId`, `sceneId`, `sceneVersion`, `formatVersion`, `sessionStartMs`, `sdkVersion` | | As for a camera chunk |
| `sensorId` | text | The lidar's name from `lidars` |
| `seq` | UInt32 | The lidar's block counter, from 0 in each session |
| `startMs`, `endMs` | UInt64 | The first and the last record's `time`, in epoch milliseconds |
| `frameId` | text | The tf frame the poses are for; the same for every record in the block |

followed by exactly one of `scans`, a `ScanBlock`, or `clouds`, a `CloudBlock`.

### Laser scan blocks

A `ScanBlock` holds the scans of one `LaserScan` sensor over about 3 seconds of stamps. A block
also closes before its codes, 2 bytes a ray and twice that with intensities, would pass 8 MiB, so a
lidar whose stamps stop advancing still makes bounded blocks. Its header is the scans' geometry,
which every scan in the block shares; a scan with different geometry, or with a different number
of rays, starts a new block:

| field | type | what |
| -- | -- | -- |
| `angleMin`, `angleMax` | Float32 | The first and last ray's angle, in radians |
| `angleIncrement` | Float32 | The angle between rays, in radians |
| `rangeMin`, `rangeMax` | Float32 | The sensor's valid range, in meters |
| `rays` | UInt16 | The number of rays in each scan |
| `rangeUnit` | enum | `millimetre` or `centimetre`: the size of one range code |
| `encoding` | enum | `deltaShuffleZstd`, or `uint16` for the same codes with no transform |
| `entries` | list of `ScanEntry` | One per scan, in time order |
| `ranges` | bytes | K x `rays` uint16 codes for K entries, laid out as `encoding` says |
| `intensities` | bytes | Empty unless the lidar is configured to store them; otherwise the same layout as `ranges` |

A `ScanEntry` is one scan and the pose it was taken from:

| field | type | what |
| -- | -- | -- |
| `time` | UInt64 | When the scan was taken, in epoch milliseconds, by the clock rule |
| `seq` | UInt32 | The lidar's scan counter, from 0 in each session |
| `pose` | 7 x Float32 | `px py pz qx qy qz qw`; see [Reading a sensor pose](#reading-a-sensor-pose) |
| `timeIncrement` | Float32 | The time between rays in this scan, in seconds |
| `scanTime` | Float32 | The time this sweep took, in seconds |

The timing is per scan because drivers measure it per sweep: a spinning lidar reports a different
`scanTime` and `timeIncrement` on almost every one.

A range is stored as one uint16 code. The unit is `millimetre` when `rangeMax` is 65 meters or
less, and `centimetre` otherwise; the block always says which. A finite range `r` in meters is
stored as `r` divided by the unit, rounded to the nearest whole number and clipped to 0 to `0xFFFD`.
The three values that are not ranges have codes of their own:

| code | the sensor said |
| -- | -- |
| `0xFFFF` | `+inf`: no return |
| `0xFFFE` | NaN |
| `0x0000` | `-inf` |
| `0x0001` to `0xFFFD` | the range, as `code` x the unit in meters (0.001 or 0.01) |

A finite range above the top code, 65.533 meters in millimeters and 655.33 meters in centimeters,
is clipped to it, and a negative finite range is clipped to 0, so it and a finite range that rounds
to 0 are stored as `0x0000` and read as `-inf`. Every other finite range reads back within half a
unit of the range the sensor published. A scan with more than 65,535 rays cannot
be stored: it is refused and counted in the lidar's `refused`.

The node writes `deltaShuffleZstd`. To read it, in this order:

1. Decompress `ranges` with zstd. The result is 2 x K x `rays` bytes, K being the number of
   `entries`: first the low byte of every value, then the high byte of every value, over the whole
   block in scan order.
2. Unshuffle: value `j` is `low[j] | (high[j] << 8)`, for `j` from 0 to K x `rays` - 1.
3. Undo the delta, scan by scan: in each scan of `rays` values, the code at ray 0 is the value, and
   the code at ray `i` is the code at ray `i - 1` plus the value, both modulo 65,536.

Ray `i` of scan `k` is then at `codes[k * rays + i]`, and the scan's pose, `timeIncrement` and
`scanTime` are in `entries[k]`. Ray `i`'s angle is `angleMin + i * angleIncrement`. With the
`uint16` encoding there is nothing to decompress, unshuffle or undelta: `ranges` is `codes` in
scan-major, little-endian order. `intensities`, when present, is read the same way. Each intensity
is the sensor's value clamped to 0 to 65,535 and rounded to the nearest whole number, with a value
that is not finite stored as 0; there is no sentinel.

### Point cloud objects

A `CloudBlock` holds one `PointCloud2` sensor's clouds, packed into one object that closes at 2 MiB
of data or 5 seconds of stamps, whichever comes first. A cloud larger than 2 MiB is an object of
its own, and its data is at most 8 MiB. The header describes every cloud in the object as written:

| field | type | what |
| -- | -- | -- |
| `position` | text | `int16_mm` or `float32` |
| `color` | text | `none`, `rgb565`, `rgb332` or `rgb888` |
| `intensity` | text | `none` or `uint8` |
| `voxelLeafMm` | UInt16 | The downsampling cube's size in millimeters; 0 when the clouds were not downsampled |
| `encoding` | enum | `planarDeltaShuffleZstd` or `planarZstd` |
| `zstdLevel` | UInt8 | 0 when each cloud's block is stored as it is; above 0, each is zstd-compressed at that level |
| `entries` | list of `CloudEntry` | One per cloud, in time order |
| `data` | bytes | The clouds' blocks, back to back |

A `CloudEntry` is one cloud and the pose it was taken from:

| field | type | what |
| -- | -- | -- |
| `time` | UInt64 | When the cloud was taken, in epoch milliseconds, by the clock rule |
| `seq` | UInt32 | The lidar's cloud counter, from 0 in each session |
| `pose` | 7 x Float32 | `px py pz qx qy qz qw`; see [Reading a sensor pose](#reading-a-sensor-pose) |
| `pointCount` | UInt32 | N, the number of points in this cloud |
| `byteOffset`, `byteLength` | UInt32 | Where this cloud's block lies inside `data` |

Each block decodes on its own. The fields describe the clouds as written. A cloud recorded with
`color: rgb565` from a sensor with no color field is stored with `color` set to `none`, while the
session property `c3d.ros.lidar.<name>.pointcloud.color` records the `rgb565` asked for.

To read a cloud, take `data[byteOffset .. byteOffset + byteLength)` and decompress it when
`zstdLevel` is above 0. With `planarZstd` the result is the planar block, which the next paragraph
reads. With `planarDeltaShuffleZstd`, which the node writes for `int16_mm` positions, the position
planes are first transformed back, one plane at a time in the order X, Y, Z, each N values long:

1. The plane is stored as its N low bytes followed by its N high bytes: value `i` is
   `low[i] | (high[i] << 8)`.
2. The value is the difference from the previous point: the plane's value `i` is the value `i - 1`
   plus it, as int16 with wrap-around, the first against 0.

The color and intensity planes follow the three position planes unchanged. What results is the
planar block, which is read in this order, little-endian, each plane N values long:

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
| camera frame, in a chunk | The sensor's rotation in scene axes, by the Dynamic Object rule | A direction `v` in the frame's own ROS axes points along `rotate(q, M v)` |
| point cloud, in an object | The sensor's rotation in scene axes, by the Dynamic Object rule | A stored point `p` is at `rotate(q, p) + position` |
| laser scan, in a block | The rotation of the scan plane | A ray is at `rotate(q, (r cos t, r sin t, 0)) + position` |

`M` is the session's position map from [Coordinates and units](#coordinates-and-units). A camera on
an optical frame looks along `v = (0, 0, 1)`, and one on a body frame along `v = (1, 0, 0)`. A stored
point cloud point is already `M` applied to the sensor's own point. For a laser scan, ray `i` has
angle `t = angleMin + i * angleIncrement` and range `r`, the decoded range of ray `i` of that scan;
skip a ray whose range is not finite or lies outside `rangeMin` to `rangeMax`. The point cloud and laser scan rules need nothing
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
packed Cap'n Proto messages, each with `formatVersion` `"2.0"`. The schemas are in the repository,
in `core/capnp/camera_payload.capnp`, `core/capnp/lidar_payload.capnp` and
`core/capnp/common_payload.capnp`, which holds the `Pose` all of them use. A camera object is a
`CameraPayload` of `FrameMeta`. A lidar object is a `LidarPayload` holding a union of a `ScanBlock`
of `ScanEntry` or a `CloudBlock` of `CloudEntry`. A `Pose` that was never written reads as the
identity rotation at the origin, because `qw` defaults to 1; a pose the node stores is always a real
tf lookup.
