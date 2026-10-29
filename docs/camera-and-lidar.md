# Camera and lidar

Beside the pose timeline, the node can record two streams of sensor data: camera frames, encoded
as H.264 in chunks of 3 seconds, and lidar scans, stored either as a `LaserScan` in blocks of 3
seconds or as a `PointCloud2` that is downsampled, delta coded and packed into objects. Every frame
and every scan carries the pose of its sensor in the scene, so Session Replay can show what the
robot saw from where it saw it. Both streams are off by default.

[Data format](data-format.md#camera-chunks) describes what is stored.

## Record a camera

Name each camera in `cameras` and give it a topic. The name is a label you choose: it is the camera
id on every stored chunk and keys the camera's counters, so a robot whose topic is renamed keeps
its history joined. It must not be empty or contain `.` or `/`.

```yaml
/**:
  ros__parameters:
    cameras: ["front"]
    camera:
      front:
        topic: "/oakd/rgb/preview/image_raw"
        rate_hz: 30.0         # frames recorded per second
        max_width_px: 640     # wider frames are scaled down; 0 never resizes
        preset: "ultrafast"   # or "veryfast": more CPU for fewer bytes
        crf: 28               # 24 to 32; lower is higher quality and more bytes
```

[Configuration](configuration.md) lists every camera parameter with its default and range. The
topic's type is read from the ROS graph, and must be `sensor_msgs/msg/Image` or
`sensor_msgs/msg/CompressedImage`:

- **An `Image`** in `rgb8`, `bgr8`, `rgba8`, `bgra8` or `mono8` is scaled down to `max_width_px`
  when it is wider, with its aspect ratio kept, and fed to the H.264 encoder. The depth encodings
  `32FC1`, `16UC1` and `mono16` are refused, because a depth image encoded as video is a grey
  picture: point the camera at the color topic instead. Any other encoding is refused too.
- **A `CompressedImage`** is decoded, scaled to `max_width_px` when it is wider, and fed to the
  encoder like an `Image`. Nothing is stored as the camera published it, whatever its format.

The encoder needs even dimensions, so a frame with an odd width or height loses its last column or
row. The stored width and height are the frame's own, after any scaling and cropping.

### How a camera is encoded

Frames are encoded as H.264 by the host's `ffmpeg`, which the node runs as a child process, using
x264. The node never links the encoder. `ffmpeg` is a package the node's deb depends on, and the
source build installs it by name: see [Install](install.md). `ffmpeg_path` names another binary
when `ffmpeg` is not on the node's `PATH`; it defaults to `ffmpeg`. With a camera configured,
configure refuses when that binary cannot run, naming the package to install, and `doctor`'s
`camera encoder` check says the same before a session opens.

A chunk is 3 seconds of real time, one closed group of pictures, whatever the camera delivers, and
every chunk starts on a keyframe. Under `sim_time_on_wire`, 3 seconds of real time is 3 x the
real-time factor seconds on the wire. `rate_hz` caps the frames fed to the encoder, as it caps
every sensor's records: it does not set a chunk's length, so a camera that delivers 15 frames a
second with `rate_hz: 30` makes chunks of about 45 frames. A camera slower than one frame every 3
seconds gets one chunk per frame. A camera whose field of view changes records the angle of each
chunk's first frame. The encoder times a chunk's 3 seconds by the wall clock, so when the wall
clock steps by more than a second while a camera records, as when NTP corrects a stale clock, the
node restarts that camera's encoder, which starts a new chunk, and logs it once per session.

A chunk also ends before its encoded frames would pass 8 MiB, because the platform refuses an
object larger than 10 MiB, and an object it refuses is deleted. The node then restarts the
camera's encoder and encodes again, from a keyframe that opens the next chunk, every frame from the
one that did not fit on, so no frame is lost and each keeps its `time`, `seq` and pose. Each such
restart counts in `size_restarts` and is logged once per session. To do this the node keeps every
frame it feeds the encoder until that frame is in a chunk, as yuv420p at 1.5 bytes a pixel, in up
to `replay_max_mb` of memory per camera, 192 MiB by default: a frame that finds that memory full is
not kept, and is dropped and counted in `dropped_replay` if it has to be encoded again. A chunk's
first frame, a keyframe, is never encoded again: up to 9 MiB it is kept as a chunk of its own, and
a larger one is dropped and counted in `c3d.camera_access_units_dropped`.

Three parameters set the encoder's trade of bytes against quality and CPU:

| parameter | default | what |
| -- | -- | -- |
| `preset` | `ultrafast` | x264's speed preset: `ultrafast` or `veryfast`. `veryfast` spends more CPU for fewer bytes, at somewhat lower quality at the same `crf` |
| `crf` | `28` | The constant rate factor, 24 to 32. It sets the quality the encoder holds, and the bytes follow the scene: lower is higher quality and more bytes, and the size roughly doubles every 4 steps lower. Measured at the default 3-second chunks: `24` is about 1.6 to 2.2 times the bytes of `28` depending on the footage, and `32` about 0.55 to 0.6 times |
| `max_bitrate_kbps` | `0` | A cap on the stream's bitrate in kilobits per second, held over a 2-second window by lowering quality when a scene needs more. `0` sets no cap |

Measured on a 20-second clip of a robot's 640 x 480 camera at 15 frames a second, `ultrafast`,
encoded in 2-second chunks, against the uncompressed frames. The table shows quality, not bytes: VMAF is a video quality score out of 100, and the keypoint columns are the
share of the source's keypoints, up to 1,000 per frame, that the video still detects and still
matches by descriptor, a measure of what a vision model can still find:

| `crf` | VMAF mean | ORB keypoints detected / matched | SIFT keypoints detected / matched |
| -- | -- | -- | -- |
| 24 | 95.7 | 97% / 90% | 80% / 76% |
| 28 | 93.8 | 96% / 87% | 74% / 70% |
| 32 | 90.2 | 95% / 83% | 69% / 63% |

About 94 at `crf: 28` holds for the robot and for an indoor sequence from a Kinect on a wheeled
robot, where `crf: 24` reaches 98 and `crf: 32` 86. On an outdoor street sequence VMAF saturates:
99.8 at `crf: 28`, and only the keypoint columns separate `crf: 24` from `crf: 28` there (ORB
matched 88% against 84%). Keep `crf: 28` unless you know what reads the video. `veryfast` is the
opt-in for fewer bytes at the same VMAF: `veryfast` at `crf: 24` matched `ultrafast` at `crf: 28`
(93.9 against 93.8 on the robot footage) for about 14 percent fewer bytes, and costs about three
times the CPU, 19 percent of a Pi 5 core against 6 to 7 percent at 640 x 480 and 15 frames a
second.

A frame held for its transform, as [Poses and frames](#poses-and-frames) describes, is held as a raw
image, so one camera holds at most 8 x 3 x width x height bytes while it waits for a pose: 7.4 MB
at 640 x 480.

### A camera's field of view

Every chunk can carry the camera's field of view, as two full angles in degrees, horizontal and
vertical. Session Replay draws the camera's frustum from it. It travels with each chunk rather than
once per session, so a camera that zooms can record a different angle in each chunk. Scaling a
frame down to `max_width_px` does not change it.

By default the node computes it from the camera's `sensor_msgs/msg/CameraInfo`, on the image
topic's sibling by image_transport's convention: for `/oakd/rgb/image_raw` or
`/oakd/rgb/image_raw/compressed`, that is `/oakd/rgb/camera_info`. Each chunk carries the field of
view of the latest CameraInfo when its first frame arrived, so a driver that republishes it as it
zooms changes it chunk by chunk. A chunk whose first frame arrives before the first CameraInfo carries none. It is computed from the calibration matrix `K`, the image size and any region of interest,
and binning changes no angle:

```text
hfov = atan(cx / fx) + atan((width - cx) / fx)
vfov = atan(cy / fy) + atan((height - cy) / fy)
```

This is the pinhole model's field of view, so a wide lens with strong barrel distortion sees
further at its edges than it says. A driver with no calibration publishes `K` as zeros, which
gives no field of view; the node says so once, naming the two parameters below. Set
`camera_info_topic` to read another topic, or to `""` to read none.

To set it yourself, from the lens datasheet or a measurement, use `hfov_degrees` and
`vfov_degrees`. They win over the CameraInfo, which is then not subscribed:

```yaml
    camera:
      front:
        topic: "/image_raw/compressed"
        hfov_degrees: 61.0
        vfov_degrees: 45.0
```

Set both or neither. Each must be greater than 0 and less than 180; anything else refuses
configure. With neither set and no usable CameraInfo, chunks carry no field of view, stored as 0
by 0, and Session Replay draws no frustum for the camera. `doctor`'s `camera/<name> field of view`
check says which source a camera has. During a run, the node warns once per session when a
camera's first frame finds its CameraInfo topic with no publisher, naming the topic and the two
parameters.

## Record a lidar

Name each lidar in `lidars` and give it a topic, with the same naming rule as a camera:

```yaml
/**:
  ros__parameters:
    lidars: ["scan", "depth"]
    lidar:
      scan:
        topic: "/scan"
        rate_hz: 30.0
      depth:
        topic: "/oakd/rgb/preview/depth/points"
        rate_hz: 1.0
        pointcloud:
          voxel_leaf_m: 0.05
          position: "int16_mm"
          color: "rgb565"
```

The topic's type must be `sensor_msgs/msg/LaserScan` or `sensor_msgs/msg/PointCloud2`.

**A `LaserScan`** is stored in blocks of 3 seconds of scans. Each range becomes a 16-bit code, in
millimeters when the sensor's `range_max` is 65 meters or less and in centimeters otherwise, with
codes of their own for `+inf`, `-inf` and NaN, so a sensor's no-return survives. A block's header
holds the geometry every scan in it shares, and every scan keeps its own `time_increment` and
`scan_time` on its entry, because drivers such as RPLIDAR's change them on every sweep. The block is
then delta coded within each scan, byte shuffled and compressed with zstd. A scan with different
geometry or a different number of rays starts a new block. Intensities are stored only with
`lidar.<name>.intensities: true`, as 16-bit values in the same layout, which doubles the size of a
scan. A scan with no ranges is skipped. One with more than 65,535 rays is refused and counted in the
lidar's `refused`.

**A `PointCloud2`** is read, converted to the scene's axes and encoded. The node reads clouds that
are little-endian, with `x`, `y` and `z` as `FLOAT32`; color from an `rgb` or `rgba` field
(`FLOAT32` or `UINT32`, packed `0x00RRGGBB`) or from `r`, `g` and `b` (`UINT8`); and intensity from
an `intensity` field (`FLOAT32`, `UINT8` or `UINT16`). Then:

- Points with a non-finite coordinate are dropped.
- With `pointcloud.voxel_leaf_m` above 0 (default 0.05 meters), the points in each cube of that
  size are averaged into one. `0` keeps every point.
- A cloud with more than `pointcloud.max_points` points after downsampling (default 100000) is
  dropped whole and counted in `dropped_oversize`. It is never cut short.
- `pointcloud.position` is `int16_mm` (the default) or `float32`. `int16_mm` stores millimeters and
  reaches 32.767 meters from the sensor on each axis; a point further out is dropped and counted in
  `dropped_range`. For a long-range outdoor lidar, use `float32`, which costs about three times as
  much.
- `pointcloud.color` is `none` (the default), `rgb565`, `rgb332` or `rgb888`. A cloud with no color
  field is stored without color whatever this says, and the node warns once per session, naming the
  lidar and the color it asked for. `c3d.ros.lidar.<name>.pointcloud.color` still records the
  request.
- `pointcloud.intensity` is `none` (the default) or `uint8`. A `FLOAT32` intensity whose values are
  all 1.0 or less is scaled to 0 to 255; any other intensity is clamped to 0 to 255, so a 16-bit
  or wide-range intensity saturates at 255.
- `pointcloud.zstd_level` (default 3) compresses the result; `0` stores it uncompressed.

With `int16_mm` positions and a `zstd_level` above 0, each position plane is delta coded and byte
shuffled before zstd, which leaves about two thirds of the bytes of the plain planar block. Clouds are packed into objects:
an object closes at 2 MiB or 5 seconds of clouds, whichever comes first, and a cloud larger than
2 MiB is an object of its own. `float32` positions are compressed without the delta.

A cloud the node cannot read, such as a big-endian one or one whose `x`, `y` and `z` are not
`FLOAT32`, is refused and counted in `refused`, with one warning naming the reason.

A lidar entry and an allowlist entry are independent. Listing `/scan` in `topics` records summary
series such as the nearest and mean range ([Recording](recording.md)); a `lidars` entry records the
scans themselves. Image and point-cloud topics belong in `cameras` and `lidars`: the allowlist
refuses those types.

## How often frames are recorded

`rate_hz` is the most records per second a sensor stores: 30 for a camera or a lidar unless you
set it. The limit runs on the time each message arrives at the node, never on its header
stamp. A source publishing at exactly `rate_hz` is recorded in full, a faster one is held to
`rate_hz`, and a pause in the source does not let a burst through afterwards. A driver that leaves
stamps at zero, two publishers on one topic, or a simulator reset neither stops nor opens the
limit.

Under `sim_time_on_wire`, the limit runs on the session's simulated clock, so `rate_hz` counts per
simulated second; see [Sessions](sessions.md).

A message the limit drops costs a deserialization, not an encode. Nothing else limits these streams
on the way to the platform: the configuration is the limit, so size it as described in
[What it costs](#what-it-costs).

## Poses and frames

Every frame and scan carries the pose of a tf frame in the session's reference frame, at the
message's header stamp, with `pose_convention` applied. It is the same reference frame the pose
timeline uses, including its fallback; [Coordinates](coordinates.md) describes both. The tf frame is
`camera.<name>.frame` or `lidar.<name>.frame` when set, and otherwise the message's
`header.frame_id`.

A record whose transform has not arrived yet, as with a driver that stamps its frames as it
publishes them, is held for up to 0.2 seconds and posed at its own stamp when the transform arrives.
The 0.2 seconds are the node's ROS time, the clock tf runs on, so under `use_sim_time` they are
simulated seconds, however slowly the simulation runs. A record still without a pose then is
dropped and counted in `pose_failures`, because a frame with no place in the scene cannot be drawn.
So is every record while the pose timeline is still waiting for its preferred reference frame. A
sensor whose frame never resolves therefore records nothing while the run looks configured;
`doctor` checks each configured sensor's frame before a session opens
([doctor and verification](doctor-and-verification.md)).

A message whose header stamp is zero is placed at the time it arrived and posed at the latest
transform, and the node warns once. A message with an empty `header.frame_id` and no configured
frame is not recorded, and the node warns once.

### A point cloud whose header names the wrong frame

> [!WARNING]
> A `PointCloud2`'s header can name a frame its points are not in. The node poses the cloud against
> the named frame, and the stored cloud lies on its side. Every point is present and nothing
> reports it.

Gazebo's `rgbd_camera` sensor does this: it writes its optical frame into the cloud's header and
leaves the points in the frame the sensor is attached to. Set `lidar.<name>.frame` to the link the
sensor is attached to:

```yaml
lidar:
  depth:
    topic: "/oakd/rgb/preview/depth/points"
    frame: "oakd_rgb_camera_frame"    # the header says oakd_rgb_camera_optical_frame
```

`doctor` samples one cloud from each `PointCloud2` lidar whose frame ends in `_optical_frame` or
`_optical`, and warns when its points have depth along +X, as a body frame does, instead of +Z, as
an optical frame does. A real depth camera that publishes its points in its optical frame needs no
override, so keep this one in the simulation's parameter file.

### A laser scan's pose is the scan plane's

A camera frame and a point cloud carry the sensor's own rotation, carried into the scene's axes. A
`LaserScan` is different: its rays are stored unconverted, so its pose carries the rotation of the
scan plane instead. A `LaserScan` and a camera on the same tf frame at the same instant therefore
have the same position and, under `unity` and `gltf_authored`, different rotations. Under `rep103`
the two rotations are the same.

| record | its rotation is | so a stored point lands at |
| -- | -- | -- |
| camera frame | the sensor's rotation in scene axes | (a frame stores no points) |
| point cloud | the sensor's rotation in scene axes | `rotate(q, p) + position`, where `p` is a stored point, already in scene axes |
| `LaserScan` | the scan plane's rotation | `rotate(q, (r cos t, r sin t, 0)) + position`, where `r` is a range and `t` its angle |

A reader that applies the point cloud rule to a scan draws the scan turned out of place.
[Data format](data-format.md#reading-a-sensor-pose) gives both rules in full.

## QoS

When the node subscribes to a camera or lidar topic, it reads the publishers' QoS and requests a
profile they can all serve: reliable only if every publisher offers reliable, transient-local only
if every publisher offers it, and a depth of at least 10. A topic with no publisher yet is requested
best-effort and volatile, which connects to any publisher.

A camera accepts an override, under `camera.<name>.qos`: `reliability` (`reliable` or
`best_effort`), `durability` (`volatile` or `transient_local`), `history` (`keep_last` or
`keep_all`) and `depth` (1 or more, or `-1` to follow the publishers). Any other value refuses
configure, naming the parameter; a `depth` of 0 is refused too. A lidar takes no override.

When a publisher and the node's subscription are incompatible, no data arrives from that publisher,
and the node logs an error naming the policy and, for a camera, the override that matches the
publisher:

```text
camera:front: QoS incompatibility on /oakd/rgb/preview/image_raw: 3 so far, last on the RELIABILITY_QOS_POLICY. No data arrives from an incompatible publisher until the profiles match; a compatible publisher on the same topic still delivers. Set camera.front.qos.reliability: best_effort to match a best-effort publisher.
```

For a lidar, the error asks you to compare the publisher's profile, from
`ros2 topic info --verbose <topic>`, with the node's.

## What it costs

### Bandwidth

What a stream costs is the bytes of what it stores each second, and that depends on resolution,
settings and what the sensor sees. These sizes were measured on recorded footage from a robot, a
Kinect on a wheeled robot and a car, and on lidar recordings from an RPLIDAR, a Velodyne and a
simulated TurtleBot 4 in Gazebo:

| record | stored size |
| -- | -- |
| camera chunk, 3 seconds at 640 wide, `ultrafast`, `crf: 28` | 225 to 280 KB median on every source measured; at most 319 KB at 15 frames a second on a robot, 437 KB at 10 on a car, 645 KB at 22 on the wheeled robot's Kinect |
| a camera over 10 minutes, the same settings | 44 MB for a robot driving at 15 frames a second, 68 MB for the indoor footage at 22 |
| `LaserScan` block, 3 seconds | 18 KB median for 720 rays at 10 Hz; 36 KB median, 45 KB at most, for 640 rays at 30 Hz; 57 to 59 KB when the ranges carry 2 centimeters of noise |
| depth point cloud, 0.05 meter voxels, `int16_mm`, no color | 60 KB median and 90 KB at most per cloud for the simulated TurtleBot 4; 270 KB median per cloud for a Velodyne scan of a street |
| point cloud object | closes at 2 MiB: about 1 second of the TurtleBot 4's clouds, 30 clouds at 30 Hz; 7 of the Velodyne's. An object of slow or sparse clouds reaches 5 seconds at 1.5 to 1.8 MB |
| the same cloud with `rgb565` color | about 20 percent more, measured on the plain `int16_mm` block |
| the same cloud with `float32` positions | about 3 times as much, measured on the plain `int16_mm` block, and `float32` is not delta coded |

A camera at 640 x 480 therefore stores about 260 to 410 MB per hour at 15 to 22 frames a second, and
more at 30. The same footage as JPEG at quality 80 is 271 to 329 MB per 10 minutes for the robot and
421 MB for the indoor sequence, so a chunk is about 13 to 16 percent of the JPEG bytes. A `LaserScan` block is a sixth to a quarter of what its scans stored one by one. A
point cloud takes about two thirds of the bytes it took without delta coding, for up to 6 percent
more CPU.

`doctor` sizes each configured camera from an estimate, from its resolution, `crf` and `rate_hz`,
and each lidar by encoding one real message with the encoder the run uses and multiplying by
`rate_hz`; both overstate a source slower than `rate_hz`. It warns when all sensors together
exceed 2 MiB per second, as a depth point cloud at 30 Hz can. To record less, lower `rate_hz`,
`max_width_px` or raise `crf`, or raise a point cloud's `voxel_leaf_m`.

### Uplink and disk

Camera and lidar objects go to the spool before they upload, and share its `spool_max_mb` budget
with every other stream. When the spool is full, the oldest camera and lidar data is evicted before
any pose, sensor or event data. When the uplink is slower than the data rate, the spool grows by
the difference for as long as the session lasts. On disk an object takes about a third more than it
uploads, so the default 1 GiB holds about two hours of the camera above with no uplink at all.
[Sessions](sessions.md) describes the spool, recording with uploads paused, and draining it later
over a faster link.

### CPU

Every message published on a configured topic is delivered to the node and deserialized, including
the ones the rate limit drops. A raw high-resolution image topic at 30 Hz is copied into the node
30 times a second. Where your robot publishes a compressed or reduced-resolution topic, record that
one.

The encoder runs in its own process, so a camera's encoding does not delay the node's other work.
For each recorded message, the node's own work is, from cheapest to most expensive:

- a `LaserScan`: held as published until its block closes, then converted to codes, delta coded
  and compressed on the node's worker thread, a few microseconds per scan
- an `Image`, or a `CompressedImage` of any format: converted, scaled and written to the encoder's
  input
- a `PointCloud2`: converted, voxel-downsampled, sorted and compressed

On a Raspberry Pi 5, the `ffmpeg` process alone costs 6 to 7 percent of one core at `ultrafast` and
`crf: 28` for a 640 x 480 camera at 15 frames a second, and 19 percent at `veryfast` and
`crf: 24`. That was measured with the robot's own software running and the SDK not, so the node's
own conversion, scaling and writes to the encoder are not in it.

The node handles its callbacks one at a time, so a slow conversion delays its other work, including
the next pose sample. At the default 30 records a second, a `PointCloud2` is a heavy load for a
small CPU: record a reduced-resolution topic there, or lower the point cloud's size or `rate_hz`.

A record held for its transform, as [Poses and frames](#poses-and-frames) describes, costs the node
no time: the node goes on with its other work and retries the held records every 10 milliseconds,
and whenever the sensor's next record arrives. Once a sensor has 8 records waiting, the oldest is
dropped and counted in `pose_failures`. Records still held when a session ends are discarded and
not counted.

What the encoder costs the node, and what it loses, is bounded:

- **A stop or a restart blocks the node for at most 1.25 seconds.** That is the time the node
  waits for the encoder to finish what it has accepted, then to be killed and collected. A healthy
  encoder finishes in milliseconds. Stopping several cameras waits once, not once per camera.
- **A wedged encoder is killed without waiting** when the camera is fast enough to show that it has
  refused every frame for 3 seconds, and the node starts a new one 5 seconds later. On a camera too
  slow to show the refusals, the wedge is found as a quiet gap and costs 1.25 seconds once and then
  0.5 seconds every 5 seconds, for as long as the cause lasts.
- **The measured envelope is `ultrafast` and `veryfast` at 640 wide on a Pi 5.** A configuration
  that saturates the encoder, such as `veryfast` at 1080p there, drops frames while it runs, counted
  in `dropped_backlog`, and at a stop it may lose up to about 18 frames the encoder had accepted,
  counted in `dropped_at_stop` and said once in the log.
- **A camera that stops publishing keeps up to 3 seconds of frames in memory**, unspooled, until
  its next keyframe or the end of the session. A crash or SIGKILL in that time loses them.

A camera that delivers fewer frames than `rate_hz` makes a chunk of fewer frames, not a chunk
shorter than 3 seconds.

## What gets refused

Configure refuses, naming the parameter:

- a name in `cameras` or `lidars` that is empty, contains `.` or `/`, or appears twice
- a `camera.<name>.*` or `lidar.<name>.*` key whose name is not in `cameras` or `lidars`, or whose
  field is not one the camera or lidar has, such as `hfov_deg` for `hfov_degrees` or a lidar
  `qos.*` key
- a camera or lidar with no `topic`
- a `rate_hz` of 0 or less
- a negative `camera.<name>.max_width_px`, a `preset` other than `ultrafast` or `veryfast`, a `crf`
  outside 24 to 32, or a `max_bitrate_kbps` that is negative or above 1,000,000
- a `camera.<name>.format` or `camera.<name>.jpeg_quality`, which a camera no longer has
- a camera configured while the binary named by `ffmpeg_path` cannot run, or lists no `libx264`
  encoder: the refusal names the binary and says `sudo apt install ffmpeg`
- a `pointcloud.voxel_leaf_m` below 0 or above 65 meters, a `pointcloud.max_points` below 1, or a
  `pointcloud.zstd_level` outside 0 to 22
- a `pointcloud.position`, `pointcloud.color` or `pointcloud.intensity` that is not one of the values
  above
- a camera `qos` value that is not one of those under [QoS](#qos)

`camera_flush_s`, `camera_flush_bytes`, `lidar_flush_s` and `lidar_flush_bytes` no longer exist: a
chunk, a block or an object closes by its own rule, and a params file that sets them has no effect.

Four refusals come later, because they depend on what the robot publishes. Each is logged once per
sensor:

- A topic whose type is not one the sensor takes is refused when the node subscribes, and nothing
  from it is recorded. The node reads the type from the graph when the topic appears, so a camera
  driver that starts after the node is picked up then.
- A frame in a depth or unsupported encoding is refused and counted in `refused`.
- A point cloud the node cannot read is refused and counted in `refused`.
- A `LaserScan` with more than 65,535 rays is refused and counted in `refused`, because a block
  stores the ray count in 16 bits.

The platform also validates every camera and lidar upload. It rejects an object whose scene id and
scene version it cannot resolve for your application key, and the object is discarded with every
record in it. Check the scene id, the version number and the project as [Scenes](scenes.md)
describes.

## What the session records

Every session with a camera or lidar configured says which sensors ran and how, as session
properties:

| property | value |
| -- | -- |
| `c3d.ros.cameras`, `c3d.ros.lidars` | The configured names, comma-separated |
| `c3d.ros.camera.<name>.topic` | The camera's topic |
| `c3d.ros.camera.<name>.rate_hertz`, `.max_width_pixels` | The camera's settings in force, defaults included |
| `c3d.ros.camera.<name>.codec`, `.preset`, `.crf`, `.max_bitrate_kilobits_per_second`, `.chunk_seconds` | How the camera is encoded: `h264`, the preset and `crf` in force, the bitrate cap with 0 for none, and the chunk length, `3` |
| `c3d.ros.camera.<name>.hfov_degrees`, `.vfov_degrees` | The field of view parameters, 0 when unset. Each chunk carries its own field of view; see [A camera's field of view](#a-cameras-field-of-view) |
| `c3d.ros.camera.<name>.fov_source` | The source the camera is configured to take its chunks' field of view from: `parameter`, `camera_info`, or `none`. `camera_info` does not say that a CameraInfo arrived: a chunk whose first frame precedes the first one carries none |
| `c3d.ros.camera.<name>.camera_info_topic` | The CameraInfo topic, resolved; empty when switched off |
| `c3d.ros.lidar.<name>.topic`, `.rate_hertz` | The lidar's topic and rate |
| `c3d.ros.lidar.<name>.message_type` | `sensor_msgs/msg/LaserScan` or `sensor_msgs/msg/PointCloud2`, once the topic is subscribed |
| `c3d.ros.lidar.<name>.intensities` | For a `LaserScan`: whether intensities are stored |
| `c3d.ros.lidar.<name>.pointcloud.voxel_leaf_meters`, `.max_points`, `.position`, `.color`, `.intensity`, `.zstd_level` | For a `PointCloud2`: the settings in force |
| `c3d.ros.lidar.<name>.pointcloud.encoding` | For a `PointCloud2`: how its position planes are coded, `planarDeltaShuffleZstd` or `planarZstd` |

A lidar whose topic never appears records neither `message_type` nor the settings that depend on it.

Every 5 seconds, the session also records what each sensor did since the session started, as sensor
series, except where a row says otherwise:

| series | counts |
| -- | -- |
| `c3d.camera.<name>.frames`, `c3d.lidar.<name>.scans` | Frames, and scans or clouds, stored |
| `c3d.camera.<name>.bytes`, `c3d.lidar.<name>.bytes` | Bytes of encoded video, scan or cloud data stored |
| `c3d.camera.<name>.pose_failures`, `c3d.lidar.<name>.pose_failures` | Records dropped because their pose could not be looked up |
| `c3d.camera.<name>.refused`, `c3d.lidar.<name>.refused` | Messages the node would not encode or could not read |
| `c3d.camera.<name>.dropped_backlog` | Frames the encoder could not take: its queue was full, no encoder could start, or the frame came during the 5 seconds after an encoder crashed or was killed, when none is started |
| `c3d.camera.<name>.dropped_at_stop` | Frames the encoder had accepted and could not finish when it was stopped. Counted since the node was configured, not since the session started, because the stop that ends a session comes after that session's last sample |
| `c3d.camera.<name>.dropped_no_session` | Encoded frames that arrived when no session was open. Counted since the node was configured, for the same reason. It reads 0 when the node stops the encoder before it ends a session, as it does on every path |
| `c3d.camera.<name>.encoder_restarts` | Encoders replaced after one died, or after a wedged one was killed |
| `c3d.camera.<name>.gap_restarts` | Encoders replaced after the camera was quiet for 3 seconds or more. A time-lapse camera, slower than a frame every 3 seconds, adds one for every frame, and that means nothing is wrong |
| `c3d.camera.<name>.size_restarts` | Encoders replaced because a chunk reached 8 MiB before its time was up; see [How a camera is encoded](#how-a-camera-is-encoded) |
| `c3d.camera.<name>.dropped_replay` | Frames a size restart could not encode again, because `replay_max_mb` had no room for them or the replacement encoder could not start |
| `c3d.lidar.<name>.dropped_oversize` | Clouds dropped whole for exceeding `max_points` |
| `c3d.lidar.<name>.dropped_range` | Points dropped for being out of `int16_mm` range |

Messages the rate limit drops are not counted. A low frame count with `refused` at zero is the rate
limit doing its job; a rising `refused` is a configuration to fix. A restart after a change of image
size or of the camera's tf frame, after a wall-clock step, at a session's start or at a stop counts in
none of `encoder_restarts`, `gap_restarts` and `size_restarts`.

These counts reach the session only. The report line and `~/diagnostics` carry nothing for a camera
or lidar; see [The report line](sessions.md#the-report-line).
