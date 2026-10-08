# Camera and lidar

Beside the pose timeline, the node can record two streams of sensor data: camera frames, stored as
JPEG, and lidar scans, stored either as a `LaserScan` copied as published or as a `PointCloud2`
that is downsampled and compressed. Every frame and every scan carries the pose of its sensor in
the scene, so Session Replay can show what the robot saw from where it saw it. Both streams are
off by default.

[Data format](data-format.md#camera-frames) describes what is stored.

## Record a camera

Name each camera in `cameras` and give it a topic. The name is a label you choose: it is the camera
id on every stored frame and keys the camera's counters, so a robot whose topic is renamed keeps
its history joined. It must not be empty or contain `.` or `/`.

```yaml
/**:
  ros__parameters:
    cameras: ["front"]
    camera:
      front:
        topic: "/oakd/rgb/preview/image_raw"
        rate_hz: 1.0          # frames recorded per second
        max_width_px: 640     # wider frames are scaled down; 0 never resizes
        jpeg_quality: 80
```

[Configuration](configuration.md) lists every camera parameter with its default and range. The
topic's type is read from the ROS graph, and must be `sensor_msgs/msg/Image` or
`sensor_msgs/msg/CompressedImage`:

- **An `Image`** in `rgb8`, `bgr8`, `rgba8`, `bgra8` or `mono8` is scaled down to `max_width_px`
  when it is wider, with its aspect ratio kept, and encoded as JPEG at `jpeg_quality`. The depth encodings `32FC1`,
  `16UC1` and `mono16` are refused, because a depth image encoded as JPEG is a grey picture: point
  the camera at the color topic instead. Any other encoding is refused too.
- **A `CompressedImage`** that is already JPEG and no wider than `max_width_px` is stored byte for
  byte, with no decoding and no re-encoding. Any other `CompressedImage` is decoded, scaled and
  encoded as JPEG.

The stored width and height are the frame's own, after any scaling. `format` takes only `jpeg`.

### A camera's field of view

Every frame can carry the camera's field of view, as two full angles in degrees, horizontal and
vertical. Session Replay draws the camera's frustum from it. It travels with each frame rather than
once per session, so a camera that zooms can record a different angle on every frame. Scaling a
frame down to `max_width_px` does not change it.

By default the node computes it from the camera's `sensor_msgs/msg/CameraInfo`, on the image
topic's sibling by image_transport's convention: for `/oakd/rgb/image_raw` or
`/oakd/rgb/image_raw/compressed`, that is `/oakd/rgb/camera_info`. Each frame carries the field of
view of the latest CameraInfo, so a driver that republishes it as it zooms changes it frame by
frame. A frame that arrives before the first CameraInfo carries none. It is computed from the calibration matrix `K`, the image size and any region of interest,
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
configure. With neither set and no usable CameraInfo, frames carry no field of view, stored as 0
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
        rate_hz: 2.0
      depth:
        topic: "/oakd/rgb/preview/depth/points"
        rate_hz: 1.0
        pointcloud:
          voxel_leaf_m: 0.05
          position: "int16_mm"
          color: "rgb565"
```

The topic's type must be `sensor_msgs/msg/LaserScan` or `sensor_msgs/msg/PointCloud2`.

**A `LaserScan`** is stored as published: its angles, timing and ranges stay in the sensor's own
units and frame, infinities and NaNs included. Intensities are stored only with
`lidar.<name>.intensities: true`, which doubles the size of a scan. A scan with no ranges is
skipped.

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
- `pointcloud.zstd_level` (default 1) compresses the result; `0` stores it uncompressed.

A cloud the node cannot read, such as a big-endian one or one whose `x`, `y` and `z` are not
`FLOAT32`, is refused and counted in `refused`, with one warning naming the reason.

A lidar entry and an allowlist entry are independent. Listing `/scan` in `topics` records summary
series such as the nearest and mean range ([Recording](recording.md)); a `lidars` entry records the
scans themselves. Image and point-cloud topics belong in `cameras` and `lidars`: the allowlist
refuses those types.

## How often frames are recorded

`rate_hz` is the most records per second a sensor stores: 1.0 for a camera and 2.0 for a lidar
unless you set it. The limit runs on the time each message arrives at the node, never on its header
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

A record whose pose cannot be looked up is dropped and counted in `pose_failures`, because a frame
with no place in the scene cannot be drawn. So is every record while the pose timeline is still
waiting for its preferred reference frame. A sensor whose frame never resolves therefore records
nothing while the run looks configured; `doctor` checks each configured sensor's frame before a
session opens ([doctor and verification](doctor-and-verification.md)).

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

What a stream costs is the size of one record times `rate_hz`. Record size depends on resolution,
settings and what the sensor sees. These sizes were measured in simulation:

| record | stored size |
| -- | -- |
| 320 x 240 camera frame, JPEG quality 80 | about 14.5 KB |
| 640 x 480 camera frame, JPEG quality 80 | about 58 KB, scaled from the above by pixel count |
| 1280 x 720 camera frame, JPEG quality 80 | about 175 KB, scaled the same way |
| 640-ray `LaserScan`, no intensities | 2,560 bytes: 4 bytes per ray, 8 with intensities |
| depth point cloud, 0.05 meter voxels, `int16_mm`, no color | about 3 bytes per stored point |
| the same cloud with `rgb565` color | about 20 percent more |
| the same cloud with `float32` positions | about 3 times as much |

For example, a 640 x 480 camera at 5 frames per second stores about 290 KB per second, or about 1
GB per hour. `doctor` sizes each configured sensor by encoding one real message with the same
encoder the run uses, and warns when all sensors together exceed 2 MiB per second.

> [!WARNING]
> The platform stores each frame or scan as one record of at most about 1 MiB. A larger record is
> accepted with its upload and then dropped, and nothing reports it. `doctor` warns about any
> sensor whose sample encodes to more than 900 KiB.

A camera at full resolution with `max_width_px: 0`, or a point cloud with `float32` positions,
`rgb888` color and no voxel leaf, can exceed it. Lower `max_width_px` or `jpeg_quality` for a
camera; for a point cloud, keep `int16_mm`, drop color, raise `voxel_leaf_m` or lower `max_points`.

### Uplink and disk

Camera and lidar records go to the spool before they upload, and share its `spool_max_mb` budget
with every other stream. When the spool is full, the oldest camera and lidar data is evicted before
any pose, sensor or event data. When the uplink is slower than the data rate, the spool grows by
the difference for as long as the session lasts. [Sessions](sessions.md) describes the spool,
recording with uploads paused, and draining it later over a faster link.

### CPU

Every message published on a configured topic is delivered to the node and deserialized, including
the ones the rate limit drops. A raw high-resolution image topic at 30 Hz is copied into the node
30 times a second. Where your robot publishes a compressed or reduced-resolution topic, record that
one.

For each recorded message, the work is, from cheapest to most expensive:

- a `CompressedImage` already in JPEG and within `max_width_px`: stored as it is
- a `LaserScan`: copied
- an `Image`, or a `CompressedImage` that needs scaling: converted, scaled and JPEG-encoded
- a `PointCloud2`: converted, voxel-downsampled, sorted and compressed

The node handles its callbacks one at a time, so a slow encode delays its other work, including the
next pose sample. Keep `rate_hz`, `max_width_px` and the point cloud's size modest on a robot with a
small CPU.

## What gets refused

Configure refuses, naming the parameter:

- a name in `cameras` or `lidars` that is empty, contains `.` or `/`, or appears twice
- a `camera.<name>.*` or `lidar.<name>.*` key whose name is not in `cameras` or `lidars`, or whose
  field is not one the camera or lidar has, such as `hfov_deg` for `hfov_degrees` or a lidar
  `qos.*` key
- a camera or lidar with no `topic`
- a `rate_hz` of 0 or less
- a camera `format` other than `jpeg`, a `jpeg_quality` outside 1 to 100, or a negative
  `max_width_px`
- a `pointcloud.voxel_leaf_m` below 0 or above 65 meters, a `pointcloud.max_points` below 1, or a
  `pointcloud.zstd_level` outside 0 to 22
- a `pointcloud.position`, `pointcloud.color` or `pointcloud.intensity` that is not one of the values
  above
- a camera `qos` value that is not one of those under [QoS](#qos)
- a `camera_flush_s` or `lidar_flush_s` of 0 or less, or a `camera_flush_bytes` or
  `lidar_flush_bytes` outside 1024 to 33554432 (32 MiB)

Three refusals come later, because they depend on what the robot publishes. Each is logged once per
sensor:

- A topic whose type is not one the sensor takes is refused when the node subscribes, and nothing
  from it is recorded. The node reads the type from the graph when the topic appears, so a camera
  driver that starts after the node is picked up then.
- A frame in a depth or unsupported encoding is refused and counted in `refused`.
- A point cloud the node cannot read is refused and counted in `refused`.

The platform also validates every camera and lidar upload. It rejects a part whose scene id and
scene version it cannot resolve for your application key, and the part is discarded with every
record in it. Check the scene id, the version number and the project as [Scenes](scenes.md)
describes.

## What the session records

Every session with a camera or lidar configured says which sensors ran and how, as session
properties:

| property | value |
| -- | -- |
| `c3d.ros.cameras`, `c3d.ros.lidars` | The configured names, comma-separated |
| `c3d.ros.camera.<name>.topic` | The camera's topic |
| `c3d.ros.camera.<name>.rate_hertz`, `.max_width_pixels`, `.jpeg_quality`, `.format` | The camera's settings in force, defaults included |
| `c3d.ros.camera.<name>.hfov_degrees`, `.vfov_degrees` | The field of view parameters, 0 when unset. Each frame carries its own field of view; see [A camera's field of view](#a-cameras-field-of-view) |
| `c3d.ros.camera.<name>.fov_source` | The source the camera is configured to take its frames' field of view from: `parameter`, `camera_info`, or `none`. `camera_info` does not say that a CameraInfo arrived: a frame before the first one carries no field of view |
| `c3d.ros.camera.<name>.camera_info_topic` | The CameraInfo topic, resolved; empty when switched off |
| `c3d.ros.lidar.<name>.topic`, `.rate_hertz` | The lidar's topic and rate |
| `c3d.ros.lidar.<name>.message_type` | `sensor_msgs/msg/LaserScan` or `sensor_msgs/msg/PointCloud2`, once the topic is subscribed |
| `c3d.ros.lidar.<name>.intensities` | For a `LaserScan`: whether intensities are stored |
| `c3d.ros.lidar.<name>.pointcloud.voxel_leaf_meters`, `.max_points`, `.position`, `.color`, `.intensity`, `.zstd_level` | For a `PointCloud2`: the settings in force |

A lidar whose topic never appears records neither `message_type` nor the settings that depend on it.

Every 5 seconds, the session also records what each sensor did since the session started, as sensor
series:

| series | counts |
| -- | -- |
| `c3d.camera.<name>.frames`, `c3d.lidar.<name>.scans` | Records stored |
| `c3d.camera.<name>.bytes`, `c3d.lidar.<name>.bytes` | Bytes of image or scan data stored |
| `c3d.camera.<name>.pose_failures`, `c3d.lidar.<name>.pose_failures` | Records dropped because their pose could not be looked up |
| `c3d.camera.<name>.refused`, `c3d.lidar.<name>.refused` | Messages the node would not encode or could not read |
| `c3d.lidar.<name>.dropped_oversize` | Clouds dropped whole for exceeding `max_points` |
| `c3d.lidar.<name>.dropped_range` | Points dropped for being out of `int16_mm` range |

Messages the rate limit drops are not counted. A low frame count with `refused` at zero is the rate
limit doing its job; a rising `refused` is a configuration to fix.

These counts reach the session only. The report line and `~/diagnostics` carry nothing for a camera
or lidar; see [The report line](sessions.md#the-report-line).
