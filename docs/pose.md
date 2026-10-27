# Pose stream

**Provisional, and off by default.** The pose stream sends the robot's tf tree to Cognitive3D as a
rig: every frame under the robot's body, moving together, in one stream. It needs a version of the
Cognitive3D platform that accepts the pose stream. An older one answers every pose upload with HTTP
400, and the node discards a refused upload rather than retry it, so check with Cognitive3D before
you turn it on. Its parameters, keys and events may change in a minor release; the
[changelog](../CHANGELOG.md) says how to migrate when they do.

Nothing else changes when you turn it on. Session Replay still draws the robot from the
[pose timeline](coordinates.md#the-pose-timeline), the gaze channel keeps recording as before, and
cameras and lidars keep their own poses. Until a model of the robot is uploaded and named in
`pose.mesh`, the pose stream is data only: a record of where each of the robot's frames was, which
no view draws yet.

## Turn it on

```yaml
/**:
  ros__parameters:
    pose:
      enabled: true
```

Run [`doctor`](doctor-and-verification.md) afterwards. Its `pose stream` check lists the frames the
rig would carry now. Every `pose.*` parameter is in [Configuration](configuration.md#the-pose-stream).

## What is sent

One robot is one rig, and one node records one robot. The rig is described once, by a manifest,
and then sampled `pose.rate_hz` times a second (10 by default):

- **The joints are tf frames.** Joint 0 is `robot_frame`. Every frame whose chain of tf parents
  reaches `robot_frame` is a joint too, named after its frame. `map`, `odom` and anything else
  above the body are not. `pose.exclude_frames` leaves frames out, each with every frame below it:
  the wheels of a wheeled base, for example, whose spinning nobody needs.
- **Joint 0 is where the robot is.** It carries `robot_frame`'s pose in the session's reference
  frame, the frame the pose timeline is in at that moment: `reference_frame`, or
  `fallback_reference_frame` while it does not resolve. Nothing is sent before the pose timeline
  has settled on one. A switch between them is the `robot.reference_frame_changed` event, as for
  the pose timeline.
- **Every other joint is local.** It carries its frame's transform in its tf parent: a position in
  meters and a rotation. No scale.
- **One sample is one instant.** Each sample looks the body up at the latest time tf has, as the
  pose timeline does, and every other frame at the body's own tf stamp. A frame whose broadcaster
  has not published that far yet is taken at its own latest.
- **Times are wall-clock time.** A sample's time is when the node took it, in epoch microseconds,
  never a tf stamp; under `sim_time_on_wire` it follows the session's timeline, as every other
  record does. See [Time](sessions.md#time).
- **Poses are converted like the robot's Dynamic Object.** Every joint, joint 0 included, takes the
  session's `pose_convention`: the position map, and the rotation by the Dynamic Object rule,
  never the gaze rule. Converting each joint on its own and composing them down the tree then gives
  the converted pose of every frame. Under `unity` and `gltf_authored` the position map is a
  reflection, so a joint turning about an axis turns about that axis's mirror image on the wire,
  and Session Replay's own mirror turns it back. See [Coordinates](coordinates.md#how-a-pose-is-converted).

Samples are encoded in blocks of `pose.block_interval_s` seconds and uploaded in the background like
every other stream: written to the spool first, and retried while the network is down.

### The manifest

The manifest says what the rig is made of. It is sent when a session starts and again whenever the
frame set changes. Everything in it comes from tf and from the node's own configuration:

| field | value |
| -- | -- |
| `manifestId` | `pose.manifest_id`, or by default `ros2-` and a hash of `device_id`, so the same robot has the same id in every session |
| `name` | `device_id` |
| `mesh`, `fileType` | `pose.mesh` and `pose.mesh_file_type`, both empty by default: no model |
| `riggingMappingType` | `meshName`: a model's node is matched to a joint by the frame's name |
| `root` | empty |

It carries these keys, the `ros2/1` profile:

| scope | key | value |
| -- | -- | -- |
| rig | `c3d.profile` | `ros2/1` |
| rig | `ros.robotName` | `device_id` |
| joint | `ros.parent` | the frame's tf parent; on every joint but joint 0 |
| joint | `c3d.kind` | `camera`, `lidar`, or a described sensor's `kind`, on a frame named by `camera.<name>.frame`, `lidar.<name>.frame` or `sensor.<name>.frame` |

A frame two sensors name keeps the first kind, in the order cameras, lidars, described sensors. A
camera or lidar whose `frame` is left empty takes its frame from each message, and its frame carries
no `c3d.kind`; set the parameter to tag it.

The node does not read the robot's URDF. A later release may add keys from `/robot_description`
when the robot publishes one, beside these.

### An example

A small robot with no `/robot_description`, whose tf tree is `map -> odom -> base_link`, with
`laser` and `camera_link` under `base_link` and `camera_optical_frame` under `camera_link`. The
camera is on a gimbal, so `camera_link` moves. Configured with `device_id: "raspbot-01"`, a lidar
whose `frame` is `laser` and a camera whose `frame` is `camera_link`, it sends:

```text
rig:   manifestId ros2-64a722d4f3a20f48, name raspbot-01, mesh "", fileType ""
       c3d.profile ros2/1, ros.robotName raspbot-01
j=0    base_link
j=1    camera_link           ros.parent base_link     c3d.kind camera
j=2    camera_optical_frame  ros.parent camera_link
j=3    laser                 ros.parent base_link     c3d.kind lidar
```

Joints are ordered parents first, and siblings by name. Each sample carries `base_link` in `map`,
and the other three in their parents: the gimbal's movement is `camera_link`'s rotation.

## When a frame comes, goes or fails

tf does not say when a frame stops existing, so the node decides from what it can look up:

- **A frame joins** at the first tick it can be looked up, with the manifest sent again. A frame
  that was never seen has no value to send.
- **A frame that cannot be looked up is held.** Its last value is sent again, and the session counts
  each such value in the `c3d.stream.pose.heldFrames` property. The node logs it once per frame.
  A frame whose transform has stopped advancing for 10 samples, and at least 5 seconds, counts as
  not looked up: tf keeps answering a stopped broadcaster's last transform for ever.
- **A frame gone too long leaves.** A frame not looked up for longer than tf's cache window, 10
  seconds, leaves the rig with every frame below it, the manifest is sent again without them, and
  the session records a `robot.frame_lost` event, with `frame`, its `parent`, and
  `last_seen_epoch_seconds`: the held values after that time are not real. If it comes back, it
  joins again.
- **A frame whose tf parent changes** is sent again under its new parent.
- **When the body cannot be looked up** for a second, the rig is marked hidden, with its last values,
  so a replay does not draw it standing where it was last seen. The next sample with a body shows
  it again.

## What the session records

| record | name | what |
| -- | -- | -- |
| session property | `c3d.ros.pose.rate_hertz` | `pose.rate_hz` |
| session property | `c3d.ros.pose.frame_count` | how many joints the last manifest sent had |
| session property | `c3d.stream.pose.blockIntervalMs` | `pose.block_interval_s`, in whole milliseconds |
| session property | `c3d.stream.pose.heldFrames` | values held for a frame that could not be looked up, at the session's end |
| session property | `c3d.stream.pose.repairedRotations` | rotations the encoder could not use and replaced, at the session's end; 0 for any rotation tf produces |
| series | `c3d.pose.samples`, `c3d.pose.held_frames` | the same counts while the session runs, every 5 seconds |
| event | `robot.frame_lost` | a frame left the rig; see above |

None of these, and no pose upload, exist in a session with `pose.enabled: false`.
[Data format](data-format.md#the-pose-stream) describes the stream as stored.

## What it costs

At the defaults, a rig of 10 frames at 10 samples a second is a few kilobytes per second before
compression, and much less after it: a frame that does not move costs almost nothing once its block
has started. A sample rate above the rate tf is published at sends the same values more than once.
