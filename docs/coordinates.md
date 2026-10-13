# Coordinates

The node turns the robot's tf tree into the pose timeline Session Replay draws. This page covers the
settings that decide where that timeline lands in your scene: the pose convention, the reference
frame, and the frame the gaze channel reports. The exact conversions are specified in
[Data format](data-format.md).

## The pose timeline

At `sample_rate_hz` (30 by default) the node looks up `reference_frame` to `robot_frame` in tf,
`map` to `base_link` by default, falling back to `odom`. It converts the pose into the scene's axes
under `pose_convention` and records its position and rotation. This is the gaze channel, the spatial timeline Session Replay draws the robot
from. Every event is placed at the robot body's position from the latest sample.

Without a pose the session has no spatial timeline. A sample that finds no transform is counted in
`c3d.pose_failures`, and the node warns every 10 seconds. doctor's `robot pose` check says which
frames resolve before you launch.

The robot can also be drawn as a Dynamic Object with a mesh of its own, `robot_dynamic_object`;
see [Scenes](scenes.md).

## Choosing a pose convention

`pose_convention` has no default, and configure refuses a node without it. It says how your scene
was authored, so the node can put ROS poses (REP-103: X forward, Y left, Z up) into the same axes.

| value | use it when |
| -- | -- |
| `gltf_authored` | The scene was exported to glTF by a ROS or Gazebo exporter that maps `gltf.xyz = (ros.x, ros.z, -ros.y)`, including the Gazebo export described in [Scenes](scenes.md). |
| `unity` | The scene was authored in Unity, against the ROS-TCP-Connector axes: Unity `(x, y, z)` is ROS `(-y, z, x)`. |
| `rep103` | Debugging only. Poses go out unconverted, as `ros2 topic echo` shows them, and Session Replay draws them on their side. |

> [!WARNING]
> A wrong choice is invisible in the payload. `gltf_authored` and `unity` differ by a 90 degree
> turn about the vertical, so the wrong one swings the whole trajectory off the floor plan and, when
> the robot is drawn as a Dynamic Object, renders its turns as rolls. The platform stores poses as
> they are sent, and every gaze record is well formed under either value, so nothing reports the
> mistake, the verifiers included. Check it in Session Replay.

To check, record a short session in which the robot drives a few meters along a wall or corridor you
can recognize in the scene. Under the right convention the path runs along it in Session Replay;
under the wrong one the path is turned 90 degrees about the scene's origin.

Each session records the value as `c3d.ros.pose_convention`, and doctor's `pose/convention` check
reports it.

### How a pose is converted

ROS axes are right-handed. Session Replay reads poses in Unity's axes, which are left-handed with Y
up, and draws a glTF scene, which is right-handed, as it was authored: it negates X on every
recorded position to put the two together. Both scene conventions therefore change handedness, and
each position map is a reflection. A rotation cannot follow a reflection by reordering its
components alone, so the node converts every rotation for the change of handedness as well: under
`unity`, a ROS quaternion `(x, y, z, w)` is sent as `(-y, z, x, -w)`.

Under `gltf_authored`, a position is the exporter's glTF map with X already negated, `(-x, z, -y)`,
so the viewer's negation puts it back on the scene. The robot's Dynamic Object rotation is carried
across the same mirror, and the viewer undoes it, so the mesh renders in its own glTF orientation.
The gaze rotation carries one more fixed turn, because the platform reads a gaze rotation's
forward along its local +Z axis and this position map sends ROS forward to -X.
[Data format](data-format.md#coordinates-and-units) gives every formula.

## Aligning the robot model

`mesh_yaw_offset_deg` turns the robot's Dynamic Object about its vertical axis, and nothing else:
the gaze channel and the events stay where they are. It applies only with `robot_dynamic_object: true`.
Set it when Session Replay shows the robot model driving sideways or backwards along a correct
path, which happens when the mesh's forward axis is not the robot frame's +X: usually `90` or `-90`.

It corrects yaw only. A mesh rotated about any other axis, such as one whose URDF visual origin
carries a roll, has to be rotated in the mesh file before upload; see [Scenes](scenes.md).

Under `pose_convention: unity`, the Dynamic Object's orientation also depends on the axes its mesh
was authored in, which the node cannot see. Check the model's heading in Session Replay, and correct
a yaw error with `mesh_yaw_offset_deg`.

## Reference frames

| parameter | default | meaning |
| -- | -- | -- |
| `reference_frame` | `map` | The preferred frame. Its origin is the scene's origin. |
| `fallback_reference_frame` | `odom` | Used when the preferred frame is unavailable. |
| `robot_frame` | `base_link` | The robot body. |
| `reference_frame_grace_s` | `5.0` | How long to wait for the preferred frame once only the fallback resolves. |

Session Replay draws the reference frame's origin at the scene's origin. A `map` built by SLAM or
loaded into AMCL must share its origin and axes with the scene, or the whole path is offset or
turned by the difference, and nothing reports it.

Each sample tries the preferred frame first. When a session starts with only the fallback
resolving, the node records nothing for up to `reference_frame_grace_s` while localization comes
up. Then it adopts the fallback and records `robot.reference_frame_unavailable`, with `wanted` and
`using`. A later change in either direction records `robot.reference_frame_changed`, with `from`
and `to`. Camera and lidar poses follow the same frame decision, and record nothing during the
wait. An empty `reference_frame` is a deliberate fallback-only run: no wait, and no event.

### Choosing the body frame

`robot_frame` is the frame whose pose the node samples. The gaze channel reports it unless
`gaze_frame` is set, every event is placed at it, and the robot's Dynamic Object is drawn at it.
Choose the frame the robot's body is built around, normally `base_link`, the default.

Many robots also have `base_footprint`, the body's position on the floor. It is a child of
`base_link` on some robots, such as the TurtleBot 4, and its parent on others. The order does not
matter to the node, which looks the transform up through the whole tree, so either name resolves.
The two are joined by a fixed transform, usually a height alone, so the path has the same shape
from either; what changes is the height the path, the robot and the events are drawn at. Keep
`base_link` unless you want them on the floor. With `robot_dynamic_object: true`, choose the frame
your robot mesh's origin is modeled at. Keep the choice from one session to the next, so sessions
are drawn at one height.

### When the robot has no map frame

`map` exists only while a localizer such as AMCL or a SLAM node publishes it, so a robot without
one falls back to `odom`. `odom`'s origin is wherever the robot started, so the whole path is drawn
at that offset from the scene, with odometry drift on top. There is no scene-origin offset
parameter. Give the robot a `map` frame aligned with the scene, or start it at the scene's origin,
facing the direction ROS +X takes in the scene. doctor's `robot pose` check names the frame in
force.

### Localization jumps

A localizer corrects its estimate in steps. In the preferred frame, the node compares the pose with
the one about 0.1 seconds earlier, or with the previous sample when `sample_rate_hz` is below 10. A
move of more than `relocalisation_jump_m` (0.5 meters) or a turn of more than
`relocalisation_jump_deg` (45 degrees) records one `robot.localisation_jump`, and counts in
`c3d.localisation_jumps`. The event carries `distance_m` and `rotation_deg`, measured over that
comparison, `kind` (`position`, `rotation` or `both`) and `frame`. The pose itself is still
recorded; the event lets an analysis exclude the discontinuity.

Comparing poses about 0.1 seconds apart, rather than consecutive samples, makes detection the same
at any `sample_rate_hz` of 10 or more. A localizer that future-dates its transform, as AMCL does, is
interpolated by tf between two of its updates, so one correction reaches the pose spread over every
sample in between, and consecutive samples at a high rate would each see only a part of it. Two
compared poses are never less than about 0.09 seconds apart, so at least half of a correction spread
over up to about 0.18 seconds falls between one pair: a right-angle re-registration behind AMCL's
7 Hz scans is recorded once at any rate of 10 Hz or more. A correction spread over more than 0.1
seconds, behind a localizer that updates slower than 10 Hz, is split between two comparisons at some
timings. One smaller than twice a threshold can then reach it in neither, and a large one can reach
it in both and be recorded twice. Nothing is compared across a change of reference frame, a stale
transform or the start of a session.

Both thresholds hold for 0.1 seconds of motion. Below a `sample_rate_hz` of 10, each sample covers
more motion, so raise them in proportion, or fast driving and turning read as jumps.

### Stale transforms

When the transform's stamp stops advancing for ten sample periods (and at least 5 seconds), its
publisher has stopped: tf still answers with the last transform, which is no longer where the robot
is. Those samples are skipped and counted in `c3d.stale_poses` and `c3d.pose_failures` until the
transform advances again. A static transform is never stale.

## Planar poses

> [!WARNING]
> On a standard mobile robot, neither `map` nor `odom` carries height or tilt. On a flat floor a
> planar pose and a correct one are identical, so a constant zero height is evidence of nothing.

`odom` from wheel encoders is planar by construction: its height, roll and pitch are always zero.
AMCL builds `map` to `odom` from x, y and yaw alone, and composing two planar transforms stays
planar. A robot that climbs a ramp is therefore recorded flat, and every verifier still passes,
because each compares a session with itself. doctor's `pose channel` check reports how much height
and tilt the frame in force carried, and whether its usual producer is planar.

To record height and tilt, sample a frame whose producer is 6-DOF: `robot_localization` with
`two_d_mode: false` for `odom`, or a 6-DOF SLAM or localizer for `map`. Failing that, broadcast
one: publish a 6-DOF estimate, from simulator ground truth, motion capture, or visual or
lidar-inertial odometry, into tf as the transform to `robot_frame`, and set `reference_frame` to
its parent frame. Two broadcasters of one child frame conflict in tf and the pose alternates
between them, so stop the existing broadcaster of that transform (for `robot_localization`,
`publish_tf: false`). Anchor the new frame at the scene's origin, not at the robot's start, or the
height and tilt come out right on a path drawn in the wrong place.

## The gaze channel

Every pose sample is a position and a rotation, and nothing else: a robot has no eyes, so the gaze
channel sends no gaze point, and Session Replay draws the robot's gaze line straight ahead along
the rotation. By default the pose is the robot body's, `robot_frame` facing along +X, which is
usually at floor level.

`gaze_frame` moves the whole sample to another frame, such as a forward-facing camera's: the gaze
channel then reports that frame's position and rotation, posed at the same instant as the body.
Session Replay's viewpoint then sits at the camera's height and faces where the camera faces. The
position and the rotation always come from one frame, so the viewpoint is somewhere on the robot.

### Optical and body frames

A camera usually has two tf frames: a body-style one, often `*_camera_frame` (+X forward, +Y left,
+Z up), and an optical one, often `*_optical_frame` (+Z forward, +X right, +Y down).
`gaze_frame_convention` says which axis of `gaze_frame` is forward:

| value | forward axis |
| -- | -- |
| `auto` (default) | Inferred from the frame's name. A name with `optical` as a word, between `_` or `/` separators, is optical, unless the next word is `flow`; any other name is body-style. |
| `body_forward_x` | +X |
| `optical_forward_z` | +Z |

A wrong convention turns the gaze rotation 90 degrees off, and nothing reports it, because the
record is still well formed. At configure the node logs the frame, the convention in force and whether it was
inferred: read that line. The node also warns when a frame named like an optical frame is set to
`body_forward_x`.

A `gaze_frame` that does not resolve falls back to the body's pose for that sample, so the session
looks healthy with its viewpoint at floor level. The node counts those samples in
`c3d.gaze_frame_failures` and warns every 10 seconds, and doctor's `gaze frame` check fails for a
frame that does not resolve. Each session records `c3d.ros.gaze_frame` and the convention in force,
`c3d.ros.gaze_frame_convention`.
