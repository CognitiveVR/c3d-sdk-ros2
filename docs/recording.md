# Recording

Besides the pose timeline, the node records what your robot's topics say: sensor series, events,
objectives from action servers, Nav2's behavior tree, and its own health. This page covers what is
recorded and how to choose it. The pose timeline is in [Coordinates](coordinates.md), and camera
and lidar streams are in [Camera and lidar](camera-and-lidar.md).

## The allowlist

`topics` is the allowlist. The node subscribes to each topic in it once the topic appears on the
graph, and records it through the translator for its message type. `topics` is empty by default.

```yaml
/**:
  ros__parameters:
    topics:
      - "/odom"
      - "/imu"
      - "/joint_states"
      - "/scan"
```

- Check the names on your robot first, with `ros2 topic list`. `/odom` is a convention, not a
  promise: a robot running an EKF usually publishes `/odometry/filtered` and no `/odom` at all. On
  a large graph, such as Nav2 with a simulator, discovery takes tens of seconds: list the topics
  again about 20 seconds later, and use the second answer.
- A topic with more than one publisher can need a QoS override; see [Ingest and QoS](#ingest-and-qos).
- The node looks for allowlisted topics every `discovery_period_s` (5 seconds), so a topic that
  appears late is picked up then. While an allowlisted topic is missing, the node warns at most every
  30 seconds, naming every one missing. Right after activation the warning can name topics it has
  not discovered yet, and they are subscribed at the next look.
- A relative name resolves against the node's namespace, and `~/name` names a topic under the node.
- A trailing `*` matches every topic with that prefix, as in `/robot1/*`. A `*` anywhere else is
  refused at configure.
- Leave out `/tf` and `/tf_static`. The pose timeline reads tf through the node's own listener,
  which follows the node's namespace and remaps, and no translator records them as topics.
- Record images and point clouds through `cameras` and `lidars` ([Camera and lidar](camera-and-lidar.md)).
  In `topics` they are refused and never subscribed. `ingest: full_ingest` on the exact topic lifts
  the refusal, but no translator records them.

> [!WARNING]
> A wrong allowlist still produces a session that uploads and looks healthy. With `topics` empty or
> naming topics your robot does not publish, a session holds the pose timeline, the node's own
> `c3d.*` series, the batteries, diagnostics and actions found without an entry, and nothing else
> from your robot. Read the plan line the node logs at configure, and run doctor: its `topics`
> check names allowlisted topics missing from the graph.

## Recorded without an allowlist entry

- Batteries and diagnostics, by message type: every `sensor_msgs/msg/BatteryState` and
  `diagnostic_msgs/msg/DiagnosticArray` topic, while `auto_subscribe_standard_types` is true (the
  default). The node's own `~/diagnostics` is left out.
- Actions: the status topic of every action server whose name is in `action_names`, while
  `action_discovery` is true (the default). See [Actions](#actions).
- Cameras and lidars named in `cameras` and `lidars`, each by its own topic.
- The node's own health series, `c3d.*`, in every session. See [Health series](#health-series).

An entry in `topics` wins over both switches, and a topic both reach is subscribed once.

## Namespaces

A node in a namespace, such as `/robot1`, takes battery and diagnostics topics by type and enrolls
actions by name only under its own namespace: `/robot1/battery_state` and
`/robot1/dock/_action/status`, never `/battery_state` or `/dock/_action/status`. A node at the root
takes them anywhere on the graph. Name any other topic in `topics`. To keep robots on one network
apart, see [Robots on a shared network](sessions.md#robots-on-a-shared-network).

## Per-topic settings

`topic_config.<key>` sets `ingest`, `rate_hz`, `series` and `qos.*` for one allowlist entry. The
key is the entry with `/` and `*` collapsed to `_`, and a leading or trailing `_` trimmed: `/odom`
is `odom`, and `/robot1/imu` is `robot1_imu`. It is read only for an entry in `topics`, so a battery
or action status topic taken without an entry needs one before its key does anything. Configure
refuses, naming it, a key that matches no entry and a key whose field is misspelled.
[Configuration](configuration.md#per-topic-rule-keys) has the full table.

```yaml
/**:
  ros__parameters:
    topics:
      - "/imu"
      - "/ultrasonic"
    topic_config:
      imu:
        rate_hz: 5.0
      ultrasonic:
        series: "ultrasonic.distance_centimeters"
```

At configure the node logs one plan line per entry, with the ingest, rate and series it resolved
to, and a line per subscription with the values in force.

### Rates

`rate_hz` caps how many messages per second a translator records, counted as messages arrive (per
simulated second under `sim_time_on_wire`). Left out, it is the message type's default:

| message type | default |
| -- | -- |
| `nav_msgs/msg/Odometry`, `sensor_msgs/msg/JointState`, `sensor_msgs/msg/Imu`, `std_msgs/msg/Float32`, `std_msgs/msg/Int32MultiArray` | 30 Hz |
| `sensor_msgs/msg/LaserScan`, `sensor_msgs/msg/BatteryState` | 1 Hz |
| `nav2_msgs/msg/BehaviorTreeLog` | 1 Hz, the window its transition rate is averaged over |

Each battery topic is held to its own rate. A status or health change on a message the limit
drops is recorded from the next message it passes, if it still holds. Diagnostics and action status
topics have no rate limit: every message is read for the changes it carries.

A sensor series also skips a value that has moved by 0.001 or less since its previous sample,
unless 2 seconds have passed, so a steady series carries one sample every 2 seconds.

### Series names

`series` names the series a `std_msgs/msg/Float32` or `std_msgs/msg/Int32MultiArray` topic records
into. These types carry no unit and no meaning, so the value is recorded unconverted, and the name
is where the unit goes: `ultrasonic.distance_centimeters`. Left out, the series is the topic with
its slashes turned into dots, plus a suffix: `/ultrasonic` records `ultrasonic.value`. For
`Int32MultiArray` the name is a base: `<base>.0`, `<base>.1` and on, up to 32 elements, and
`<base>.count`, where the base is the dotted topic when `series` is left out.

Every other message type ignores `series`, and the node warns when it subscribes such a topic,
naming the key and the type. A `series` set on a wildcard entry is ignored with a warning, because
one name cannot describe a namespace. The plan line prints the series each entry resolved to, a set
one as `series=<name> (if the topic is std_msgs/msg/Float32 or std_msgs/msg/Int32MultiArray)`, and
`series=(from topic name)` on an entry you named a series for means the series is under another
entry's key.

### Ingest and QoS

`ingest: refuse` keeps a topic from being subscribed at all. The other ingest classes currently
behave alike; see [Configuration](configuration.md).

The node chooses each subscription's QoS when it subscribes, from the publishers it has discovered
on the topic at that moment:

- Reliability: `reliable` when every publisher offers it, otherwise `best_effort`.
- Durability: `transient_local` when every publisher offers it, otherwise `volatile`.
- Depth: the deepest any publisher offers, and at least 10.
- With no publisher discovered yet: `best_effort` and `volatile`, which connect to any publisher.

The choice holds until the node is deactivated. The `subscribed` line the node logs for each topic
gives the profile and the reason, such as `publishers disagree on durability, requesting volatile`.
`qos.*` overrides any part of it; a value other than those
[Configuration](configuration.md#the-allowlist-and-per-topic-rules) lists refuses configure, naming
the parameter.

A publisher discovered after the node subscribed can offer less than the node requested. The common
case is `/joint_states` on a robot that runs ros2_control: `joint_state_broadcaster` publishes it
transient-local, and `joint_state_publisher` publishes it volatile. When the node subscribes before
it has discovered the volatile publisher, it requests `transient_local`, which a volatile publisher
cannot serve, and no message from that publisher reaches the node. Which publisher is discovered
first varies from one start to the next. The node logs an error that names the policy and the
override:

```text
[ERROR] [...] [cognitive3d]: QoS incompatibility on /joint_states: 1 so far, last on the DURABILITY_QOS_POLICY. … Set topic_config.joint_states.qos.durability: volatile to match a volatile publisher.
```

The topic's line in the [report](sessions.md#the-report-line) is marked `[QOS INCOMPATIBLE]`, and
its `~/diagnostics` status is an error. Set the override the error names:

```yaml
/**:
  ros__parameters:
    topic_config:
      joint_states:
        qos:
          durability: "volatile"
```

A volatile subscription receives from transient-local and volatile publishers alike, and misses
only messages published before it subscribed, which on a topic published continuously is nothing.
A best-effort publisher discovered after the node chose `reliable` is the same case for
reliability, and its error names `qos.reliability: best_effort`.

doctor's `topics/qos` check finds this before launch: for each allowlisted topic, it warns when the
publishers disagree on durability or reliability, or when the subscription's QoS would be
incompatible with one of them, and names the override. Set the override for a topic it warns about
even when the node happens to connect, because the discovery order can change at the next start.

## What each message type records

At configure the node logs the message types it can translate (`translating N message type(s)`).
A topic of any other type is subscribed and counted, its line in the periodic report reads
`no-xlat`, and nothing from it reaches the session.

| message type | records |
| -- | -- |
| `nav_msgs/msg/Odometry` | `odom.speed` (meters per second), `odom.yaw_rate` (radians per second), `odom.distance_travelled` (meters, summed from the pose; a step of 10 meters or more is taken as a reset, not motion) |
| `sensor_msgs/msg/Imu` | `imu.roll_degrees`, `imu.pitch_degrees`, `imu.yaw_rate_degrees_per_second`, `imu.acceleration_meters_per_second_squared` (the magnitude, gravity included). An estimate whose covariance marks it absent (REP-145) is skipped, not recorded as zero. The IMU never feeds the pose timeline. |
| `sensor_msgs/msg/JointState` | `joints.count`, `joints.effort_rms`, `joints.effort_max`, `joints.velocity_max` |
| `sensor_msgs/msg/LaserScan` | `scan.range_min_meters` and `scan.range_mean_meters` over valid returns, and `scan.valid_fraction`. The ranges themselves are recorded only by a lidar stream. |
| `sensor_msgs/msg/BatteryState` | `battery.percent` (0 to 100), `battery.voltage`, `battery.current`, `battery.temperature`, and the events `robot.power.status_changed` and `robot.power.health_changed` |
| `diagnostic_msgs/msg/DiagnosticArray` | the events `robot.diagnostic.degraded` and `robot.diagnostic.recovered` when a status changes level, and `robot.diagnostic.degraded` for a status first seen unhealthy |
| `std_msgs/msg/Float32` | one series; see [Series names](#series-names) |
| `std_msgs/msg/Int32MultiArray` | a series per element and a count; see [Series names](#series-names) |
| `action_msgs/msg/GoalStatusArray` | objectives; see [Objectives](#objectives) |
| `nav2_msgs/msg/BehaviorTreeLog` | with `cognitive3d_ros_nav2` only; see [Nav2](#nav2) |

A second battery topic records under its own name. The first battery topic a session receives
keeps `battery.*`, and each further one records `battery.<topic>.*`, its slashes turned into dots:
`battery.robot1.aux_battery.percent`. Which topic keeps the bare name is decided by arrival order
in each session, so on a robot with two batteries the bare series can belong to a different battery
from one session to the next.

### One topic per type for odometry, IMU, joint states and scans

The odometry, IMU, joint-state and laser-scan translators name their series by message type, not by
topic, and serve every topic of their type with one rate limit. Two allowlisted topics of one of
these types interleave in the same series and split one rate between them, and
`odom.distance_travelled` sums both. Allowlist one topic per type for these four. The node warns
when it subscribes a second, naming both, and doctor's `shared series` check reports the pair
before launch.

## Events

An event is a named, timestamped point in the session, with properties. Each is placed where the
robot body was at the latest pose sample. Before the first pose, an event is placed at the origin
with the property `c3d.position_unknown: true`.

### Publishing your own events

Any node can publish a `cognitive3d_ros_msgs/msg/Event` on the node's `~/events` topic, which is
`/cognitive3d/events` under the shipped launch file:

```bash
ros2 topic pub --once /cognitive3d/events cognitive3d_ros_msgs/msg/Event \
  "{name: 'mission.waypoint_reached', properties_json: '{\"leg\": 3, \"speed_meters_per_second\": 0.42}'}"
```

| field | meaning |
| -- | -- |
| `name` | Required. Dotted and namespaced by convention, such as `mission.waypoint_reached`. An event with an empty name is dropped. |
| `properties_json` | Optional. A flat JSON object of scalars, each value keeping its JSON type. Anything else records the event without properties, with a warning. A key in `c3d.` is dropped, because those event properties are the SDK's: the node warns once per key per session and counts each drop in `c3d.event_properties_dropped`. Spell units out in property names: `speed_meters_per_second`. |
| `dynamic_object_id` | Optional. Attaches the event to the robot's Dynamic Object, `robot_object_id`, when `robot_dynamic_object` is true. Empty attaches it to the session. |
| `stamp` | Optional. When it happened, on the publisher's ROS clock. Zero means when the node receives it. |

- A non-zero `stamp` is placed like a header stamp ([Time](sessions.md#time)): one more than 1
  second ahead of the node's ROS clock, or more than an hour behind it, is placed at the current
  time. A stamp from before the session began is placed at the session's start, with a warning once
  per session.
- The position is always where the robot is when the node receives the event, whatever the stamp.
- The subscription is reliable, with a queue of 100. A best-effort publisher does not connect, and
  none of its events arrive. `ros2 topic pub` publishes reliably by default, and with `--once` it
  waits for a subscriber before it publishes.
- An event published while the node is active with no session open is dropped and counted: the
  report line shows `DROPPED=` with the count, and the node warns. One published while the node is
  inactive is lost uncounted; see [Deactivate is the kill switch](sessions.md#deactivate-is-the-kill-switch).

### Built-in events

| event | from |
| -- | -- |
| `<action>.<state>`, `<action>.outcome_unknown` | action servers; see [Objectives](#objectives) |
| `robot.diagnostic.degraded`, `robot.diagnostic.recovered` | a diagnostics status changing level. Carries `component`, `level`, `previous_level`, `message`, `hardware_id`, `topic`, and up to 8 of the status's values as `kv.<key>`. |
| `robot.power.status_changed`, `robot.power.health_changed` | a battery's status or health changing. Carries `from`, `to` and `topic`, and `percent` for a status change. |
| `robot.topology` | once per session: the tf frame count, the reference and robot frames, and up to 40 frame names |
| `robot.reference_frame_unavailable`, `robot.reference_frame_changed` | the pose timeline's frame; see [Reference frames](coordinates.md#reference-frames) |
| `robot.localisation_jump` | a localization step; see [Localization jumps](coordinates.md#localization-jumps) |
| `nav.recovery.started`, `nav.recovery.finished`, `nav.recovery.failed`, `nav.recovery.halted`, `nav.bt.node_failed` | Nav2's behavior tree; see [Nav2](#nav2) |
| `c3d.remote_variables.unavailable` | a failed remote-variables fetch; see [Remote variables](remote-variables.md) |

No translator records events while no session is open, and each translator starts over at every
session.

### The event rate cap

`event_rate_limit_hz` (5 by default) caps how many events each source records per second, allowing
a burst of one second's worth. A source is one topic, `~/events` (shared by everything publishing
to it), the pose timeline's own events, or the remote-variables fetch. The excess is discarded and
counted in `c3d.events_suppressed`, and the node warns once per source per session. Sensor series
and poses are not capped. `0` removes the cap, which is there because one source in a loop would
otherwise crowd out every event after it. The cap counts wall-clock seconds, also under
`sim_time_on_wire`.

## Objectives

The status topic of an action server, `/<action>/_action/status`, becomes events named
`<action>.<state>`, one per goal transition: `navigate_to_pose.succeeded`, `dock.aborted`. The
Cognitive3D dashboard matches objectives by event name, so these become objectives with no further
setup. `<action>` is the last segment before `/_action/status`, so `/robot1/dock/_action/status`
and `/robot2/dock/_action/status` both record `dock.*`, and a fleet aggregates. Each event carries
`goal_id`, `action` (the full topic), `action_name`, `status`, `previous_status` and `duration_s`,
the time since the goal was first seen.

| goal status | event |
| -- | -- |
| ACCEPTED | `<action>.started` |
| EXECUTING | `<action>.executing` |
| CANCELING | `<action>.canceling` |
| SUCCEEDED | `<action>.succeeded` |
| CANCELED | `<action>.canceled` |
| ABORTED | `<action>.aborted` |
| any other status | `<action>.unknown` |

> [!WARNING]
> Key the start of a goal on `<action>.executing`, never `<action>.started`. Nav2's and the iRobot
> Create 3's servers accept goals with `ACCEPT_AND_EXECUTE`, which moves a goal to EXECUTING before
> its first status publication, so ACCEPTED never reaches the topic and `.started` is never
> recorded. An objective on `.started` matches nothing, and reports no runs of an action that ran.

- A goal already finished when the node first sees it records nothing: a status topic replays its
  last state to a new subscriber, and that outcome belongs to a run before this session.
- Every session starts tracking afresh. A goal already running when `~/start_session` opens a
  session records from its next status change, and when that change is its outcome, it records
  nothing. Start the session before sending the goal.
- A goal that leaves its own server's status array for 30 seconds without a terminal state records
  `<action>.outcome_unknown`, with `last_status`. A quiet status topic is not a vanished goal: a
  server publishes only when one of its goals changes state.

## Actions

With `action_discovery: true`, the default, the node enrolls every action server whose name is in
`action_names`, under its namespace (anywhere, for a node at the root), with no allowlist entry.
The default list covers Nav2 and the iRobot Create 3:

```text
navigate_to_pose  navigate_through_poses  follow_waypoints  spin  backup
drive_on_heading  wait  assisted_teleop  dock_robot  undock_robot
dock  undock  drive_distance  drive_arc  rotate_angle  navigate_to_position  wall_follow
```

- Setting `action_names` replaces the list rather than adding to it, so keep the defaults you want
  beside a custom action's name.
- `action_names: []` stops the node at startup, because rclcpp cannot type an empty YAML list. To
  enroll no actions by name, set `action_discovery: false`; status topics named in `topics` are
  still recorded.
- Nav2's planner, smoother and controller actions (`compute_path_to_pose`, `smooth_path`,
  `follow_path` and their kind) are left out on purpose: every replan preempts the running
  `follow_path` goal, which then ends aborted, so a healthy run records hundreds of aborts. Naming
  one records it.
- `ingest: refuse` in a status topic's `topic_config` block keeps that action from being recorded.
  The block is read only when the status topic is in `topics`, so list it there too.

An action that is not recorded records nothing at all, which afterwards looks exactly like an
action that never ran. When the node finds action servers it logs both halves, as
`action servers on the graph: N recorded [...]; M NOT recorded [...]` with the reason for each, and
doctor's `actions` check lists them before launch.

## Nav2

Nav2's actions are recorded by default through action discovery. Its behavior tree log,
`/behavior_tree_log`, needs the `cognitive3d_ros_nav2` package, which adds the
`nav2_msgs/msg/BehaviorTreeLog` translator:

1. Install `cognitive3d_ros_nav2` ([Install](install.md)). The shipped launch file runs its
   executable whenever the package is installed; without the launch file, run
   `ros2 run cognitive3d_ros_nav2 cognitive3d_node`. It is the same node, with the same parameters
   and services.
2. Add `/behavior_tree_log` to `topics`. It is not taken by type.

`doctor` checks the same build of the node the launch file runs, the Nav2 build whenever
`cognitive3d_ros_nav2` is installed, and names the build it checked. In its output, as in the
node's configure log, the line `translating N message type(s)` names
`nav2_msgs/msg/BehaviorTreeLog` once the translator is in.

What the behavior tree records:

- `nav.bt.transitions_per_second`, how busy the tree is: its node status changes divided by the
  time they covered, sampled at most at the topic's `rate_hz`. The tree's routine cycling, such as
  replanning, is recorded here and not as events.
- `nav.recovery.started` when a recovery behavior starts to run, and one outcome:
  `nav.recovery.finished`, `nav.recovery.failed`, or `nav.recovery.halted` when the tree stops it
  mid-run because the goal was canceled or preempted. A recovery that completes within one tick,
  such as a costmap clear, records only its outcome. The recovery behaviors are `Spin`, `BackUp`,
  `Wait`, `DriveOnHeading`, `AssistedTeleop`, the costmap clearers, and the names Nav2's default
  trees give them, such as `SpinRecovery` and `ClearLocalCostmap-Subtree`.
- `nav.bt.node_failed` when any other node fails, except Nav2's condition nodes (`GoalUpdated`,
  `IsStuck` and the rest), for which a failure is the answer "no".

Each event carries `node`, `from`, `to`, `uid` (not on Humble), `topic`, and `bt_stamp`, the log's
own timestamp, recorded as data. A second behavior tree log topic records its rate as
`nav.bt.<topic>.transitions_per_second`; the first a session receives keeps the bare name, as with
batteries.

In a component container, load the Nav2 package's component, `cognitive3d_ros_nav2::Cognitive3DNode`,
which is the same node with this translator added:

```bash
ros2 component load <container> cognitive3d_ros_nav2 cognitive3d_ros_nav2::Cognitive3DNode
```

Make `<container>` a `component_container_isolated` or a container of the node's own, never a
shared `component_container`. On a namespaced robot, load it with the robot's namespace and
`-r /tf:=tf -r /tf_static:=tf_static`, as its other nodes are.
[Running in a container or under systemd](sessions.md#running-in-a-container-or-under-systemd)
says why, and shows the command.

Loaded as the base component, `cognitive3d_ros::Cognitive3DNode`, the node subscribes an allowlisted
`/behavior_tree_log`, counts it and records nothing from it. Action objectives are recorded by
either. `doctor` reports on the launch file's build and cannot see which component a container
loaded: confirm it from the loaded node's configure log, whose line `translating N message type(s)`
names `nav2_msgs/msg/BehaviorTreeLog` only for the Nav2 component.

On Humble, Nav2's behavior tree log carries no node uid, so two behavior tree nodes that share a
name are treated as one. In Nav2's default tree, when the `FollowPath` action fails and the recovery
node of the same name then gives up, one `nav.bt.node_failed` is recorded instead of two. Behavior
tree events on Humble carry no `uid` property.

## Health series

The node records its own health into every session as `c3d.*` sensor series, every 5 seconds of
session time while the session is open: 5 simulated seconds under `sim_time_on_wire`. They answer
whether a gap in the data is real. Counts marked "since configure" run across every session since
the node was configured; the others start over with each session.

| series | meaning | counts |
| -- | -- | -- |
| `c3d.spool_chunks`, `c3d.spool_mb` | chunks and MiB in the spool, every session's included | current |
| `c3d.records_dropped` | records lost before reaching the spool: a full in-memory batch, an empty or unencodable camera frame, scan or cloud, or a part the spool refused or could not serialize | since configure |
| `c3d.spool_write_failures` | parts the spool refused: no room under the cap, or a failed write | since configure |
| `c3d.serialization_failures` | parts that could not be serialized | since configure |
| `c3d.chunks_evicted` | chunks evicted at `spool_max_mb`, never to be uploaded | since configure |
| `c3d.upload_failures` | failed upload attempts, each of which kept its chunk | since configure |
| `c3d.upload_auth_rejections` | uploads answered 401, 403 or 407 | since configure |
| `c3d.upload_auth_paused` | 1 while every upload is paused for refused credentials | current |
| `c3d.upload_auth_refused_streams` | how many streams' credentials are refused | current |
| `c3d.chunks_held` | chunks kept but not sent, because they were recorded for another host, or for another project (another application key and another scene) | current |
| `c3d.chunks_deferred` | chunks waiting behind the rest of the spool after failing on their own | current |
| `c3d.upload_deferred_streams` | how many streams wait behind the rest of the spool after their chunks kept failing on their own | current |
| `c3d.thread_exceptions` | exceptions caught on the session's own threads | since configure |
| `c3d.pose_failures` | pose samples with no transform, or a stale one | this session |
| `c3d.stale_poses` | pose samples skipped because the transform stopped advancing | this session |
| `c3d.localisation_jumps` | localization jumps | this session |
| `c3d.events_suppressed` | events discarded by the rate cap | this session |
| `c3d.event_properties_dropped` | event properties dropped because their key was in `c3d.` | this session |
| `c3d.gaze_frame_failures` | samples whose `gaze_frame` did not resolve; only with `gaze_frame` set | this session |
| `c3d.camera.<name>.*`, `c3d.lidar.<name>.*` | per camera and lidar: frames or scans, bytes, pose failures, refusals and drops; see [Camera and lidar](camera-and-lidar.md) | this session |

Between sessions, the upload state is on the periodic report line and in the
`cognitive3d: uploads` status on `~/diagnostics`; see [Doctor and verification](doctor-and-verification.md).
