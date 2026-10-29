# Sessions

A session is one recording: a bounded run of the robot that Cognitive3D stores and Session Replay
plays back. This page covers how the node opens and closes sessions, the two services another node
uses to drive them, how time reaches the wire, and the spool, the on-disk queue every recording
passes through before it is uploaded.

## Lifecycle

The node (`cognitive3d_node`) is a ROS 2 managed lifecycle node. It starts unconfigured, and it
records nothing until it is configured and activated.

| transition | what the node does |
| -- | -- |
| configure | Reads every parameter, and refuses with a message naming each problem when one is missing or invalid. Opens the spool, closes any session an earlier process left open (see [Shutdown and crashes](#shutdown-and-crashes)), starts uploading what the spool holds, and creates `~/start_session` and `~/end_session`. Subscribes to nothing. |
| activate | Opens a session when `auto_start_session` is true, then creates every subscription: the allowlisted topics, the cameras and lidars, the tf listener and `~/events`. Pose sampling starts at `sample_rate_hz`. An activation that fails part-way, refused or thrown, is undone whole: the node stays inactive with nothing subscribed, and a session it had opened ends with the reason `activation_failed`. |
| deactivate | Ends the open session, and destroys every subscription and timer. |
| cleanup | Ends any open session, gives the uploads a last 2 seconds, stops uploading and releases the spool. |
| shutdown | Ends any open session, from any state. |

Configure and activate the node with whatever drives lifecycle transitions in your deployment: a
lifecycle manager, or by hand:

```bash
ros2 launch cognitive3d_ros cognitive3d.launch.py params_file:=<path/to/params.yaml>
ros2 lifecycle set /cognitive3d configure
ros2 lifecycle set /cognitive3d activate
```

Or let the launch file do it. With `autostart:=true` it configures the node as soon as it starts,
and activates it once configure succeeds:

```bash
ros2 launch cognitive3d_ros cognitive3d.launch.py params_file:=<path/to/params.yaml> autostart:=true
```

Only the launch's own configure is followed by an activate. After that the lifecycle is yours: a
later cleanup and configure leaves the node inactive. A configure that fails leaves the node
unconfigured, with the reason in its log.

A node that is never activated records nothing and reports no error. Check its state with
`ros2 lifecycle get /cognitive3d`. The configure refusals are listed in
[Configuration](configuration.md).

### Deactivate is the kill switch

Deactivating destroys every subscription the node holds: the topics, the cameras and lidars, the tf
listener and `~/events`. An inactive node receives no topic traffic at all, rather than receiving it
and discarding it. The session services stay up and refuse, and an event published while the node
is inactive is lost without being counted, because nothing is subscribed to receive it.

Deactivating stops recording, not uploading: chunks already in the spool keep uploading. To stop
network traffic too, clean the node up, or pause uploads (see [Pausing uploads](#pausing-uploads)).

The node publishes nothing except its own `~/diagnostics`.

## Automatic sessions

With `auto_start_session: true`, the default, every activation opens one session and the matching
deactivation ends it. A standalone robot needs nothing more.

An automatic session takes its name from the `session_name` parameter, so a name set in the params
file repeats on every session the node opens. To name each run, call `~/start_session` with its own
`session_name` (see [Starting a session from another node](#starting-a-session-from-another-node)),
or set the parameter before the session starts, which applies from the next session:

```bash
ros2 param set /cognitive3d session_name "patrol/run-008"
```

Left empty, each session is named by its id.

With `auto_start_session: false`, activation brings ingest up without opening a session, and
nothing is recorded until another node calls `~/start_session`. Use it when a mission or task
orchestrator owns the session boundaries. While the node is active with no session open, no topic
is recorded, no pose is sampled, and an event published on `~/events` is dropped and counted: the
periodic report line shows `DROPPED=` with the count, and the node warns.

## Starting a session from another node

`~/start_session` opens a session with a name, typed properties and a participant id, and returns
its id. Under the shipped launch file the service is `/cognitive3d/start_session`:

```bash
ros2 service call /cognitive3d/start_session cognitive3d_ros_msgs/srv/StartSession \
  "{session_name: 'patrol/run-007', properties_json: '{\"site\": \"warehouse-3\", \"trial\": 7}', participant_id: '$(uuidgen)'}"
```

| request field | meaning |
| -- | -- |
| `session_name` | Recorded as `c3d.sessionname`. Empty falls back to the `session_name` parameter, which names every session that does not get a name here, then to the session id. |
| `properties_json` | A flat JSON object of session properties, each value keeping its JSON type: a number, a string or a boolean. Nested objects, arrays and `null` are refused. |
| `participant_id` | Recorded as `c3d.participant.id`, and the identifier A/B arms are drawn for. Empty falls back to `device_id`. At most 64 bytes. See [Choosing the participant id](remote-variables.md#choosing-the-participant-id). |

| response field | meaning |
| -- | -- |
| `ok` | True when this call opened a session. |
| `session_id` | The new session's id. On a refusal, the id of the session already running, or empty. |
| `message` | Empty on success; otherwise what went wrong and what to do. |
| `remote_variables_json` | The session's resolved remote variables and A/B arms. See [Remote variables](remote-variables.md). |

With `remote_variables` on, the call returns once the session's remote variables are fetched,
which takes up to `remote_variables_timeout_s` (3 seconds by default), and the node's other
callbacks wait meanwhile.

A session id is the session's start in Unix seconds and the device id, `<seconds>_<device_id>`.
Sessions the node begins within the same second, between one configure and the next, take
successive seconds, so their ids differ. A cleanup and configure, or a restart, starts over: a
session begun then, in the same second as the session before it, can share that session's id, and
the platform merges the two. An orchestrator that cleans up and reconfigures the node between runs
must start each run at least one second after the previous run started.

### The refusal contract

`ok: false` with a non-empty `session_id` means exactly one thing: a session is already running, it
is that one, and this call did not open it. Leave it alone, or end it if it is yours to end. Every
other refusal returns an empty `session_id` and has opened nothing, so it is safe to fix the cause
and call again.

| refused because | `session_id` |
| -- | -- |
| A session is already running. With `auto_start_session: true`, activation opened it: end it first, or set `auto_start_session: false`. | the running session |
| The node is inactive. Sessions exist only while it is active. | empty |
| The node is not configured, or `dry_run` is set. | empty |
| `properties_json` is not a flat JSON object of scalars. | empty |
| `participant_id` is longer than 64 bytes. | empty |
| `properties_json` names a key in `c3d.`, the SDK's namespace. `c3d.remote_variable.*` is allowed while `remote_variables` is false. | empty |
| The wall clock is unset. See [An unset wall clock](#an-unset-wall-clock). | empty |
| Anything else; the message points at the node log. | empty |

## Ending a session

`~/end_session` ends the running session:

```bash
ros2 service call /cognitive3d/end_session cognitive3d_ros_msgs/srv/EndSession "{reason: 'completed'}"
```

`reason` is recorded on the session's end event, and empty records `ended_by_service`. Say why the
session ended (`completed`, `aborted`, `operator_stop`): a session the robot gave up on is not the
same measurement as one that finished. The response carries the id of the session it ended. With
no session running, the call returns `ok: false` and an empty id.

Ending a session writes its end to the spool and returns; it never waits for an upload. Once the
call returns, stopping the process loses nothing and adds no abnormal end: what is spooled uploads
in the background, or at the next start.

Sessions the node ends on its own carry the reason:

| reason | when |
| -- | -- |
| `deactivated`, `cleanup` | the lifecycle transition of that name |
| `shutdown` | the shutdown transition, or a stop by SIGINT or SIGTERM, from the executable or a component container |
| `activation_failed` | an activation that opened the session and then failed |
| `ros_time_jumped_backwards` | ROS time stepped back; see [Clock jumps](#clock-jumps) |
| `abnormalEnd` | a session closed after a crash; see [Shutdown and crashes](#shutdown-and-crashes) |
| `destructor` | the node was destroyed while its process ran on, as when a container unloads it |

## What every session records

The node records properties about every session, so a stored session can be read without the launch
configuration beside it: the SDK version (`c3d.version` and `c3d.app.version`), the device, the
participant, the ROS distro, RMW, domain and namespace, the pose convention and the clock settings.
More appear when a feature is on, such as a gaze frame, a camera or remote variables.
[Session properties](data-format.md#session-properties) lists every one.

Your own properties come from the `session_properties` parameter and from `properties_json`. A
`session_properties` entry is a `key=value` string, typed from its text: `true` and `false` are
booleans, a whole number is an integer, a decimal is a double, and anything else, a zero-padded
number such as `002` included, is a string. `properties_json` keeps each value's JSON type, and
wins over `session_properties` for a key both set. Types matter: the platform indexes numeric
properties separately from text, so a number sent as a string is invisible to a numeric filter.

Keep your keys out of the `c3d.` namespace, where the SDK records its own properties. A
`session_properties` entry there refuses configure, naming the key, and a `ros2 param set` that adds
one is rejected; a `~/start_session` whose `properties_json` names one is refused and opens nothing. The
one exception is `c3d.remote_variable.*` with the remote-variables fetch turned off; see
[Turning the fetch off](remote-variables.md#turning-the-fetch-off).

## Time

Every timestamp on the wire is Unix epoch seconds from the wall clock. ROS time never reaches the
wire: under `use_sim_time` a simulator's `/clock` starts near zero, and the platform accepts a
session stamped in 1970 and never shows it.

- Poses, and the events the pose timeline produces, are stamped when they are sampled. Sensor
  samples, and the events the node derives from topics, are stamped when their message arrives.
- Camera frames, lidar scans and events that carry a stamp are placed by that stamp: its age
  against the node's ROS clock is subtracted from the wall clock, so stamps from a simulation land
  in the right year and keep their spacing. A stamp more than 1 second ahead of the node's ROS clock,
  or more than an hour behind it, is placed at the current time instead.
- A session whose start was moved ahead of the wall clock to keep its id unique (see
  [Starting a session from another node](#starting-a-session-from-another-node)) records on the wall
  clock moved by the same amount, so no pose, sample, event or end precedes its start. A camera or
  lidar frame stamped before the session's start is still placed at its stamp.

### An unset wall clock

The node refuses to begin a session while the wall clock reads earlier than 2026-01-01T00:00:00Z,
which means the clock has not been set, as on a board without a real-time clock that boots at 1970
until NTP synchronizes it. `~/start_session` returns `ok: false`, an empty id and a message
beginning `no session was started: the wall clock reads`. With `auto_start_session: true`,
activation fails and the node stays inactive. The launch file's `autostart:=true` activates once
and does not retry, so a node started this way records nothing until something activates it again.
doctor's `wall clock` check fails for the same reading.

The floor catches only a clock before 2026-01-01. A board that restores the time it last saved,
as `fake-hwclock` and `systemd-timesyncd` do at boot, starts with a clock that is set but stale.
The node and doctor accept it, sessions are dated at the stale time, and a session that is open
when NTP corrects the clock jumps forward with it. Each camera's encoder, which times its chunks by
the wall clock, is restarted at the step, so the chunks after it keep their length.
Start the node only once the clock is
synchronized: `timedatectl` reports it, and under systemd the unit can wait for it (see
[Running in a container or under systemd](#running-in-a-container-or-under-systemd)).

### Simulation time

Set `use_sim_time: true` when the robot runs in a simulator; the launch file's `use_sim_time:=true`
argument sets it. The wire stays on the wall clock, so a run of 5 simulated minutes that took 20
wall-clock minutes is a 20-minute session. Every rate limit admits its rate per wall-clock second,
so that session also carries four times the samples a real 5-minute run would.

`sim_time_on_wire: true` keeps the session in the real calendar while spacing it by simulated time.
At session start the node pairs the session's wall-clock start with the current ROS time, and from
then on a record's time is that start plus the ROS time elapsed since.

- It requires `use_sim_time`. Configure refuses it otherwise.
- The session id still embeds the real wall-clock start.
- Pose sampling and the rate limits for topics, cameras and lidars count per simulated second, so a
  paused simulation records no poses.
- The session's end and length describe the simulated run.
- The event rate cap stays on the wall clock.
- A session that opens before anything publishes `/clock` stays on the wall clock until it does.
- Time on the wire never runs backwards within a session.

Whether to set it depends on the simulator's real-time factor: the simulated seconds that pass per
wall-clock second, which Gazebo shows as RTF. At a factor of 1 the two settings record the same
session. Below 1, a session recorded without `sim_time_on_wire` is longer than the run by one over
the factor: at 0.25, 5 simulated minutes make a 20-minute session. Set `sim_time_on_wire: true`
whenever the factor stays below 1. Its rates then count per simulated second, so at a factor of 0.2
the default 30 Hz pose rate records 6 poses per wall-clock second, and 30 per simulated second.

On a computer with no simulator window, measure the factor from `/clock`, with two readings about
30 seconds apart:

```bash
sim0=$(ros2 topic echo --once /clock --field clock.sec | head -n 1); wall0=$(date +%s)
sleep 30
sim1=$(ros2 topic echo --once /clock --field clock.sec | head -n 1); wall1=$(date +%s)
echo "$((sim1 - sim0)) simulated seconds in $((wall1 - wall0)) wall-clock seconds"
```

7 simulated seconds in 30 wall-clock seconds is a factor of about 0.23.

### Clock jumps

When ROS time steps backwards by more than 0.5 seconds, for a simulator reset, a looping bag or a
large correction to the system clock, the node ends the session with the reason
`ros_time_jumped_backwards` and opens a new one. The new session keeps the name (when one was
given), the properties, the participant id and the remote-variable arms of the one it replaces,
and does not fetch the arms again.

## The spool

Every recording is written to disk before any upload is attempted, and uploads run in the
background, oldest first. A lost network or a lost robot loses nothing already spooled, and no
recording call ever waits for the network.

### Where the spool lives

The spool is `spool_dir`. Unset, it is `$XDG_STATE_HOME/cognitive3d`, or
`$HOME/.local/state/cognitive3d` when `XDG_STATE_HOME` is not an absolute path; configure refuses
when neither variable names an absolute directory. The node and `doctor` create the directory, and
any parent of it they have to create, readable by its owner alone, because chunks hold poses and
camera frames. A directory that already exists keeps its permissions; the `chunks/` and `sessions/`
inside it are owner-only either way. Each chunk reaches the disk before it becomes
visible, so after a power cut a chunk is complete or absent.

### One process per spool directory

A spool directory belongs to one process at a time. A second node or process configured with a
directory already in use fails to configure with `spool_dir … is already in use by process <pid>`.
Give every node its own `spool_dir`, including two nodes in one component container. The lock goes
when the process exits, however it exits. doctor's `spool directory` check reports a directory
another process holds.

### The size cap

`spool_max_mb` (1024, 1 GiB, by default) caps the spool on disk. At the cap, the oldest chunks of
the lowest tier are evicted to make room. From the first evicted to the last:

1. camera and lidar
2. poses and sensor series
3. the robot's Dynamic Object samples
4. events, which carry objectives and session boundaries, and the pose parts that carry session
   properties

The pose parts that carry a session's properties are kept with the events because the platform
discards a session whose properties never arrive, `c3d.app.version` among them. A session's last
pose part carries its whole property set again, so a session whose first parts were evicted still
delivers its properties.

A chunk never evicts one of a higher tier. When it cannot fit by evicting its own tier and below,
it is dropped instead, and counted in `c3d.spool_write_failures` and `c3d.records_dropped`. An
evicted chunk is never uploaded. The node logs the first eviction and every 50th after it, counts
them in `c3d.chunks_evicted`, and `~/diagnostics` warns once any has happened.

### Outages and refused keys

| the gateway answers | the node |
| -- | -- |
| a success: any 2xx carrying the Cognitive3D gateway's `cvr-request-time` header | removes the chunk |
| 401, 403 or 407 | tries once more a second later, then pauses every upload with everything kept, logged once at error level. Authentication is tried again every `retry_max_s`, one stream at a time, and the first accepted chunk resumes uploads. A stream that stays refused while another is accepted stops alone. |
| 400, 413, 415 or 422 | discards the chunk, whose payload was rejected, and logs the status |
| anything else, or nothing | keeps the chunk and backs off: `retry_min_s` (60 seconds), doubling up to `retry_max_s` (240 seconds). A `Retry-After` can lengthen a wait, up to an hour. 404 is logged as a configuration problem. |

The last row covers redirects, a captive portal's 2xx without the gateway's header, 404, 408, 429,
every 5xx and a timeout. A chunk that fails three times on its own while the gateway accepts others
is moved behind the rest of the spool and retried on its own interval, so it never blocks the
queue; it is never deleted for failing. When a second chunk of the same stream fails on its own
too, the whole stream steps aside: its chunks are kept, wait behind the rest of the spool, and are
tried again on the stream's own interval, from `retry_max_s` doubling up to an hour, until one is
accepted and the stream resumes. An outage, where every stream fails, defers nothing. A deferred
stream shows as `[STREAMS DEFERRED: <streams>]` on the report line, a WARN `cognitive3d: uploads`
status naming it in `deferred_streams`, and `c3d.upload_deferred_streams`.

Spooled data is sent only where it was recorded for:

- A chunk recorded for another host, such as another Cognitive3D environment, is kept and not sent
  until a process configured for that host uploads it.
- A chunk recorded for another project, under another application key and for another `scene_id`,
  is kept and not sent under this one's key, which would file it in an empty scene of this
  project. The node logs that the backlog was recorded under another application key, for another
  scene. A key rotated within a project, with the same `scene_id`, sends the backlog under the new
  key, and a new scene under the same key sends it to the scene it was recorded for.

`c3d.chunks_held` counts the chunks held either way. To clear a held backlog, configure a node on
that spool with the old key and the old project's scene, and let it upload; or, while no node runs
on the spool, delete the `chunks/` directory in it, which loses that data. Better still, let the
spool empty before you move a robot to another project or stack.

> [!WARNING]
> A refused application key loses nothing, but nothing uploads either, and the spool grows until
> its cap evicts. Watch for `[UPLOADS PAUSED: credentials refused]` on the report line, an ERROR
> `cognitive3d: uploads` status on `~/diagnostics`, or `c3d.upload_auth_paused` reading 1.

### Shutdown and crashes

An orderly stop, by `~/end_session`, deactivate, cleanup, Ctrl-C or SIGTERM, writes the session's
end to the spool and returns without waiting for an upload. As the process exits, uploads get up to
2 more seconds. If anything is still spooled, one log line says how many chunks are left and whether
a session's end is among them, and they upload at the next start. Uploads start at configure,
before activation, so a node that is configured and never activated still uploads its spool.

A session whose end is still in the spool stays open on the platform until something uploads it.
On a robot that is about to be retired or reimaged, configure the node once more, with its network
up, and let the spool empty.

After a crash, a power cut or SIGKILL, the session's marker is left open in the spool. At the next
configure, the node closes every such session: it records the session's end with the reason
`abnormalEnd`, a length taken from the session's last heartbeat (written every 5 seconds) and the
property `c3d.recovered: true`, and logs how many sessions it recovered. A marker refreshed within
`orphan_stale_after_s` (30 seconds) may belong to a process that is still running, so it is left
alone and checked again once that time has passed.

Whatever reached the spool before a crash still uploads. Records still in memory are lost: up to
about the last 10 seconds of poses, sensor series and events, and of camera and lidar what their
open chunk, block or object holds: up to about 3 seconds of a camera's frames or a laser scan's
scans, and up to 2 MiB or 5 seconds of a point cloud's clouds.

### Pausing uploads

`uploads_paused: true` records and spools as usual and sends nothing. A process configured with it
false later uploads the spool into the same sessions. It is the one parameter that applies at once
on a configured node:

```bash
ros2 param set /cognitive3d uploads_paused false
```

Use it for a run whose uplink cannot carry the data rate; [Camera and lidar](camera-and-lidar.md)
sizes the data.

## Watching a running node

### The report line

While it is active, the node logs a report every `report_period_s` (10 seconds), and publishes the
same counts on `~/diagnostics`, as a `cognitive3d: uploads` status and one `cognitive3d: <topic>`
status per topic:

```text
[INFO] [...] [cognitive3d]: observing 3 topic(s)
  session <seconds>_<your-device-id>  poses=5121 frame=map  recorded=29532 dropped=0  uploaded=38 spooled=1  events=1
  /odom  3404 msg  3404 xlat  20Hz  2393KiB  reliable/volatile/keep_last(10)
  /joint_states  8510 msg  8510 xlat  50Hz  1662KiB  reliable/volatile/keep_last(10)
  /navigate_to_pose/_action/status  14 msg  14 xlat  0Hz  2KiB  reliable/transient_local/keep_last(10)
```

The first line counts the topics the node subscribes: allowlisted, taken by type, or enrolled as
actions. `, refusing <n>` follows when it refuses some. The session line:

| field | meaning |
| -- | -- |
| `session` | The open session's id. Empty between sessions. |
| `poses=` | Pose samples recorded in this session. It rises at `sample_rate_hz` while the pose resolves. |
| `frame=` | The reference frame the poses are in: `reference_frame`, or `fallback_reference_frame` after a fallback (see [Reference frames](coordinates.md#reference-frames)). Empty until the first pose. |
| `recorded=` | Records taken into sessions since configure: poses, sensor samples, events and Dynamic Object samples. Camera frames and lidar scans are not counted here. |
| `dropped=` | Records lost before they reached the spool, since configure, as `c3d.records_dropped`. Anything above 0 is lost data; see [The size cap](#the-size-cap). |
| `uploaded=` | Chunks the platform accepted, since configure. |
| `spooled=` | Chunks in the spool now, every session's included. It returns to 0 once everything is uploaded. |
| `[UPLOADS PAUSED: credentials refused]`, `[CREDENTIALS REFUSED: <streams>]`, `[STREAMS DEFERRED: <streams>]`, `[UPLOADS PAUSED]`, `[BACKING OFF]` | The upload state, shown only when it applies. See [Outages and refused keys](#outages-and-refused-keys) and [Pausing uploads](#pausing-uploads). |
| `held=`, `deferred=` | Chunks held for another host or project, and chunks waiting behind the rest of the spool. See [Outages and refused keys](#outages-and-refused-keys). |
| `EVICTED=` | Chunks evicted at `spool_max_mb`, which never upload. See [The size cap](#the-size-cap). |
| `[<n> chunk(s) REJECTED and discarded; …]` | Chunks the platform rejected with 400, 413, 415 or 422. The log names the status of each. |
| `401retries=` | Retries scheduled after a refused upload, most of which then succeed. |
| `events=` | Events published on `~/events` that the node recorded since it started. Only these: objectives, and the events the node records from diagnostics, batteries, Nav2 and the pose timeline, are not counted. Shown once anything is counted. |
| `DROPPED=` | Events on `~/events` that were dropped, because no session was open or the name was empty. |
| `suppressed=` | Events from every source discarded by the [event rate cap](recording.md#the-event-rate-cap) this session, as `c3d.events_suppressed`. |

Then one line per topic: the messages received since configure (`msg`), the messages handed to the
topic's translator while a session was open (`xlat`, or `no-xlat` for a type with no translator,
which records nothing), the observed rate (`-Hz` until 3 messages have arrived over a second or
more), the bytes received, and the QoS the subscription requested as
`reliability/durability/history(depth)`. `[QOS INCOMPATIBLE]` marks a topic a publisher could not
connect to (see [Ingest and QoS](recording.md#ingest-and-qos)), and `REFUSED (…)` a topic the node
never subscribed. `xlat` counts messages before the translator's rate limit, not samples recorded.

Nothing on the robot counts objectives. Each action's status topic has its own line, and its `xlat`
is the status updates the node read during sessions, each of which can record one
`<action>.<state>` event (see [Objectives](recording.md#objectives)). No tool reads a stored
session's sensor series, events or objectives back yet: check them on the Cognitive3D dashboard or
in Session Replay.

Cameras and lidars have no line in the report and no status on `~/diagnostics`. The node counts
their frames and scans, but nothing on the robot shows those counts while they run. A `LaserScan`
topic that is also in `topics` has a line of its own, which counts the allowlist's subscription,
not the lidar's. While they run, the node logs `subscribed <topic> for camera:<name>` or
`lidar:<name>` once it subscribes, and warns once per session when a sensor's messages cannot be
read or carry no frame. A transform that does not resolve drops each record without a log line.
The counts reach the session every 5 seconds of session time, which under `sim_time_on_wire` is 5
simulated seconds, as the `c3d.camera.<name>.*` and `c3d.lidar.<name>.*` series, `pose_failures`
among them (see [What the session records](camera-and-lidar.md#what-the-session-records)). Before a
run, doctor's `camera/<name>` and `lidar/<name>` checks read a message from each and check its
frame.

## Running in a container or under systemd

- Bring the node up. Nothing configures and activates it unless your deployment does: the launch
  file's `autostart:=true`, a lifecycle manager, or `ros2 lifecycle set` from a script.
- Give the spool a home that outlives the process. In a container `$HOME` is usually inside the
  container's writable layer, and the spool is lost with the container: mount a volume and set
  `spool_dir` to it. Under systemd, set `spool_dir` explicitly, because a system service may have
  neither `XDG_STATE_HOME` nor `HOME`, and the node then refuses to configure. `StateDirectory=`
  gives the service a directory of its own.
- Let the stop signal reach the node. SIGINT or SIGTERM ends the session in order within a few
  seconds. Run the node in the container's main process tree and forward the signal to it: `exec`
  it from an entrypoint script, or start the container with `docker run --init`. A process started
  with `docker exec` is outside that tree, is killed outright when the container stops, and its
  session is closed as `abnormalEnd` at the next start. Allow at least 5 seconds between the stop
  signal and SIGKILL. For a robot whose stack you can reach only with `docker exec`, see
  [A robot stack in a container you cannot recreate](#a-robot-stack-in-a-container-you-cannot-recreate).
- Pass the application key in the environment. The node reads it from `C3D_APPLICATION_API_KEY`
  only ([Configuration](configuration.md)): `docker run -e`, or `Environment=` or
  `EnvironmentFile=` in the unit.
- Under systemd, start the node after the clock is synchronized. On a board without a real-time
  clock, a unit that starts at boot with `autostart:=true` otherwise finds the clock unset, and
  activation fails once with nothing to retry it, or finds it stale and dates every session wrongly
  (see [An unset wall clock](#an-unset-wall-clock)). Order the unit after `time-sync.target`:

  ```ini
  [Unit]
  Wants=time-sync.target
  After=time-sync.target
  ```

  `time-sync.target` alone does not wait for anything. Also enable the service that holds it back
  until the clock is synchronized: with `systemd-timesyncd`, Ubuntu's default,

  ```bash
  sudo systemctl enable systemd-time-wait-sync.service
  ```

  With chrony, enable `chrony-wait.service` where your distribution ships it, or a one-shot unit
  that runs `chronyc waitsync` and is ordered `Before=time-sync.target`. A robot that cannot reach
  a time server then waits at boot: give it a real-time clock, or a time server on its own network.
- Give the node a container of its own. The node is also a composable component,
  `cognitive3d_ros::Cognitive3DNode`. Load it into `component_container_isolated`, which gives each
  component its own executor, or into a container that holds nothing else. A shared
  `component_container` runs every component in it on one single-threaded executor, and the
  node's callbacks hold that executor: for up to `remote_variables_timeout_s` (3 seconds by
  default) at every session start while the remote variables are fetched, and for about 3 seconds
  at cleanup and shutdown while uploads drain. A driver or other node in the same container waits as
  long, and its topics are served late. Under `component_container_mt` the node is free of data
  races, because every callback it owns runs in one mutually exclusive group, but a lifecycle
  transition or service call can wait seconds behind ingest while the robot's topics keep the
  node saturated. On a Nav2 robot, load `cognitive3d_ros_nav2::Cognitive3DNode` instead, which
  adds the behavior tree translator (see [Nav2](recording.md#nav2)). Every node in one container
  needs its own `spool_dir`.
- Give a component the robot's namespace and tf remaps. The node's tf listener follows the
  namespace and remaps it is loaded with, so on a robot whose tree is on `/robot1/tf`, load it as
  the robot's other nodes are loaded:

  ```bash
  ros2 component load <container> cognitive3d_ros cognitive3d_ros::Cognitive3DNode \
    --node-namespace /robot1 -r /tf:=tf -r /tf_static:=tf_static
  ```

  Without the remaps, a namespaced node still reads the global `/tf`, which holds another robot's
  tree or none. Give `doctor` the same namespace and remaps.

### A robot stack in a container you cannot recreate

On some robots the whole ROS stack is one container that you can reach only with `docker exec`, and
cannot recreate with a volume, another entrypoint or an init process. The node runs there, started
with `docker exec`, as long as you take over the two things the container would otherwise do for
it.

Stop it with SIGINT to the launch process, as Ctrl+C does, and never by stopping or restarting the
container. SIGINT ends the session in order, writes its end to the spool and gives the uploads up to
2 more seconds. A process started with `docker exec` is outside the container's main process tree,
so stopping the container kills it outright: up to the last 10 seconds of records still in memory
are lost, and its session is closed as `abnormalEnd` at the next start (see
[Shutdown and crashes](#shutdown-and-crashes)). Record the launch's process id when you start it,
and signal that process to stop:

```bash
docker exec -d -e C3D_APPLICATION_API_KEY <robot-container> bash -c '
  source /opt/ros/jazzy/setup.bash      # or humble, or the robot workspace setup the stack uses
  echo $$ > /tmp/cognitive3d.pid
  exec ros2 launch cognitive3d_ros cognitive3d.launch.py \
    params_file:="$HOME/cognitive3d/params.yaml" autostart:=true > /tmp/cognitive3d.log 2>&1'

docker exec <robot-container> bash -c 'kill -INT "$(cat /tmp/cognitive3d.pid)"'
```

`-e C3D_APPLICATION_API_KEY` passes the key from the environment of the shell you run
`docker exec` in. If the key is already in the container's environment, leave out `-e`. If it is
in a file inside the container, such as the `~/cognitive3d.env` the
[Quickstart](quickstart.md#step-1-get-your-application-key) writes, leave out `-e` and load the
file in the `bash -c` script, before `exec ros2 launch`, with `set -a && . ~/cognitive3d.env && set +a`.
The container's own environment, such as `ROS_DOMAIN_ID` and the RMW, applies as it does to the
robot's nodes. The node logs to `/tmp/cognitive3d.log`, and if anything is still spooled when it
stops, one line there says how many chunks are left.

Point `spool_dir` at a directory that already persists. The default spool, under the container
user's home, is in the container's writable layer: it survives a restart of the same container, and
is lost, with every session not yet uploaded, when the container is replaced, as an update of the
robot's software often does. If the container already mounts a volume or a host directory,
`docker inspect --format '{{json .Mounts}}' <robot-container>` lists them on the machine that runs
Docker. From a shell inside the container, `findmnt` lists them:

```bash
findmnt -o TARGET,FSTYPE,OPTIONS -t nooverlay,notmpfs,noproc,nosysfs,nocgroup,nocgroup2,nodevpts,nomqueue
```

A directory it lists with `rw` options is a volume or a host directory; `/etc/hosts`,
`/etc/hostname` and `/etc/resolv.conf` are files Docker mounts itself. Set `spool_dir` to a
directory of its own on such a mount.

If none persists, `spool_dir` gains nothing: every directory in the container's writable layer, the
default included, lasts exactly as long as the container. Setting it anyway puts the spool where
you can name it. Then let the spool empty before anything replaces the container. End the session
with `~/end_session`, or, for the session `autostart:=true` opened, with a deactivate, which keeps
uploading what the spool holds:

```bash
docker exec <robot-container> bash -c '
  source /opt/ros/jazzy/setup.bash      # as for the launch above
  ros2 lifecycle set /cognitive3d deactivate'
```

Then wait for the node to log `spool drained: … none remaining`, or, for a session ended with
`~/end_session`, for `spooled=0` on the [report line](#the-report-line), before you stop it.

## Robots on a shared network

ROS 2 discovery reaches every machine on the subnet by default, so two robots on one network join
one ROS graph, and each node records the other robot's tf, batteries, diagnostics and actions along
with its own. Give each robot its own `ROS_DOMAIN_ID`, a value from 0 to 101, exported in the
environment of every ROS process on that robot:

```bash
export ROS_DOMAIN_ID=17
```

Do not use `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` for isolation. Humble does not have the
variable, and on Jazzy localhost-only discovery caps how many participants one host discovers, so a
large graph such as a Nav2 bringup goes partly blind. Every session records `c3d.ros.domain_id`,
and on Jazzy `c3d.ros.discovery_range`, so a stored session says which graph it came from.
