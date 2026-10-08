# Remote variables

Remote variables and A/B tests let the Cognitive3D dashboard choose a value for each session, such
as a navigation speed or a behavior variant, and slice sessions by it afterwards. The node fetches
a session's values once, as the session opens, records each as a session property, and hands them
to whoever started the session. Making the robot act on them is your code's job.

## Set up a test

Create the remote variable in the Cognitive3D dashboard's Remote Controls, then an A/B test on it.
Set them up with these platform behaviors in mind:

- Set the variable's own probability, `remoteVariableProbability`, to 0. It is a weight, not an
  enrollment rate: the variable's own value competes in every draw as one more arm with that weight.
  At 100 beside two arms at 50/50, the variable's value takes half of all draws, and the split is
  not what the dashboard shows.
- A value is an integer, a boolean or a string. There are no floating-point remote variables, so
  carry a speed as an integer in a unit the name states: `nav_speed_centimeters_per_second`.
- A variable's name is at most 32 characters.
- Give the variable a base value outside every arm, such as `0` for a speed, so a session in no
  arm reads as not enrolled rather than as one of the arms.

## What the node does at session start

With `remote_variables: true`, the default, every session start does the following:

1. The node asks the platform for the project's remote variables and A/B arms, with the application
   key, for the session's [draw identifier](#the-draw-identifier).
2. It records each resolved variable as the typed session property `c3d.remote_variable.<name>`,
   which the dashboard slices sessions by.
3. It returns them in the `~/start_session` response's `remote_variables_json`, a flat JSON object
   keyed by variable name: `{"nav_speed_centimeters_per_second":30}`. `{}` means nothing is
   configured for this identifier.

The platform returns only each variable's resolved value, never the name of the arm.

When the fetch fails, the session still opens, `remote_variables_json` is empty, the node warns,
and the session carries a `c3d.remote_variables.unavailable` event with `identifier`, `status` and
`error`. Status `0` is no connection, `401` a refused application key, and `200` with an error
naming `cvr-request-time` an answer from something other than the Cognitive3D gateway, such as a
captive portal.

The fetch holds the node up. `~/start_session`, or activation with `auto_start_session: true`,
waits for it for up to `remote_variables_timeout_s` (3 seconds by default, rounded up to whole
seconds between 1 and 60), and the node's other callbacks wait with it. On a slow uplink, every
session therefore starts with a gap of up to that long in its poses and sensor series.

When a clock jump reopens a session ([Clock jumps](sessions.md#clock-jumps)), the new session
keeps the arms of the one it replaces, without fetching again.

## Applying the arm

The node records the arm; it never changes how the robot behaves. Start the session through
`~/start_session`, read `remote_variables_json` from the response, and apply it before the robot
starts the task:

```bash
ID=$(uuidgen)
ros2 service call /cognitive3d/start_session cognitive3d_ros_msgs/srv/StartSession \
  "{session_name: 'patrol/$ID', participant_id: '$ID'}"
```

The response carries the arm, for example `remote_variables_json: '{"nav_speed_centimeters_per_second":30}'`.
With `auto_start_session: true` there is no response to read: the arms reach the session and the
node log, but nothing hands them to your code. Set `auto_start_session: false` to act on arms; see
[Automatic sessions](sessions.md#automatic-sessions).

## Choosing the participant id

The `participant_id` in `~/start_session` is the identifier arms are drawn for, and the session's
participant, `c3d.participant.id`. In a sticky test the draw is deterministic: one identifier draws
the same arm every time, and the platform stores nothing.

- Pass a new UUID for every run, so each run gets a draw of its own.
- Left empty, it falls back to the node's `device_id`, which keeps one robot in one arm of a sticky
  test for good: right for a test across a fleet, wrong for comparing runs of one robot.
- It is at most 64 bytes. A longer one is refused before any session opens.

## The draw identifier

The node never sends the participant id itself. It sends the SHA-256 digest of the participant id's
exact bytes, written as 64 lowercase hexadecimal characters, while the session records the
participant id itself as `c3d.participant.id`. Sent raw, near-identical identifiers such as
`robot-01` and `robot-02`, or `run-001` and `run-002`, tend to draw the same arm; their digests
spread across the arms.

Another client that must draw the same arm for the same identifier sends the same digest:

```bash
printf '%s' 'robot-01' | sha256sum
```

A client that sends the raw identifier draws from a different seed, so a robot and, say, a Unity
application given the same identifier are not guaranteed the same arm.

## Two platform behaviors to plan around

- Two sticky tests in one project are correlated for the same identifier: both draws are seeded by
  the identifier alone, so one test's arm predicts the other's. Run one sticky test per project at
  a time, or account for the correlation in the analysis.
- A disabled test is indistinguishable from no test. It is left out of the response, and the robot
  keeps its own default, exactly as for a project with nothing configured. A successful fetch is
  not proof a test is running: check that `remote_variables_json` names the variables you expect.

## Turning the fetch off

`remote_variables: false` skips the fetch: no request, no event, an empty `remote_variables_json`,
and no `c3d.remote_variable.*` property from the node. `remote_variables_timeout_s` is only the
fetch's timeout. It must be positive, and `0` refuses configure rather than turning the fetch off.

While the fetch is on, the node owns the `c3d.remote_variable.*` properties: `~/start_session`
refuses a `properties_json` that names one, and a `session_properties` entry that names one
refuses configure, naming the key. With the fetch off, you can set them yourself, for example for
arms your own code draws. Every other key in `c3d.` is the SDK's whether the fetch is on or off.
