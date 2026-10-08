# A named event for the Cognitive3D session, published by any node on the
# Cognitive3D node's `~/events` topic (`/cognitive3d/events` under the shipped
# launch file).
#
# The node derives events from the robot's own topics: diagnostics transitions,
# battery changes, action outcomes, Nav2 behavior tree recoveries. Publish this
# for the facts no topic carries: a task's own state, a mission outcome, an
# operator decision. The node places the event where the robot is and, unless
# `stamp` says otherwise, when it arrives; the publisher supplies the meaning.
#
# The subscription is reliable, so publish with reliable QoS: a best-effort
# publisher does not connect. An event published while no session is open is
# dropped and counted. One published while the node is inactive is lost,
# because deactivating destroys the subscription. Every publisher on this topic
# shares one budget of `event_rate_limit_hz` events per second, and the excess
# is discarded and counted in `c3d.events_suppressed`.

# Required. Dotted and namespaced by convention, e.g. mission.waypoint_reached.
# An event with an empty name is dropped.
string name

# Optional. A flat JSON object of scalars as text, or empty for none. Each
# value keeps its own JSON type, because the platform indexes numeric
# properties separately from text: a number sent as a string is invisible to a
# numeric filter. Spell units out in full words in property names:
# meters_per_second, not m/s. Anything that is not a flat object of scalars is
# rejected, and the event is recorded without properties. A key in the `c3d.`
# namespace is the SDK's own: it is dropped, and counted in the session's
# `c3d.event_properties_dropped`.
string properties_json

# Optional. The id of a Dynamic Object recorded in this session: the robot's,
# `robot_object_id`, when `robot_dynamic_object` is true. Empty attaches the
# event to the session rather than to an object.
string dynamic_object_id

# Optional. When the event happened, on the publisher's ROS clock -- the clock a
# message header stamp is on. Zero, the default, means when the node receives
# it. A stamp is placed on the wire the way the node places header stamps: as
# an age against the node's own ROS clock, so ROS time never reaches the wire
# and a simulation's stamp near zero lands in the right year. A stamp more than
# an hour old, or more than a second ahead, is taken as now, and one from before
# the session began is placed at the session's start, with a warning once per
# session. The position is always where the robot is when the node receives the
# event, not where it was at the stamp.
builtin_interfaces/Time stamp
