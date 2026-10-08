#pragma once

#include <string>
#include <vector>

#include "c3d/types.hpp"

namespace cognitive3d_ros {

// One described sensor: the fixed facts about a sensor whose readings ride a plain series -- what
// kind of sensor it is, which series carries it, how it is mounted -- recorded once per session as
// `c3d.ros.sensor.<name>.*` properties, because nothing in a Float32 or an Int32MultiArray says any
// of it and Session Replay draws the sensor from them.
//
// The NAME is the property segment a reader keys on, so it is the operator's label rather than
// the topic.
struct SensorConfig {
  std::string name;
  // `cone_rangefinder` or `binary_array`; anything else is refused.
  std::string kind;
  // Resolved against the node's namespace, as a camera's is.
  std::string topic;
  // The series the topic's readings are recorded under, worked out at configure by the same rule
  // the topic's translator names them with. Never a parameter: a series typed by hand is a series
  // that drifts from the one recorded. For a binary array, every element's series, in order.
  std::vector<std::string> series;

  // Both kinds. `frame` is the sensor's own tf frame and `mount_parent` the frame its mount is
  // measured from; empty records neither.
  std::string frame;
  std::string mount_parent;
  // Whether the mount was measured rather than estimated. A reader treats an absent flag as no.
  bool mount_measured = false;

  // cone_rangefinder. The half-angle of the cone, in degrees, above 0 and below 90.
  double cone_half_angle_deg = 0.0;
  // The unit the series values are in, as published: the translator records them unconverted.
  std::string units;
  // 0 records no limit.
  double range_min_m = 0.0;
  double range_max_m = 0.0;
  // Empty records no mount position; otherwise x, y, z in metres, in `mount_parent`.
  std::vector<double> mount_xyz_m;
  // The mount's yaw in `mount_parent`, recorded only with a mount position.
  double mount_yaw_deg = 0.0;

  // binary_array. The number of one-bit elements, each its own series.
  int element_count = 0;
  // What a 0 and a 1 mean, recorded verbatim; empty records nothing.
  std::string meaning_0;
  std::string meaning_1;
};

// The two kinds a reader knows how to draw.
inline constexpr const char* kConeRangefinder = "cone_rangefinder";
inline constexpr const char* kBinaryArray = "binary_array";

// The units a reader converts a cone rangefinder's series from.
const std::vector<std::string>& SensorUnits();

// The message type a sensor of `kind` publishes its readings as: std_msgs/msg/Float32 for a cone
// rangefinder, std_msgs/msg/Int32MultiArray for a binary array. Empty for any other kind.
std::string SensorMessageType(const std::string& kind);

// Empty when the graph's `graph_type` for the sensor's topic is the one its kind reads, or is not
// known yet. Otherwise why not, naming `sensor.<name>.kind` and `.topic` and both types: the
// topic's translator then records under other series names, or none, and the description names a
// series that never exists. SensorTypeRemedy is what to do about it.
std::string SensorTypeMismatch(const SensorConfig& sensor, const std::string& graph_type);
std::string SensorTypeRemedy(const SensorConfig& sensor);

// Empty when the configuration is usable, otherwise the reason, naming the parameter. A field of
// the other kind left set is refused too: it would be read by nothing while the run looks
// configured. `series` is not checked; the node works it out.
std::string ValidateSensorConfig(const SensorConfig& sensor);

// The session properties describing `sensor`, under `c3d.ros.sensor.<name>.`.
//
// The keys and the value formats are a contract with readers and with sessions already stored, so
// a number that is whole in the property's unit is an integer and any other a double -- what the
// same text given as a `key=value` session property is typed as -- except `mount_yaw_radians`,
// which is always a double. `mount_xyz_meters` is text, `x,y,z` with two to five decimals each.
c3d::Properties SensorSessionProperties(const SensorConfig& sensor);

}  // namespace cognitive3d_ros
