#include "cognitive3d_ros/sensor_config.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>

namespace cognitive3d_ros {
namespace {

constexpr double kPi = 3.14159265358979323846;

bool Finite(double v) { return std::isfinite(v); }

// Five decimals, the precision every number on the wire is rounded to.
double Round5(double v) {
  const double rounded = std::round(v * 1e5) / 1e5;
  return rounded == 0.0 ? 0.0 : rounded;  // never -0
}

// Whole in its unit: an integer, as `2` in a `key=value` entry is. Otherwise a double.
c3d::PropertyValue Number(double v) {
  const double rounded = Round5(v);
  if (std::fabs(rounded) < 9.0e15 && rounded == std::floor(rounded)) {
    return c3d::PropertyValue(static_cast<std::int64_t>(rounded));
  }
  return c3d::PropertyValue(rounded);
}

// At least two decimals, as `0.10` is written, and no more than the wire's five.
std::string Metres(double v) {
  char text[64];
  std::snprintf(text, sizeof(text), "%.5f", Round5(v));
  std::string out(text);
  const std::size_t point = out.find('.');
  while (out.size() > point + 3 && out.back() == '0') out.pop_back();
  return out;
}

}  // namespace

const std::vector<std::string>& SensorUnits() {
  static const std::vector<std::string> kUnits = {
      "meters", "metres", "m", "centimeters", "centimetres", "cm",
      "millimeters", "millimetres", "mm"};
  return kUnits;
}

std::string SensorMessageType(const std::string& kind) {
  if (kind == kConeRangefinder) return "std_msgs/msg/Float32";
  if (kind == kBinaryArray) return "std_msgs/msg/Int32MultiArray";
  return "";
}

std::string SensorTypeMismatch(const SensorConfig& sensor, const std::string& graph_type) {
  const std::string expected = SensorMessageType(sensor.kind);
  if (graph_type.empty() || expected.empty() || graph_type == expected) return "";
  return "sensor." + sensor.name + ".kind is " + sensor.kind + ", whose readings are " + expected +
         ", and sensor." + sensor.name + ".topic " + sensor.topic + " is " + graph_type +
         " on the graph: the series the description names is never recorded";
}

std::string SensorTypeRemedy(const SensorConfig& sensor) {
  return "Point sensor." + sensor.name + ".topic at a " + SensorMessageType(sensor.kind) +
         " topic, or correct sensor." + sensor.name + ".kind";
}

std::string ValidateSensorConfig(const SensorConfig& sensor) {
  const std::string where = "sensor." + sensor.name + ".";
  if (sensor.kind.empty()) {
    return where + "kind is unset: a name in `sensors` must say what it describes, '" +
           kConeRangefinder + "' or '" + kBinaryArray + "'";
  }
  const bool cone = sensor.kind == kConeRangefinder;
  const bool binary = sensor.kind == kBinaryArray;
  if (!cone && !binary) {
    return where + "kind must be '" + kConeRangefinder + "' or '" + kBinaryArray + "', got '" +
           sensor.kind + "'";
  }
  if (sensor.topic.empty()) {
    return where + "topic is unset: the sensor's readings are the series its topic records, and "
                   "without one the description names no series";
  }

  // A value set for the other kind is read by nothing.
  const auto other_kind = [&](const std::string& field) {
    return where + field + " applies to a " + (cone ? kBinaryArray : kConeRangefinder) +
           " sensor only, and this one is a " + sensor.kind;
  };

  if (cone) {
    if (sensor.element_count != 0) return other_kind("element_count");
    if (!sensor.meaning_0.empty()) return other_kind("meaning_0");
    if (!sensor.meaning_1.empty()) return other_kind("meaning_1");
    if (!Finite(sensor.cone_half_angle_deg) || sensor.cone_half_angle_deg <= 0.0 ||
        sensor.cone_half_angle_deg >= 90.0) {
      return where + "cone_half_angle_deg must be above 0 and below 90 degrees: it is half the "
                     "cone's full angle, and a reader draws no cone without it";
    }
    bool known_unit = false;
    std::string accepted;
    for (const auto& unit : SensorUnits()) {
      known_unit = known_unit || sensor.units == unit;
      accepted += (accepted.empty() ? "'" : ", '") + unit + "'";
    }
    if (!known_unit) {
      return where + "units must be the unit the topic publishes in, one of " + accepted +
             ", got '" + sensor.units + "'";
    }
    for (const auto& [field, value] :
         {std::pair<const char*, double>{"range_min_m", sensor.range_min_m},
          std::pair<const char*, double>{"range_max_m", sensor.range_max_m}}) {
      if (!Finite(value) || value < 0.0) {
        return where + field + " must be 0 (not recorded) or a positive length in metres";
      }
    }
    if (sensor.range_min_m > 0.0 && sensor.range_max_m > 0.0 &&
        sensor.range_min_m >= sensor.range_max_m) {
      return where + "range_min_m must be below range_max_m";
    }
    if (!sensor.mount_xyz_m.empty()) {
      bool finite = sensor.mount_xyz_m.size() == 3;
      for (const double v : sensor.mount_xyz_m) finite = finite && Finite(v);
      if (!finite) {
        return where + "mount_xyz_m must be empty (not recorded) or three finite numbers, x, y "
                       "and z in metres";
      }
    }
    if (!Finite(sensor.mount_yaw_deg)) return where + "mount_yaw_deg must be a finite number";
    if (sensor.mount_yaw_deg != 0.0 && sensor.mount_xyz_m.empty()) {
      return where + "mount_yaw_deg is set without mount_xyz_m: the yaw is recorded as part of "
                     "the mount, and a reader would place the sensor at the parent's origin";
    }
  } else {
    if (sensor.cone_half_angle_deg != 0.0) return other_kind("cone_half_angle_deg");
    if (!sensor.units.empty()) return other_kind("units");
    if (sensor.range_min_m != 0.0) return other_kind("range_min_m");
    if (sensor.range_max_m != 0.0) return other_kind("range_max_m");
    if (!sensor.mount_xyz_m.empty()) return other_kind("mount_xyz_m");
    if (sensor.mount_yaw_deg != 0.0) return other_kind("mount_yaw_deg");
    if (sensor.element_count <= 0) {
      return where + "element_count must be a positive number of elements";
    }
  }
  return "";
}

c3d::Properties SensorSessionProperties(const SensorConfig& sensor) {
  const std::string prefix = "c3d.ros.sensor." + sensor.name + ".";
  c3d::Properties out;
  const auto put = [&](const std::string& key, c3d::PropertyValue value) {
    out.emplace_back(prefix + key, std::move(value));
  };
  std::string series;
  for (const auto& name : sensor.series) series += (series.empty() ? "" : ",") + name;

  put("kind", sensor.kind);
  put("topic", sensor.topic);
  put("series", series);
  if (sensor.kind == kConeRangefinder) {
    put("units", sensor.units);
    put("cone_half_angle_degrees", Number(sensor.cone_half_angle_deg));
    if (sensor.range_min_m > 0.0) put("range_min_centimeters", Number(sensor.range_min_m * 100.0));
    if (sensor.range_max_m > 0.0) put("range_max_centimeters", Number(sensor.range_max_m * 100.0));
    if (sensor.mount_xyz_m.size() == 3) {
      put("mount_xyz_meters", Metres(sensor.mount_xyz_m[0]) + "," + Metres(sensor.mount_xyz_m[1]) +
                                  "," + Metres(sensor.mount_xyz_m[2]));
      put("mount_yaw_radians", Round5(sensor.mount_yaw_deg * kPi / 180.0));
    }
  } else {
    put("element_count", static_cast<std::int64_t>(sensor.element_count));
    if (!sensor.meaning_0.empty()) put("meaning_0", sensor.meaning_0);
    if (!sensor.meaning_1.empty()) put("meaning_1", sensor.meaning_1);
  }
  if (!sensor.frame.empty()) put("frame", sensor.frame);
  if (!sensor.mount_parent.empty()) put("mount_parent", sensor.mount_parent);
  put("mount_measured", sensor.mount_measured);
  return out;
}

}  // namespace cognitive3d_ros
