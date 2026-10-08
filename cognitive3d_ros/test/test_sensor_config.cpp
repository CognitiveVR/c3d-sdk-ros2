// A described sensor's session properties and its refusals, without a node. What reaches the
// wire from a real node, and the compatibility contract with sessions already stored, are pinned
// in test_session_control.cpp.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <map>
#include <string>

#include "cognitive3d_ros/sensor_config.hpp"

using cognitive3d_ros::SensorConfig;
using c3d::PropertyValue;

namespace {

std::map<std::string, PropertyValue> AsMap(const c3d::Properties& properties) {
  std::map<std::string, PropertyValue> out;
  for (const auto& kv : properties) {
    EXPECT_TRUE(out.emplace(kv.first, kv.second).second) << kv.first << " written twice";
  }
  return out;
}

SensorConfig Cone() {
  SensorConfig sensor;
  sensor.name = "sonar";
  sensor.kind = "cone_rangefinder";
  sensor.topic = "/sonar";
  sensor.series = {"sonar.value"};
  sensor.cone_half_angle_deg = 15.0;
  sensor.units = "meters";
  return sensor;
}

SensorConfig Binary() {
  SensorConfig sensor;
  sensor.name = "bumpers";
  sensor.kind = "binary_array";
  sensor.topic = "/bumpers";
  sensor.series = {"bumpers.0", "bumpers.1"};
  sensor.element_count = 2;
  return sensor;
}

}  // namespace

// What is unset is not recorded: a reader treats an absent range, mount or meaning as unknown,
// and a recorded default would read as a measurement.
TEST(SensorSessionProperties, AnUnsetOptionalFieldWritesNoKey) {
  const auto cone = AsMap(cognitive3d_ros::SensorSessionProperties(Cone()));
  EXPECT_EQ(cone.size(), 6u);
  for (const char* key : {"kind", "topic", "series", "units", "cone_half_angle_degrees",
                          "mount_measured"}) {
    EXPECT_EQ(cone.count(std::string("c3d.ros.sensor.sonar.") + key), 1u) << key;
  }
  EXPECT_EQ(cone.at("c3d.ros.sensor.sonar.series").as_string(), "sonar.value");
  EXPECT_FALSE(cone.at("c3d.ros.sensor.sonar.mount_measured").as_bool());

  const auto binary = AsMap(cognitive3d_ros::SensorSessionProperties(Binary()));
  EXPECT_EQ(binary.size(), 5u);
  for (const char* key : {"kind", "topic", "series", "element_count", "mount_measured"}) {
    EXPECT_EQ(binary.count(std::string("c3d.ros.sensor.bumpers.") + key), 1u) << key;
  }
  EXPECT_EQ(binary.at("c3d.ros.sensor.bumpers.series").as_string(), "bumpers.0,bumpers.1");
  EXPECT_EQ(binary.at("c3d.ros.sensor.bumpers.element_count").kind(), PropertyValue::Kind::kInt);
}

// A number whole in the property's unit is an integer, as the same text in a `key=value` entry
// is typed, and any other a double; the yaw is always a double. The mount is `x,y,z` text with two
// to five decimals.
TEST(SensorSessionProperties, NumbersAreTypedAsTheirTextWouldBeAndTheMountIsText) {
  SensorConfig sensor = Cone();
  sensor.cone_half_angle_deg = 7.5;
  sensor.range_min_m = 0.07;  // 7.000000000000001 cm before rounding
  sensor.range_max_m = 0.4525;
  sensor.mount_xyz_m = {0.1, -0.0000001, 1.234567};
  sensor.mount_yaw_deg = 90.0;
  sensor.frame = "sonar_link";
  sensor.mount_parent = "base_link";
  sensor.mount_measured = true;
  const auto properties = AsMap(cognitive3d_ros::SensorSessionProperties(sensor));
  const auto at = [&properties](const std::string& key) {
    return properties.at("c3d.ros.sensor.sonar." + key);
  };
  EXPECT_EQ(at("cone_half_angle_degrees").kind(), PropertyValue::Kind::kDouble);
  EXPECT_DOUBLE_EQ(at("cone_half_angle_degrees").as_double(), 7.5);
  EXPECT_EQ(at("range_min_centimeters").kind(), PropertyValue::Kind::kInt);
  EXPECT_EQ(at("range_min_centimeters").as_int(), 7);
  EXPECT_EQ(at("range_max_centimeters").kind(), PropertyValue::Kind::kDouble);
  EXPECT_DOUBLE_EQ(at("range_max_centimeters").as_double(), 45.25);
  EXPECT_EQ(at("mount_xyz_meters").as_string(), "0.10,0.00,1.23457");
  EXPECT_EQ(at("mount_yaw_radians").kind(), PropertyValue::Kind::kDouble);
  EXPECT_DOUBLE_EQ(at("mount_yaw_radians").as_double(), 1.5708);
  EXPECT_EQ(at("frame").as_string(), "sonar_link");
  EXPECT_EQ(at("mount_parent").as_string(), "base_link");
  EXPECT_TRUE(at("mount_measured").as_bool());

  sensor.mount_yaw_deg = 0.0;
  const auto zero = AsMap(cognitive3d_ros::SensorSessionProperties(sensor));
  EXPECT_EQ(zero.at("c3d.ros.sensor.sonar.mount_yaw_radians").kind(),
            PropertyValue::Kind::kDouble);
}

TEST(ValidateSensorConfig, EachRefusalNamesTheParameter) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(cognitive3d_ros::ValidateSensorConfig(Cone()), "");
  EXPECT_EQ(cognitive3d_ros::ValidateSensorConfig(Binary()), "");
  struct Case {
    SensorConfig sensor;
    std::string names;
  };
  std::vector<Case> cases;
  const auto cone = [&cases](const std::string& names, auto change) {
    SensorConfig sensor = Cone();
    change(sensor);
    cases.push_back({sensor, names});
  };
  const auto binary = [&cases](const std::string& names, auto change) {
    SensorConfig sensor = Binary();
    change(sensor);
    cases.push_back({sensor, names});
  };
  cone("sensor.sonar.kind", [](SensorConfig& s) { s.kind.clear(); });
  cone("sensor.sonar.kind", [](SensorConfig& s) { s.kind = "rangefinder"; });
  cone("sensor.sonar.topic", [](SensorConfig& s) { s.topic.clear(); });
  cone("sensor.sonar.cone_half_angle_deg", [](SensorConfig& s) { s.cone_half_angle_deg = 0.0; });
  cone("sensor.sonar.cone_half_angle_deg", [](SensorConfig& s) { s.cone_half_angle_deg = 90.0; });
  cone("sensor.sonar.cone_half_angle_deg", [nan](SensorConfig& s) { s.cone_half_angle_deg = nan; });
  cone("sensor.sonar.units", [](SensorConfig& s) { s.units.clear(); });
  cone("sensor.sonar.units", [](SensorConfig& s) { s.units = "inches"; });
  cone("sensor.sonar.range_min_m", [](SensorConfig& s) { s.range_min_m = -0.1; });
  cone("sensor.sonar.range_max_m", [nan](SensorConfig& s) { s.range_max_m = nan; });
  cone("sensor.sonar.range_min_m", [](SensorConfig& s) {
    s.range_min_m = 4.0;
    s.range_max_m = 4.0;
  });
  cone("sensor.sonar.mount_xyz_m", [](SensorConfig& s) { s.mount_xyz_m = {0.1, 0.2}; });
  cone("sensor.sonar.mount_xyz_m", [nan](SensorConfig& s) { s.mount_xyz_m = {0.1, nan, 0.0}; });
  cone("sensor.sonar.mount_yaw_deg", [](SensorConfig& s) { s.mount_yaw_deg = 10.0; });
  cone("sensor.sonar.element_count", [](SensorConfig& s) { s.element_count = 4; });
  cone("sensor.sonar.meaning_1", [](SensorConfig& s) { s.meaning_1 = "blocked"; });
  binary("sensor.bumpers.element_count", [](SensorConfig& s) { s.element_count = 0; });
  binary("sensor.bumpers.units", [](SensorConfig& s) { s.units = "cm"; });
  binary("sensor.bumpers.cone_half_angle_deg", [](SensorConfig& s) { s.cone_half_angle_deg = 5; });
  binary("sensor.bumpers.mount_xyz_m", [](SensorConfig& s) { s.mount_xyz_m = {0.0, 0.0, 0.0}; });
  for (const auto& c : cases) {
    const std::string refusal = cognitive3d_ros::ValidateSensorConfig(c.sensor);
    EXPECT_NE(refusal.find(c.names), std::string::npos) << c.names << ": '" << refusal << "'";
  }
  // Every unit a reader converts from.
  for (const char* unit : {"meters", "metres", "m", "centimeters", "centimetres", "cm",
                           "millimeters", "millimetres", "mm"}) {
    SensorConfig sensor = Cone();
    sensor.units = unit;
    EXPECT_EQ(cognitive3d_ros::ValidateSensorConfig(sensor), "") << unit;
  }
}
