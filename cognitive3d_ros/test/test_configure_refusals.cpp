// Values configure refuses by name, each driven through a real node's configure, because a pure
// validator that nothing calls refuses nothing.
//
// The nodes are dry runs, so nothing is spooled, and one node serves most cases: a value is set,
// configure is asked, and the value is put back. Each node is a DDS participant, and a binary
// that constructs one per case runs past its timeout.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdlib>
#include <limits>
#include <map>
#include <set>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "cognitive3d_ros/node.hpp"

using cognitive3d_ros::Cognitive3DNode;

namespace {

// Configure refuses a scene_id that is not a UUID.
constexpr const char kSceneId[] = "a1b2c3d4-0000-4000-8000-000000000001";

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

std::vector<rclcpp::Parameter> DryRun(std::vector<rclcpp::Parameter> extra = {}) {
  std::vector<rclcpp::Parameter> overrides = {
      {"scene_id", std::string(kSceneId)},
      {"scene_version", std::string("1")},
      {"pose_convention", std::string("unity")},
      {"device_id", std::string("robot")},
      {"dry_run", true},
  };
  overrides.insert(overrides.end(), extra.begin(), extra.end());
  return overrides;
}

std::shared_ptr<Cognitive3DNode> MakeNode(const std::vector<rclcpp::Parameter>& overrides,
                                          const std::string& node_namespace = "") {
  rclcpp::NodeOptions options;
  options.parameter_overrides(overrides);
  if (!node_namespace.empty()) options.arguments({"--ros-args", "-r", "__ns:=" + node_namespace});
  return std::make_shared<Cognitive3DNode>(options);
}

// What configure refused with, or empty when it configured.
std::string Configure(Cognitive3DNode& node) {
  if (node.configure().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    node.cleanup();
    return "";
  }
  return node.configure_error().empty() ? "(refused with no reason)" : node.configure_error();
}

// The parameters the shared node below is made with.
std::vector<rclcpp::Parameter> SharedNodeParameters() {
  return DryRun({
      {"topics", std::vector<std::string>{"/ultrasonic", "/line_sensor"}},
      {"topic_config.line_sensor.series", std::string("line")},
      {"cameras", std::vector<std::string>{"front"}},
      {"camera.front.topic", std::string("/image")},
      {"sensors", std::vector<std::string>{"sonar", "line"}},
      {"sensor.sonar.kind", std::string("cone_rangefinder")},
      {"sensor.sonar.topic", std::string("/ultrasonic")},
      {"sensor.sonar.cone_half_angle_deg", 7.5},
      {"sensor.sonar.units", std::string("centimeters")},
      {"sensor.line.kind", std::string("binary_array")},
      {"sensor.line.topic", std::string("/line_sensor")},
      {"sensor.line.element_count", 4},
  });
}

class ConfigureRefusals : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    node_ = MakeNode(SharedNodeParameters());
    // The per-topic and per-camera keys are declared by the first configure.
    ASSERT_EQ(Configure(*node_), "");
  }
  static void TearDownTestSuite() { node_.reset(); }

  // What configure refuses with while `bad` is set; the old value is put back afterwards.
  static std::string RefusalWith(const rclcpp::Parameter& bad) {
    const rclcpp::Parameter before = node_->get_parameter(bad.get_name());
    EXPECT_TRUE(node_->set_parameter(bad).successful) << bad.get_name();
    const std::string refusal = Configure(*node_);
    EXPECT_TRUE(node_->set_parameter(before).successful) << bad.get_name();
    return refusal;
  }

  static std::shared_ptr<Cognitive3DNode> node_;
};
std::shared_ptr<Cognitive3DNode> ConfigureRefusals::node_;

}  // namespace

// A misspelt override would otherwise be ignored while the subscription still reported its profile
// as overridden by configuration.
TEST_F(ConfigureRefusals, AQosOverrideOutsideItsValuesIsRefusedNamingTheParameter) {
  struct Case {
    rclcpp::Parameter bad;
    std::string accepted;
  };
  const std::vector<Case> cases = {
      {{"topic_config.ultrasonic.qos.reliability", std::string("best-effort")},
       "'reliable' or 'best_effort'"},
      {{"topic_config.ultrasonic.qos.durability", std::string("VOLATILE")},
       "'volatile' or 'transient_local'"},
      {{"topic_config.ultrasonic.qos.history", std::string("keep_some")},
       "'keep_last' or 'keep_all'"},
      {{"topic_config.ultrasonic.qos.depth", 0}, "1 or more"},
      {{"camera.front.qos.reliability", std::string("BEST_EFFORT")},
       "'reliable' or 'best_effort'"},
      {{"camera.front.qos.durability", std::string("latched")},
       "'volatile' or 'transient_local'"},
  };
  for (const auto& c : cases) {
    const std::string refusal = RefusalWith(c.bad);
    EXPECT_NE(refusal.find(c.bad.get_name()), std::string::npos) << refusal;
    EXPECT_NE(refusal.find(c.accepted), std::string::npos) << refusal;
  }
  EXPECT_EQ(RefusalWith({"topic_config.ultrasonic.qos.reliability", std::string("best_effort")}),
            "");
  EXPECT_EQ(RefusalWith({"camera.front.qos.durability", std::string("transient_local")}), "");
}

// Each of these compares against a sample or a clock, where a negative value or NaN either flags
// every sample or none.
TEST_F(ConfigureRefusals, ThresholdsAndIntervalsOutsideTheirRangeAreRefusedByName) {
  const std::vector<rclcpp::Parameter> refused = {
      {"relocalisation_jump_m", -1.0},       {"relocalisation_jump_m", 0.0},
      {"relocalisation_jump_m", kNaN},       {"relocalisation_jump_deg", -45.0},
      {"relocalisation_jump_deg", kNaN},     {"reference_frame_grace_s", -1.0},
      {"reference_frame_grace_s", kNaN},     {"camera_flush_s", kNaN},
      {"lidar_flush_s", kNaN},
  };
  for (const auto& bad : refused) {
    const std::string refusal = RefusalWith(bad);
    EXPECT_NE(refusal.find(bad.get_name()), std::string::npos)
        << bad.get_name() << " = " << bad.value_to_string() << ": " << refusal;
  }
  EXPECT_EQ(RefusalWith({"reference_frame_grace_s", 0.0}), "")
      << "no grace is a choice: the fallback is adopted as soon as it resolves";
}

// A camera's field of view is two full angles or none. One angle alone is half a frustum, and an
// angle of 0 or less, 180 or more, or not a number is not a camera's: each is refused naming the
// parameter rather than sent as a frustum nobody measured, or silently as none.
TEST_F(ConfigureRefusals, ACameraFieldOfViewIsBothAnglesUnder180OrNeither) {
  const auto with = [](double h, double v) {
    const rclcpp::Parameter h_before = node_->get_parameter("camera.front.hfov_degrees");
    const rclcpp::Parameter v_before = node_->get_parameter("camera.front.vfov_degrees");
    EXPECT_TRUE(node_->set_parameter({"camera.front.hfov_degrees", h}).successful);
    EXPECT_TRUE(node_->set_parameter({"camera.front.vfov_degrees", v}).successful);
    const std::string refusal = Configure(*node_);
    EXPECT_TRUE(node_->set_parameter(h_before).successful);
    EXPECT_TRUE(node_->set_parameter(v_before).successful);
    return refusal;
  };
  EXPECT_EQ(with(0.0, 0.0), "") << "unset is the default";
  EXPECT_EQ(with(61.0, 45.0), "");
  EXPECT_EQ(with(179.9, 0.1), "");

  struct Case {
    double h, v;
    std::string names;
  };
  const double inf = std::numeric_limits<double>::infinity();
  const std::vector<Case> cases = {
      {61.0, 0.0, "only hfov_degrees"},
      {0.0, 45.0, "only vfov_degrees"},
      {180.0, 45.0, "camera.front.hfov_degrees"},
      {61.0, 180.0, "camera.front.vfov_degrees"},
      {-61.0, 45.0, "camera.front.hfov_degrees"},
      {61.0, -45.0, "camera.front.vfov_degrees"},
      {kNaN, 45.0, "camera.front.hfov_degrees"},
      {61.0, inf, "camera.front.vfov_degrees"},
      // A double the Float32 wire field would round to 180.
      {179.999999999, 45.0, "camera.front.hfov_degrees"},
  };
  for (const auto& c : cases) {
    const std::string refusal = with(c.h, c.v);
    EXPECT_NE(refusal.find(c.names), std::string::npos)
        << c.h << " x " << c.v << ": '" << refusal << "'";
  }
}

// A camera's CameraInfo topic is the image topic's sibling unless named, expanded as the image topic
// is when named, and no topic at all when empty.
TEST_F(ConfigureRefusals, ACamerasCameraInfoTopicIsTheSiblingNamedOrNone) {
  const auto resolved = [](const rclcpp::Parameter& set) {
    const rclcpp::Parameter before = node_->get_parameter(set.get_name());
    EXPECT_TRUE(node_->set_parameter(set).successful);
    std::string topic = "(refused)";
    if (node_->configure().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      topic = node_->resolved_cameras().at(0).camera_info_topic;
      node_->cleanup();
    }
    EXPECT_TRUE(node_->set_parameter(before).successful);
    return topic;
  };
  EXPECT_EQ(resolved({"camera.front.camera_info_topic", std::string("auto")}), "/camera_info");
  EXPECT_EQ(resolved({"camera.front.topic", std::string("/oakd/rgb/image_raw/compressed")}),
            "/oakd/rgb/camera_info");
  EXPECT_EQ(resolved({"camera.front.camera_info_topic", std::string("/calibrated/info")}),
            "/calibrated/info");
  EXPECT_EQ(resolved({"camera.front.camera_info_topic", std::string("info")}), "/info");
  EXPECT_EQ(resolved({"camera.front.camera_info_topic", std::string("")}), "");
  EXPECT_NE(RefusalWith({"camera.front.camera_info_topic", std::string("~info")})
                .find("camera.front.camera_info_topic"),
            std::string::npos);
}

// The SDK records its own session properties under `c3d.`; one set here would replace the SDK's,
// and without `c3d.app.version` the platform discards the session.
TEST_F(ConfigureRefusals, ASessionPropertyInTheSdkNamespaceIsRefusedAtConfigureAndAtRuntime) {
  {
    const auto node = MakeNode(DryRun({{"session_properties",
                                        std::vector<std::string>{"site=w3", "c3d.app.version=9"}}}));
    const std::string refusal = Configure(*node);
    EXPECT_NE(refusal.find("session_properties"), std::string::npos) << refusal;
    EXPECT_NE(refusal.find("c3d.app.version"), std::string::npos) << refusal;
  }
  {
    // The arms are the node's while it fetches them, and the consumer's once it does not.
    const auto node = MakeNode(
        DryRun({{"session_properties", std::vector<std::string>{"c3d.remote_variable.speed=3"}}}));
    EXPECT_NE(Configure(*node).find("c3d.remote_variable.speed"), std::string::npos);
    ASSERT_TRUE(node->set_parameter({"remote_variables", false}).successful);
    EXPECT_EQ(Configure(*node), "");
  }
  // Read again at every session start, so a set at runtime is refused as configure refuses it.
  const auto refused = node_->set_parameter(
      {"session_properties", std::vector<std::string>{"c3d.sessionname=spoofed"}});
  EXPECT_FALSE(refused.successful);
  EXPECT_NE(refused.reason.find("c3d.sessionname"), std::string::npos) << refused.reason;
  // A described sensor's keys are its parameters' to write, never this list's.
  const auto sensor_key = node_->set_parameter(
      {"session_properties", std::vector<std::string>{"c3d.ros.sensor.sonar.units=meters"}});
  EXPECT_FALSE(sensor_key.successful);
  EXPECT_NE(sensor_key.reason.find("c3d.ros.sensor.sonar.units"), std::string::npos)
      << sensor_key.reason;
  // The whole namespace, not a list of the keys the SDK happens to write: a key no run writes is
  // refused as one every run writes, so no parameter is a way to put a `c3d.` key on a session. A
  // camera's field of view is its own parameters' to send, on every frame.
  for (const std::string key :
       {"c3d.version", "c3d.ros.pose_convention", "c3d.ros.camera.front.hfov_degrees",
        "c3d.ros.camera.front.vfov_degrees", "c3d.ros.anything"}) {
    const auto node = MakeNode(DryRun({{"session_properties",
                                        std::vector<std::string>{"site=w3", key + "=61"}}}));
    const std::string refusal = Configure(*node);
    EXPECT_NE(refusal.find("session_properties"), std::string::npos) << key << ": " << refusal;
    EXPECT_NE(refusal.find(key), std::string::npos) << key << ": " << refusal;
    const auto at_runtime =
        node_->set_parameter({"session_properties", std::vector<std::string>{key + "=61"}});
    EXPECT_FALSE(at_runtime.successful) << key;
    EXPECT_NE(at_runtime.reason.find(key), std::string::npos) << key << ": " << at_runtime.reason;
  }
  EXPECT_TRUE(
      node_->set_parameter({"session_properties", std::vector<std::string>{"site=w3"}}).successful);
  EXPECT_TRUE(
      node_->set_parameter({"session_properties", std::vector<std::string>{}}).successful);
}

// A described sensor is refused by name for anything a reader would draw wrong or could not draw,
// and for a field of the other kind or a key no sensor reads, which would be read by nothing. Each
// sensor's series comes from its topic's rule, never from a parameter.
TEST_F(ConfigureRefusals, ASensorDescriptionIsRefusedByNameAndItsSeriesAreItsTopics) {
  ASSERT_EQ(Configure(*node_), "");
  ASSERT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  const auto sensors = node_->resolved_sensors();
  node_->cleanup();
  ASSERT_EQ(sensors.size(), 2u);
  // No series configured for /ultrasonic: the Float32 translator's own `<dotted topic>.value`.
  EXPECT_EQ(sensors[0].series, std::vector<std::string>{"ultrasonic.value"});
  EXPECT_EQ(sensors[1].series,
            (std::vector<std::string>{"line.0", "line.1", "line.2", "line.3"}));

  struct Case {
    rclcpp::Parameter bad;
    std::string names;
  };
  const std::vector<Case> cases = {
      {{"sensor.sonar.kind", std::string("")}, "sensor.sonar.kind"},
      {{"sensor.sonar.kind", std::string("sonar")}, "sensor.sonar.kind"},
      {{"sensor.sonar.topic", std::string("")}, "sensor.sonar.topic"},
      {{"sensor.sonar.topic", std::string("/elsewhere")}, "sensor.sonar.topic"},
      {{"sensor.sonar.cone_half_angle_deg", 0.0}, "sensor.sonar.cone_half_angle_deg"},
      {{"sensor.sonar.units", std::string("feet")}, "sensor.sonar.units"},
      {{"sensor.sonar.range_min_m", -1.0}, "sensor.sonar.range_min_m"},
      {{"sensor.sonar.mount_xyz_m", std::vector<double>{0.1, 0.0}}, "sensor.sonar.mount_xyz_m"},
      {{"sensor.sonar.mount_yaw_deg", 5.0}, "sensor.sonar.mount_yaw_deg"},
      {{"sensor.sonar.element_count", 2}, "sensor.sonar.element_count"},
      {{"sensor.line.element_count", 0}, "sensor.line.element_count"},
      {{"sensor.line.element_count", 33}, "sensor.line.element_count"},
      {{"sensor.line.units", std::string("cm")}, "sensor.line.units"},
      {{"sensors", std::vector<std::string>{"sonar", "sonar"}}, "twice"},
      {{"sensors", std::vector<std::string>{"a.b"}}, "a.b"},
  };
  for (const auto& c : cases) {
    const std::string refusal = RefusalWith(c.bad);
    EXPECT_NE(refusal.find(c.names), std::string::npos)
        << c.bad.get_name() << " = " << c.bad.value_to_string() << ": '" << refusal << "'";
  }
  EXPECT_EQ(RefusalWith({"sensor.sonar.mount_xyz_m", std::vector<double>{0.1, 0.0, 0.05}}), "");

  // A key the node does not read, given where a params file gives it: a misspelt field, or a
  // block under a name `sensors` leaves out. Either would configure and record nothing of it.
  const std::vector<Case> unread = {
      {{"sensor.sonar.mount_xyz", std::vector<double>{0.1, 0.0, 0.05}},
       "sensor.sonar.mount_xyz is not a parameter"},
      {{"sensor.sonr.kind", std::string("cone_rangefinder")},
       "sensor.sonr.kind describes a sensor 'sonr' that `sensors` does not list"},
  };
  for (const auto& c : unread) {
    auto parameters = SharedNodeParameters();
    parameters.push_back(c.bad);
    const auto node = MakeNode(parameters);
    const std::string refusal = Configure(*node);
    EXPECT_NE(refusal.find(c.names), std::string::npos) << c.bad.get_name() << ": '" << refusal
                                                       << "'";
  }
}

// A camera or lidar key the node does not read, given where a params file gives it, is refused by
// name: a block under a name `cameras` or `lidars` leaves out, or a field its block does not have,
// nested ones included. Either would configure and apply nothing of it. The refusal says which,
// and for a field, offers the block's own.
TEST_F(ConfigureRefusals, ACameraOrLidarKeyNothingReadsIsRefusedByName) {
  const auto with_lidar = [] {
    auto parameters = SharedNodeParameters();
    parameters.push_back({"lidars", std::vector<std::string>{"depth"}});
    parameters.push_back({"lidar.depth.topic", std::string("/points")});
    return parameters;
  };
  {
    auto parameters = with_lidar();
    parameters.push_back({"camera.front.hfov_degrees", 61.0});
    parameters.push_back({"camera.front.vfov_degrees", 45.0});
    parameters.push_back({"camera.front.qos.reliability", std::string("best_effort")});
    parameters.push_back({"lidar.depth.frame", std::string("oakd_link")});
    parameters.push_back({"lidar.depth.pointcloud.voxel_leaf_m", 0.1});
    const auto node = MakeNode(parameters);
    ASSERT_EQ(Configure(*node), "") << "every key here is a field of a listed block";
  }

  struct Case {
    rclcpp::Parameter bad;
    std::string names;
    std::string offers;
  };
  const std::vector<Case> unread = {
      {{"camera.rear.topic", std::string("/rear/image")},
       "camera.rear.topic describes a camera 'rear' that `cameras` does not list",
       "add 'rear' to `cameras`, or remove every camera.rear.* key"},
      {{"camera.front.hfov_deg", 61.0}, "camera.front.hfov_deg is not a parameter",
       "hfov_degrees"},
      {{"camera.front.qos.reliabilty", std::string("best_effort")},
       "camera.front.qos.reliabilty is not a parameter", "qos.reliability"},
      {{"lidar.scan.topic", std::string("/scan")},
       "lidar.scan.topic describes a lidar 'scan' that `lidars` does not list",
       "add 'scan' to `lidars`, or remove every lidar.scan.* key"},
      {{"lidar.depth.frame_id", std::string("oakd_link")},
       "lidar.depth.frame_id is not a parameter", "a lidar's fields are"},
      {{"lidar.depth.pointcloud.voxel_leaf", 0.1},
       "lidar.depth.pointcloud.voxel_leaf is not a parameter", "pointcloud.voxel_leaf_m"},
      // A camera's field is not a lidar's: a lidar takes no QoS override.
      {{"lidar.depth.qos.reliability", std::string("best_effort")},
       "lidar.depth.qos.reliability is not a parameter", "pointcloud.zstd_level"},
  };
  for (const auto& c : unread) {
    auto parameters = with_lidar();
    parameters.push_back(c.bad);
    const auto node = MakeNode(parameters);
    const std::string refusal = Configure(*node);
    EXPECT_EQ(refusal.rfind(c.names, 0), 0u) << c.bad.get_name() << ": '" << refusal << "'";
    EXPECT_NE(refusal.find(c.offers), std::string::npos) << c.bad.get_name() << ": '" << refusal
                                                        << "'";
  }
}

// A topic_config key the node does not read, given where a params file gives it, is refused by
// name: one under no key a `topics` entry gives, or a field the entry's group does not have, nested
// ones included. Either would configure and apply nothing of it. The refusal says which: for a
// topic, that it must be in `topics`, how an entry becomes a key, and the keys `topics` gives; for
// a field, every field an entry has.
TEST_F(ConfigureRefusals, ATopicConfigKeyNothingReadsIsRefusedByName) {
  const std::string entry = "topic_config.ultrasonic.";
  std::vector<std::string> fields;
  for (const std::string& name : node_->list_parameters({"topic_config.ultrasonic"}, 0).names) {
    fields.push_back(name.substr(entry.size()));
  }
  ASSERT_FALSE(fields.empty()) << "the shared node declared no field for /ultrasonic";

  struct Case {
    std::vector<rclcpp::Parameter> parameters;
    std::string names;
    std::vector<std::string> offers;
  };
  const auto shared_with = [](const rclcpp::Parameter& bad) {
    auto parameters = SharedNodeParameters();
    parameters.push_back(bad);
    return parameters;
  };
  const std::string unlisted = " matches no entry in `topics`, and a topic's topic_config is read "
                               "only when the topic is in `topics`";
  const std::vector<Case> unread = {
      {shared_with({"topic_config.ultrasonic.rate", 5.0}),
       "topic_config.ultrasonic.rate is not a parameter: a topic_config entry's fields are ",
       {"rate_hz", "Correct the spelling"}},
      {shared_with({"topic_config.ultrasonic.qos.reliabilty", std::string("best_effort")}),
       "topic_config.ultrasonic.qos.reliabilty is not a parameter",
       {"qos.reliability"}},
      {shared_with({"topic_config.ultrasonic", std::string("refuse")}),
       "topic_config.ultrasonic is not a parameter",
       {"ingest"}},
      // A battery is taken by its type with no entry, and its own key is still read by nothing.
      {shared_with({"topic_config.battery_state.ingest", std::string("refuse")}),
       "topic_config.battery_state.ingest" + unlisted,
       {"add the topic to `topics`, or remove the key", "'/odom' is topic_config.odom",
        "'/robot1/*' is topic_config.robot1",
        "The keys `topics` gives are topic_config.line_sensor, topic_config.ultrasonic"}},
      {DryRun({{"topics", std::vector<std::string>{"/navigate_to_pose/_action/status"}},
               {"topic_config.navigate_to_pose_action_status.ingest", std::string("refuse")}}),
       "topic_config.navigate_to_pose_action_status.ingest" + unlisted,
       {"The keys `topics` gives are topic_config.navigate_to_pose__action_status"}},
      // A topic under a wildcard entry is configured by the wildcard's key alone.
      {DryRun({{"topics", std::vector<std::string>{"/robot1/*"}},
               {"topic_config.robot1_battery_state.rate_hz", 2.0}}),
       "topic_config.robot1_battery_state.rate_hz" + unlisted,
       {"The keys `topics` gives are topic_config.robot1"}},
      {DryRun({{"topic_config.odom.rate_hz", 5.0}}), "topic_config.odom.rate_hz" + unlisted,
       {"`topics` is empty"}},
  };
  for (const auto& c : unread) {
    const auto node = MakeNode(c.parameters);
    const std::string refusal = Configure(*node);
    EXPECT_EQ(refusal.rfind(c.names, 0), 0u) << "'" << refusal << "'";
    for (const std::string& offer : c.offers) {
      EXPECT_NE(refusal.find(offer), std::string::npos) << offer << ": '" << refusal << "'";
    }
  }

  // The fields offered are every one the entry has, and none of another entry's whose key extends
  // this one's past a '.'.
  const auto node = MakeNode(
      DryRun({{"topics", std::vector<std::string>{"/sonar", "/sonar.rear"}},
              {"topic_config.sonar.rate", 5.0}}));
  const std::string refusal = Configure(*node);
  ASSERT_EQ(refusal.rfind("topic_config.sonar.rate is not a parameter", 0), 0u) << refusal;
  for (const std::string& field : fields) {
    EXPECT_NE(refusal.find(field), std::string::npos) << field << ": '" << refusal << "'";
  }
  EXPECT_EQ(refusal.find("rear."), std::string::npos) << refusal;
}

// Every field an entry has is accepted under the key each kind of entry gives: a wildcard, a
// namespaced topic, an action status topic, a relative entry in a namespaced node, whose key is the
// entry as written, and a topic whose key holds a '.', which no key's first segment names.
TEST_F(ConfigureRefusals, EveryFieldIsAcceptedUnderTheKeyEachEntryGives) {
  const std::string entry = "topic_config.ultrasonic.";
  std::vector<std::pair<std::string, rclcpp::ParameterValue>> fields;
  for (const std::string& name : node_->list_parameters({"topic_config.ultrasonic"}, 0).names) {
    fields.emplace_back(name.substr(entry.size()),
                        node_->get_parameter(name).get_parameter_value());
  }
  ASSERT_FALSE(fields.empty()) << "the shared node declared no field for /ultrasonic";

  const std::vector<std::pair<std::string, std::string>> entries = {
      {"/robot1/*", "robot1"},
      {"/robot1/sensors/ultrasonic", "robot1_sensors_ultrasonic"},
      {"/navigate_to_pose/_action/status", "navigate_to_pose__action_status"},
      {"odom", "odom"},
      {"~/status", "~_status"},
      {"/sensor.v2/range", "sensor.v2_range"},
  };
  std::vector<std::string> topics;
  for (const auto& topic_and_key : entries) topics.push_back(topic_and_key.first);
  std::vector<rclcpp::Parameter> extra = {{"topics", topics}};
  for (const auto& topic_and_key : entries) {
    for (const auto& [field, value] : fields) {
      extra.push_back({"topic_config." + topic_and_key.second + "." + field, value});
    }
  }
  const auto node = MakeNode(DryRun(extra), "/robot3");
  EXPECT_EQ(Configure(*node), "");
}

// A group an earlier configure declared is read by nothing once its entry leaves `topics`, so its
// key is refused as unlisted rather than taken for a field because it is declared.
TEST_F(ConfigureRefusals, ATopicConfigKeyIsRefusedOnceItsEntryLeavesTopics) {
  const auto node = MakeNode(DryRun({{"topics", std::vector<std::string>{"/odom"}},
                                     {"topic_config.odom.rate_hz", 5.0}}));
  ASSERT_EQ(Configure(*node), "");
  ASSERT_TRUE(node->set_parameter({"topics", std::vector<std::string>{"/scan"}}).successful);
  const std::string refusal = Configure(*node);
  EXPECT_EQ(refusal.rfind("topic_config.odom.rate_hz matches no entry in `topics`", 0), 0u)
      << refusal;
}

// A key in the parameters, whatever its value, keeps its own refusal beside a key nothing reads, so
// an unread-key refusal never stands in for the message that says where the key belongs.
TEST_F(ConfigureRefusals, AKeyParameterKeepsItsOwnRefusalBesideAnUnreadKey) {
  for (const std::string value : {"from-a-params-file", ""}) {
    for (const rclcpp::Parameter& unread :
         {rclcpp::Parameter("camera.front.hfov_deg", 61.0),
          rclcpp::Parameter("topic_config.battery_state.ingest", std::string("refuse")),
          rclcpp::Parameter("topic_config.ultrasonic.rate", 5.0)}) {
      auto parameters = SharedNodeParameters();
      parameters.push_back({"application_key", value});
      parameters.push_back(unread);
      const auto node = MakeNode(parameters);
      const std::string refusal = Configure(*node);
      EXPECT_EQ(refusal.rfind("application_key is not a parameter", 0), 0u)
          << "'" << value << "' beside " << unread.get_name() << ": " << refusal;
    }
  }
}

// A set of session_properties is judged against the remote_variables it lands beside: one atomic
// set may turn the fetch off and supply arms, and one that turns it on may not keep them.
TEST_F(ConfigureRefusals, ASessionPropertySetIsJudgedAgainstTheRemoteVariablesSetWithIt) {
  ASSERT_TRUE(node_->get_parameter("remote_variables").as_bool());
  const auto off_with_arms = node_->set_parameters_atomically(
      {{"remote_variables", false},
       {"session_properties", std::vector<std::string>{"c3d.remote_variable.speed=3"}}});
  EXPECT_TRUE(off_with_arms.successful) << off_with_arms.reason;
  ASSERT_TRUE(node_->set_parameters_atomically(
                       {{"session_properties", std::vector<std::string>{}},
                        {"remote_variables", false}})
                  .successful);
  const auto on_with_arms = node_->set_parameters_atomically(
      {{"remote_variables", true},
       {"session_properties", std::vector<std::string>{"c3d.remote_variable.speed=4"}}});
  EXPECT_FALSE(on_with_arms.successful);
  EXPECT_NE(on_with_arms.reason.find("c3d.remote_variable.speed"), std::string::npos)
      << on_with_arms.reason;
  ASSERT_TRUE(node_->set_parameter({"remote_variables", true}).successful);
}

// The pose stream's values are checked whether or not it is on, each refusal naming the
// parameter: a rate outside what a replay can seek in, a block interval outside the format's, a
// model named by half, an exclude list that would remove the rig's root.
TEST_F(ConfigureRefusals, ThePoseStreamsValuesAreRefusedByNameWhetherOrNotItIsOn) {
  const std::vector<rclcpp::Parameter> refused = {
      {"pose.rate_hz", 0.0},           {"pose.rate_hz", -10.0},
      {"pose.rate_hz", kNaN},          {"pose.rate_hz", 100.5},
      // One sample every 26 s: slower than 30 s less the 5 s block interval.
      {"pose.rate_hz", 1.0 / 26.0},
      {"pose.block_interval_s", 0.05}, {"pose.block_interval_s", 10.5},
      {"pose.block_interval_s", kNaN},
      {"pose.mesh", std::string("raspbot")},
      {"pose.mesh_file_type", std::string("gltf")},
      {"pose.exclude_frames", std::vector<std::string>{"wheel_left_link", "base_link"}},
      {"pose.exclude_frames", std::vector<std::string>{""}},
  };
  for (bool enabled : {false, true}) {
    ASSERT_TRUE(node_->set_parameter({"pose.enabled", enabled}).successful);
    for (const auto& bad : refused) {
      const std::string refusal = RefusalWith(bad);
      EXPECT_NE(refusal.find(bad.get_name()), std::string::npos)
          << "pose.enabled " << enabled << ", " << bad.get_name() << " = "
          << bad.value_to_string() << ": " << refusal;
    }
    EXPECT_EQ(RefusalWith({"pose.rate_hz", 1.0 / 25.0}), "")
        << "one sample every 25 s is the slowest a 5 s block interval allows";
    EXPECT_EQ(RefusalWith({"pose.rate_hz", 100.0}), "");
  }
  ASSERT_TRUE(node_->set_parameter({"pose.enabled", false}).successful);
}

TEST_F(ConfigureRefusals, ThePoseStreamResolvesItsRigFromTheOtherSensorsAndDeviceId) {
  auto parameters = SharedNodeParameters();
  parameters.push_back({"device_id", std::string("raspbot-01")});
  parameters.push_back({"camera.front.frame", std::string("camera_link")});
  parameters.push_back({"sensor.sonar.frame", std::string("sonar_link")});
  parameters.push_back({"pose.enabled", true});
  parameters.push_back({"pose.rate_hz", 20});
  parameters.push_back({"pose.block_interval_s", 2});
  parameters.push_back({"pose.exclude_frames", std::vector<std::string>{"wheel_left_link"}});
  parameters.push_back({"pose.mesh", std::string("raspbot")});
  parameters.push_back({"pose.mesh_file_type", std::string("gltf")});
  parameters.push_back({"pose.send_log_path", std::string("/tmp/send_log.jsonl")});
  const auto node = MakeNode(parameters);
  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      << node->configure_error();
  EXPECT_TRUE(node->resolved_pose_enabled());
  EXPECT_DOUBLE_EQ(node->resolved_pose_rate_hz(), 20.0);
  EXPECT_DOUBLE_EQ(node->resolved_config().pose_block_interval_s, 2.0);
  EXPECT_EQ(node->resolved_config().pose_send_log_path, "/tmp/send_log.jsonl");
  const auto& rig = node->resolved_rig_config();
  EXPECT_EQ(rig.robot_frame, "base_link");
  EXPECT_EQ(rig.excluded, (std::set<std::string>{"wheel_left_link"}));
  EXPECT_EQ(rig.manifest.manifest_id, "ros2-64a722d4f3a20f48") << "hashed from device_id";
  EXPECT_EQ(rig.manifest.robot_name, "raspbot-01");
  EXPECT_EQ(rig.manifest.mesh, "raspbot");
  EXPECT_EQ(rig.manifest.file_type, "gltf");
  EXPECT_EQ(rig.manifest.kinds, (std::map<std::string, std::string>{
                                    {"camera_link", "camera"},
                                    {"sonar_link", "cone_rangefinder"}}));
  // The tf cache window, read from the buffer at configure.
  EXPECT_DOUBLE_EQ(rig.frame_lost_after_s, 10.0);
  node->cleanup();

  parameters.push_back({"pose.manifest_id", std::string("raspbot-chassis-7")});
  const auto named = MakeNode(parameters);
  ASSERT_EQ(Configure(*named), "");
  EXPECT_EQ(named->resolved_rig_config().manifest.manifest_id, "raspbot-chassis-7");

  const auto off = MakeNode(SharedNodeParameters());
  ASSERT_EQ(Configure(*off), "");
  EXPECT_FALSE(off->resolved_pose_enabled()) << "off unless asked for";
}

TEST_F(ConfigureRefusals, APoseKeyNothingReadsIsRefusedByName) {
  auto parameters = SharedNodeParameters();
  parameters.push_back({"pose.rate_hertz", 20.0});
  const auto node = MakeNode(parameters);
  const std::string refusal = Configure(*node);
  EXPECT_EQ(refusal.rfind("pose.rate_hertz is not a parameter", 0), 0u) << refusal;
  EXPECT_NE(refusal.find("rate_hz"), std::string::npos) << refusal;
}

TEST(DefaultDeviceId, IsTheNamespaceWithoutItsSlashes) {
  EXPECT_EQ(cognitive3d_ros::DefaultDeviceId("/"), "ros-unnamed");
  EXPECT_EQ(cognitive3d_ros::DefaultDeviceId(""), "ros-unnamed");
  EXPECT_EQ(cognitive3d_ros::DefaultDeviceId("/robot1"), "ros-robot1");
  EXPECT_EQ(cognitive3d_ros::DefaultDeviceId("/fleet/robot1"), "ros-fleet-robot1");
  EXPECT_EQ(cognitive3d_ros::DefaultDeviceId("/fleet//robot1/"), "ros-fleet-robot1");
}

TEST(DefaultDeviceId, ANamespacedNodeWithNoDeviceIdUsesIt) {
  auto overrides = DryRun();
  overrides.erase(std::remove_if(overrides.begin(), overrides.end(),
                                 [](const rclcpp::Parameter& p) {
                                   return p.get_name() == "device_id";
                                 }),
                  overrides.end());
  const auto node = MakeNode(overrides, "/fleet/robot1");
  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      << node->configure_error();
  EXPECT_EQ(node->resolved_config().device_id, "ros-fleet-robot1");
  node->cleanup();
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ::setenv("C3D_APPLICATION_API_KEY", "K", 1);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
