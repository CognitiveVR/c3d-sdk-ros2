// The component a container loads, found the way `ros2 component load` finds it: by package and
// class name in the ament index, its library opened by class_loader and its factory picked by name.
// Loaded that way it must be the node that translates `/behavior_tree_log`. The base package's
// component subscribes the same topic and discards every message of it. Loaded with a namespace
// and tf remaps, it must read the tree they name.

#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <nav2_msgs/msg/behavior_tree_log.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/component_manager.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/qos.hpp>

#include "cognitive3d_ros_nav2/register.hpp"

namespace fs = std::filesystem;

namespace {

// Configure refuses a scene_id that is not a UUID.
constexpr const char kSceneId[] = "a1b2c3d4-0000-4000-8000-000000000001";

class ComponentTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    ::setenv("C3D_APPLICATION_API_KEY", "K", 1);
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() {
    if (rclcpp::ok()) rclcpp::shutdown();
  }
};

// Every spooled chunk that mentions `needle`. An event name carries no quote, so the envelope's
// escaping of the payload does not hide it.
int SpoolMentions(const fs::path& dir, const std::string& needle) {
  int found = 0;
  std::error_code ec;
  for (const auto& entry : fs::recursive_directory_iterator(dir, ec)) {
    if (!entry.is_regular_file()) continue;
    std::ifstream in(entry.path(), std::ios::binary);
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (text.find(needle) != std::string::npos) ++found;
  }
  return found;
}

// The `p` of every gaze record in the spool under `dir`.
std::vector<std::vector<double>> SpooledGazePositions(const fs::path& dir) {
  std::vector<std::vector<double>> positions;
  std::error_code ec;
  for (const auto& entry : fs::recursive_directory_iterator(dir, ec)) {
    if (!entry.is_regular_file()) continue;
    std::ifstream in(entry.path(), std::ios::binary);
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    try {
      const auto envelope = nlohmann::json::parse(text);
      if (!envelope.contains("body") || !envelope["body"].is_string()) continue;
      const auto body = nlohmann::json::parse(envelope["body"].get<std::string>());
      if (!body.contains("interval") || !body.contains("data")) continue;
      for (const auto& record : body["data"]) {
        if (record.contains("p")) positions.push_back(record["p"].get<std::vector<double>>());
      }
    } catch (const nlohmann::json::exception&) {
    }
  }
  return positions;
}

// A container's manager with the components it has loaded in reach.
class Container : public rclcpp_components::ComponentManager {
 public:
  using ComponentManager::ComponentManager;
  // Through the manager's own load_node handler, which builds the node's options from the
  // request alone, as it does for `ros2 component load`.
  LoadNode::Response Load(const std::shared_ptr<LoadNode::Request>& request) {
    auto response = std::make_shared<LoadNode::Response>();
    on_load_node(std::make_shared<rmw_request_id_t>(), request, response);
    return *response;
  }
  std::shared_ptr<cognitive3d_ros::Cognitive3DNode> Loaded() const {
    if (node_wrappers_.empty()) return nullptr;
    return std::static_pointer_cast<cognitive3d_ros_nav2::Cognitive3DNode>(
        node_wrappers_.begin()->second.get_node_instance());
  }
};

tf2_msgs::msg::TFMessage Transform(const std::string& parent, const std::string& child, double x,
                                   const rclcpp::Time& stamp) {
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = parent;
  transform.header.stamp = stamp;
  transform.child_frame_id = child;
  transform.transform.translation.x = x;
  transform.transform.rotation.w = 1.0;
  tf2_msgs::msg::TFMessage message;
  message.transforms.push_back(transform);
  return message;
}

}  // namespace

TEST_F(ComponentTest, LoadedByClassNameItTranslatesABehaviorTreeLog) {
  const fs::path dir =
      fs::temp_directory_path() / ("c3d_nav2_component_" + std::to_string(::getpid()));
  fs::remove_all(dir);
  fs::create_directories(dir);

  auto manager = std::make_shared<rclcpp_components::ComponentManager>(
      std::weak_ptr<rclcpp::Executor>(), "nav2_component_test_manager");
  std::vector<rclcpp_components::ComponentManager::ComponentResource> resources;
  ASSERT_NO_THROW(resources = manager->get_component_resources("cognitive3d_ros_nav2"))
      << "the package registers no component in the ament index";
  const auto resource = std::find_if(resources.begin(), resources.end(), [](const auto& entry) {
    return entry.first == "cognitive3d_ros_nav2::Cognitive3DNode";
  });
  ASSERT_NE(resource, resources.end()) << "no component of that class name";
  const auto factory = manager->create_component_factory(*resource);
  ASSERT_NE(factory, nullptr) << "the library holds no factory for the class it is indexed under";

  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"scene_id", std::string(kSceneId)},
      {"scene_version", std::string("1")},
      {"device_id", std::string("robot")},
      {"spool_dir", dir.string()},
      {"protocol", std::string("http")},
      {"gateway", std::string("127.0.0.1:1")},
      {"pose_convention", std::string("unity")},
      {"remote_variables", false},
      {"auto_start_session", true},
      {"discovery_period_s", 0.2},
      {"topics", std::vector<std::string>{"/behavior_tree_log"}},
  });
  rclcpp_components::NodeInstanceWrapper wrapper = factory->create_node_instance(options);
  std::shared_ptr<cognitive3d_ros::Cognitive3DNode> node =
      std::static_pointer_cast<cognitive3d_ros_nav2::Cognitive3DNode>(wrapper.get_node_instance());
  ASSERT_NE(node, nullptr);

  auto client = std::make_shared<rclcpp::Node>("nav2_component_test_client");
  auto log = client->create_publisher<nav2_msgs::msg::BehaviorTreeLog>("/behavior_tree_log", 10);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(wrapper.get_node_base_interface());
  executor.add_node(client);
  const auto spin_until = [&executor](const std::function<bool()>& done,
                                      std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline && !done()) {
      executor.spin_some(std::chrono::milliseconds(20));
    }
    return done();
  };
  const auto translated = [&node] {
    const auto it = node->topic_stats().find("/behavior_tree_log");
    return it != node->topic_stats().end() && it->second.translated > 0;
  };

  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      << node->configure_error();
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  nav2_msgs::msg::BehaviorTreeLog message;
  nav2_msgs::msg::BehaviorTreeStatusChange spin;
  spin.node_name = "Spin";
  spin.previous_status = "IDLE";
  spin.current_status = "RUNNING";
  message.event_log.push_back(spin);
  EXPECT_TRUE(spin_until(
      [&] {
        log->publish(message);
        return translated();
      },
      std::chrono::seconds(15)))
      << "/behavior_tree_log reached the component and nothing translated it";

  ASSERT_EQ(node->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_GE(SpoolMentions(dir, "nav.recovery.started"), 1)
      << "the recovery the tree logged never reached the session";

  node->cleanup();
  executor.remove_node(client);
  executor.remove_node(wrapper.get_node_base_interface());
  node.reset();
  std::error_code ec;
  fs::remove_all(dir, ec);
}

// A container gives a component the namespace and remaps of its load request and none of the
// process's arguments. A namespaced Nav2 robot loads its nodes with `/tf:=tf` and
// `/tf_static:=tf_static`, so its tree is on `/robot1/tf`, and loaded that way the node must read
// that tree: not a global one beside it carrying the same frame names, and not nothing.
TEST_F(ComponentTest, LoadedWithANamespaceAndTfRemapsItReadsTheNamespacedTree) {
  const fs::path dir =
      fs::temp_directory_path() / ("c3d_nav2_component_tf_" + std::to_string(::getpid()));
  fs::remove_all(dir);
  fs::create_directories(dir);

  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto container = std::make_shared<Container>(executor, "nav2_component_tf_test_container");
  auto request = std::make_shared<Container::LoadNode::Request>();
  request->package_name = "cognitive3d_ros_nav2";
  request->plugin_name = "cognitive3d_ros_nav2::Cognitive3DNode";
  request->node_name = "cognitive3d";
  request->node_namespace = "/robot1";
  request->remap_rules = {"/tf:=tf", "/tf_static:=tf_static"};
  for (const rclcpp::Parameter& parameter : std::vector<rclcpp::Parameter>{
           {"scene_id", std::string(kSceneId)},
           {"scene_version", std::string("1")},
           {"device_id", std::string("robot1")},
           {"spool_dir", dir.string()},
           {"protocol", std::string("http")},
           {"gateway", std::string("127.0.0.1:1")},
           {"pose_convention", std::string("rep103")},
           {"remote_variables", false},
           {"auto_start_session", true},
           {"reference_frame", std::string("map")},
           {"fallback_reference_frame", std::string("")},
       }) {
    request->parameters.push_back(parameter.to_parameter_msg());
  }
  const auto loaded = container->Load(request);
  ASSERT_TRUE(loaded.success) << loaded.error_message;
  EXPECT_EQ(loaded.full_node_name, "/robot1/cognitive3d");
  const std::shared_ptr<cognitive3d_ros::Cognitive3DNode> node = container->Loaded();
  ASSERT_NE(node, nullptr);

  // The robot's tree, map -> odom -> base_link at x = 1.5, only under its namespace, and another
  // robot's with the same frames at x = -10 on the global topics.
  auto client = std::make_shared<rclcpp::Node>("nav2_component_tf_test_client");
  executor->add_node(client);
  auto robot_static = client->create_publisher<tf2_msgs::msg::TFMessage>(
      "/robot1/tf_static", tf2_ros::StaticBroadcasterQoS());
  auto robot_tf = client->create_publisher<tf2_msgs::msg::TFMessage>(
      "/robot1/tf", tf2_ros::DynamicBroadcasterQoS());
  auto other_static = client->create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf_static", tf2_ros::StaticBroadcasterQoS());
  auto other_tf =
      client->create_publisher<tf2_msgs::msg::TFMessage>("/tf", tf2_ros::DynamicBroadcasterQoS());
  robot_static->publish(Transform("map", "odom", 1.0, client->now()));
  other_static->publish(Transform("map", "odom", -7.0, client->now()));
  const auto spin_publishing = [&](const std::function<bool()>& done,
                                   std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline && !done()) {
      robot_tf->publish(Transform("odom", "base_link", 0.5, client->now()));
      other_tf->publish(Transform("odom", "base_link", -3.0, client->now()));
      executor->spin_some(std::chrono::milliseconds(20));
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    return done();
  };
  const auto subscribed_by_the_node = [&client](const std::string& topic) {
    for (const auto& endpoint : client->get_subscriptions_info_by_topic(topic)) {
      if (endpoint.node_name() == "cognitive3d" && endpoint.node_namespace() == "/robot1") {
        return true;
      }
    }
    return false;
  };

  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      << node->configure_error();
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_TRUE(spin_publishing(
      [&] {
        return subscribed_by_the_node("/robot1/tf") && subscribed_by_the_node("/robot1/tf_static");
      },
      std::chrono::seconds(10)))
      << "the node's tf subscriptions are not the namespaced, remapped topics";
  EXPECT_TRUE(spin_publishing(
      [&] { return client->count_subscribers("/tf") + client->count_subscribers("/tf_static") == 0; },
      std::chrono::seconds(10)))
      << "something in the node subscribes the global tf topics";
  ASSERT_TRUE(spin_publishing(
      [&] { return node->tf_buffer()->canTransform("map", "base_link", tf2::TimePointZero); },
      std::chrono::seconds(10)))
      << "the node's buffer never resolved the robot's tree";
  EXPECT_DOUBLE_EQ(
      node->tf_buffer()->lookupTransform("map", "base_link", tf2::TimePointZero).transform
          .translation.x,
      1.5)
      << "the buffer holds another tree than the robot's";
  // Poses from that tree, at the default rate, into the session.
  spin_publishing([] { return false; }, std::chrono::milliseconds(1500));
  ASSERT_EQ(node->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  const auto positions = SpooledGazePositions(dir);
  EXPECT_GE(positions.size(), 5u) << "the session recorded no poses";
  for (const auto& p : positions) {
    ASSERT_EQ(p.size(), 3u);
    EXPECT_NEAR(p[0], 1.5, 1e-3) << "a pose from another tree than the robot's";
  }

  node->cleanup();
  executor->remove_node(client);
  container.reset();
  std::error_code ec;
  fs::remove_all(dir, ec);
}
