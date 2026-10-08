// A translator library can build, install and pass its own unit tests while nothing loads it.
// `/behavior_tree_log` is then subscribed at full_ingest and every message of it is discarded,
// because the node's registry was never told the type exists.
//
// So this test asserts the WIRING rather than the translator: it builds the node the shipped entry
// point builds and asks what that node can translate.

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "cognitive3d_ros/node.hpp"
#include "cognitive3d_ros_nav2/register.hpp"

namespace {

class EntryPointTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() {
    if (rclcpp::ok()) rclcpp::shutdown();
  }
};

}  // namespace

TEST_F(EntryPointTest, TheShippedEntryPointCanTranslateBehaviorTreeLog) {
  const auto node = cognitive3d_ros_nav2::MakeNav2Node(rclcpp::NodeOptions());
  ASSERT_NE(node, nullptr);
  EXPECT_TRUE(node->translators().Handles("nav2_msgs/msg/BehaviorTreeLog"))
      << "the Nav2 translator is installed but unreachable from anything that runs";
}

TEST_F(EntryPointTest, ComposingNav2KeepsEveryCoreType) {
  // The optional package ADDS; it must not replace the set every robot gets.
  const auto node = cognitive3d_ros_nav2::MakeNav2Node(rclcpp::NodeOptions());
  for (const char* type : {"nav_msgs/msg/Odometry", "sensor_msgs/msg/BatteryState",
                           "sensor_msgs/msg/JointState", "diagnostic_msgs/msg/DiagnosticArray",
                           "action_msgs/msg/GoalStatusArray"}) {
    EXPECT_TRUE(node->translators().Handles(type)) << type;
  }
}

TEST_F(EntryPointTest, TheBaseNodeStillHasNoNav2Types) {
  // The other half of the contract: `cognitive3d_ros` must stay buildable and runnable on a robot
  // with no nav2_msgs installed, so its own node cannot have acquired the type.
  const auto plain = std::make_shared<cognitive3d_ros::Cognitive3DNode>(rclcpp::NodeOptions());
  EXPECT_FALSE(plain->translators().Handles("nav2_msgs/msg/BehaviorTreeLog"));
}
