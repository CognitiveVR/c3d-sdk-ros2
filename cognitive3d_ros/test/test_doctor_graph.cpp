// topics/qos against real publishers on a real graph: what the RMW reports for each publisher's
// profile, read the way doctor reads it, through to the verdict and the fix it prints.
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "cognitive3d_ros/doctor.hpp"
#include "cognitive3d_ros/run_doctor.hpp"
#include "cognitive3d_ros/topic_plan.hpp"

using cognitive3d_ros::doctor::Check;
using cognitive3d_ros::doctor::Severity;

namespace {

class TopicQosOnTheGraph : public ::testing::Test {
 protected:
  void SetUp() override { observer_ = std::make_shared<rclcpp::Node>("doctor_observer"); }

  // A node of its own per publisher, named, so the check has a node to name.
  void Publish(const std::string& node_name, const std::string& topic, const rclcpp::QoS& qos) {
    auto node = std::make_shared<rclcpp::Node>(node_name);
    publishers_.push_back(node->create_publisher<std_msgs::msg::String>(topic, qos));
    nodes_.push_back(node);
  }

  // The verdict once every publisher is discovered, which is what doctor's waits buy it.
  Check Verdict(const std::string& topic, const cognitive3d_ros::TopicRule& rule) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
    std::vector<rclcpp::TopicEndpointInfo> found;
    while (std::chrono::steady_clock::now() < deadline) {
      found = observer_->get_publishers_info_by_topic(topic);
      if (found.size() >= publishers_.size()) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    EXPECT_EQ(found.size(), publishers_.size()) << "discovery did not finish";
    return cognitive3d_ros::doctor::CheckTopicQos(
        {cognitive3d_ros::TopicQosFor(topic, found, rule)});
  }

  static cognitive3d_ros::TopicRule Rule(const std::string& key) {
    cognitive3d_ros::TopicRule rule;
    rule.config_key = "topic_config." + key;
    return rule;
  }

  std::shared_ptr<rclcpp::Node> observer_;
  std::vector<std::shared_ptr<rclcpp::Node>> nodes_;
  std::vector<rclcpp::PublisherBase::SharedPtr> publishers_;
};

bool Says(const Check& check, const std::string& text) {
  return (check.detail + " " + check.remedy).find(text) != std::string::npos;
}

}  // namespace

// The stock ros2_control case: a transient-local broadcaster and a volatile publisher on one topic.
TEST_F(TopicQosOnTheGraph, PublishersThatDisagreeOnDurabilityAreEachNamed) {
  const std::string topic = "/doctor_graph/joint_states";
  Publish("joint_state_broadcaster", topic, rclcpp::QoS(10).reliable().transient_local());
  Publish("joint_state_publisher", topic, rclcpp::QoS(10).reliable().durability_volatile());

  const Check check = Verdict(topic, Rule("doctor_graph.joint_states"));
  EXPECT_EQ(check.severity, Severity::kAdvisory);
  EXPECT_TRUE(Says(check, "/joint_state_broadcaster reliable/transient_local")) << check.detail;
  EXPECT_TRUE(Says(check, "/joint_state_publisher reliable/volatile")) << check.detail;
  EXPECT_TRUE(Says(check, "Set topic_config.doctor_graph.joint_states.qos.durability: volatile "
                          "to match a volatile publisher."))
      << check.remedy;

  // Pinned by the override the check names, either discovery order requests the same profile.
  cognitive3d_ros::TopicRule pinned = Rule("doctor_graph.joint_states");
  pinned.qos.durability = "volatile";
  const Check settled = Verdict(topic, pinned);
  EXPECT_EQ(settled.severity, Severity::kOk);
  EXPECT_TRUE(Says(settled, topic + ": its publishers disagree on durability")) << settled.detail;
  EXPECT_TRUE(Says(settled, "and your override (topic_config.doctor_graph.joint_states.qos."
                            "durability: volatile) reaches every one"))
      << settled.detail;
  EXPECT_FALSE(Says(settled, "topic(s) agree")) << settled.detail;
}

TEST_F(TopicQosOnTheGraph, PublishersThatDisagreeOnReliabilityAreEachNamed) {
  const std::string topic = "/doctor_graph/scan";
  Publish("lidar_driver", topic, rclcpp::QoS(10).best_effort());
  Publish("scan_relay", topic, rclcpp::QoS(10).reliable());

  const Check check = Verdict(topic, Rule("doctor_graph.scan"));
  EXPECT_EQ(check.severity, Severity::kAdvisory);
  EXPECT_TRUE(Says(check, "/lidar_driver best_effort/volatile")) << check.detail;
  EXPECT_TRUE(Says(check, "/scan_relay reliable/volatile")) << check.detail;
  EXPECT_TRUE(Says(check, "Set topic_config.doctor_graph.scan.qos.reliability: best_effort to "
                          "match a best-effort publisher."))
      << check.remedy;
}

TEST_F(TopicQosOnTheGraph, PublishersThatAgreePass) {
  const std::string topic = "/doctor_graph/odom";
  Publish("odometry_a", topic, rclcpp::QoS(10).reliable().transient_local());
  Publish("odometry_b", topic, rclcpp::QoS(5).reliable().transient_local());

  const Check check = Verdict(topic, Rule("doctor_graph.odom"));
  EXPECT_EQ(check.name, "topics/qos");
  EXPECT_EQ(check.severity, Severity::kOk) << check.detail;
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
