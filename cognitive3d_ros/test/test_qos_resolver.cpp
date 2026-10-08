// QoS resolution is the highest-risk area for a passive subscriber, because a
// mismatch produces silence rather than an error. These tests pin the two rules
// that matter.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "cognitive3d_ros/qos_resolver.hpp"

using namespace cognitive3d_ros;

namespace {

rclcpp::TopicEndpointInfo MakeEndpoint(bool reliable, bool transient_local, std::size_t depth) {
  rmw_topic_endpoint_info_t raw = rmw_get_zero_initialized_topic_endpoint_info();
  static std::string node_name = "pub";
  static std::string node_ns = "/";
  static std::string type_name = "std_msgs/msg/String";
  raw.node_name = node_name.c_str();
  raw.node_namespace = node_ns.c_str();
  raw.topic_type = type_name.c_str();
  raw.endpoint_type = RMW_ENDPOINT_PUBLISHER;
  raw.qos_profile = rmw_qos_profile_default;
  raw.qos_profile.reliability = reliable ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
                                         : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  raw.qos_profile.durability = transient_local ? RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
                                               : RMW_QOS_POLICY_DURABILITY_VOLATILE;
  raw.qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  raw.qos_profile.depth = depth;
  return rclcpp::TopicEndpointInfo(raw);
}

bool IsBestEffort(const rclcpp::QoS& q) {
  return q.get_rmw_qos_profile().reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
}
bool IsTransientLocal(const rclcpp::QoS& q) {
  return q.get_rmw_qos_profile().durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
}

}  // namespace

TEST(QosResolver, NoPublishersYieldsTheWeakestProfile) {
  const auto r = ResolveQos({}, QosOverride{});
  EXPECT_FALSE(r.publishers_found);
  EXPECT_TRUE(IsBestEffort(r.qos));
  EXPECT_FALSE(IsTransientLocal(r.qos));
}

TEST(QosResolver, AllReliablePublishersYieldReliable) {
  const auto r = ResolveQos({MakeEndpoint(true, false, 10), MakeEndpoint(true, false, 10)},
                            QosOverride{});
  EXPECT_FALSE(IsBestEffort(r.qos));
  EXPECT_EQ(r.publisher_count, 2u);
}

TEST(QosResolver, OneBestEffortPublisherForcesBestEffort) {
  // publisher BEST_EFFORT + subscriber RELIABLE means no connection at all, and
  // the symptom is silence. Requesting best_effort still connects to the
  // reliable publisher, so it is strictly the safer request.
  const auto r = ResolveQos({MakeEndpoint(true, false, 10), MakeEndpoint(false, false, 10)},
                            QosOverride{});
  EXPECT_TRUE(IsBestEffort(r.qos));
  EXPECT_TRUE(r.publishers_disagreed);
}

TEST(QosResolver, AllTransientLocalPublishersYieldTransientLocal) {
  // This is /tf_static and /robot_description. Requesting volatile here means
  // never seeing the single latched message published before we started.
  const auto r = ResolveQos({MakeEndpoint(true, true, 100)}, QosOverride{});
  EXPECT_TRUE(IsTransientLocal(r.qos));
}

TEST(QosResolver, MixedDurabilityFallsBackToVolatile) {
  // publisher VOLATILE + subscriber TRANSIENT_LOCAL means no connection, so a
  // single volatile publisher forces the whole request down.
  const auto r = ResolveQos({MakeEndpoint(true, true, 10), MakeEndpoint(true, false, 10)},
                            QosOverride{});
  EXPECT_FALSE(IsTransientLocal(r.qos));
  EXPECT_TRUE(r.publishers_disagreed);
}

TEST(QosResolver, DepthTakesTheDeepestOffer) {
  const auto r = ResolveQos({MakeEndpoint(true, false, 5), MakeEndpoint(true, false, 250)},
                            QosOverride{});
  EXPECT_EQ(r.qos.get_rmw_qos_profile().depth, 250u);
}

TEST(QosResolver, ConfigurationOverridesResolution) {
  QosOverride o;
  o.reliability = "best_effort";
  o.durability = "transient_local";
  o.depth = 42;
  const auto r = ResolveQos({MakeEndpoint(true, false, 10)}, o);
  EXPECT_TRUE(r.overridden);
  EXPECT_TRUE(IsBestEffort(r.qos));
  EXPECT_TRUE(IsTransientLocal(r.qos));
  EXPECT_EQ(r.qos.get_rmw_qos_profile().depth, 42u);
}

TEST(QosResolver, DescribeQosIsReadable) {
  const auto r = ResolveQos({MakeEndpoint(false, true, 7)}, QosOverride{});
  const std::string text = DescribeQos(r.qos);
  EXPECT_NE(text.find("best_effort"), std::string::npos);
  EXPECT_NE(text.find("transient_local"), std::string::npos);
}

// The incompatibility error names the policy and the line that fixes it, not a number.
TEST(QosIncompatibility, TheErrorNamesThePolicyAndTheOverrideLine) {
  const std::string reliability = cognitive3d_ros::DescribeQosIncompatibility(
      "/odom", 2, RMW_QOS_POLICY_RELIABILITY, "topic_config.odom");
  EXPECT_NE(reliability.find(rclcpp::qos_policy_name_from_kind(RMW_QOS_POLICY_RELIABILITY)),
            std::string::npos)
      << reliability;
  EXPECT_NE(reliability.find("topic_config.odom.qos.reliability: best_effort"), std::string::npos)
      << reliability;
  // A second, compatible publisher on the topic keeps delivering, so the error must not say the
  // whole topic is silent.
  EXPECT_NE(reliability.find("No data arrives from an incompatible publisher"), std::string::npos)
      << reliability;
  EXPECT_NE(reliability.find("a compatible publisher on the same topic still delivers"),
            std::string::npos)
      << reliability;

  const std::string durability = cognitive3d_ros::DescribeQosIncompatibility(
      "/map", 1, RMW_QOS_POLICY_DURABILITY, "camera.front");
  EXPECT_NE(durability.find("camera.front.qos.durability: volatile"), std::string::npos)
      << durability;

  const std::string unconfigured = cognitive3d_ros::DescribeQosIncompatibility(
      "/battery", 1, RMW_QOS_POLICY_RELIABILITY, "");
  EXPECT_NE(unconfigured.find("ros2 topic info --verbose /battery"), std::string::npos)
      << unconfigured;
  EXPECT_EQ(unconfigured.find(".qos."), std::string::npos)
      << "a subscription with no override group must not be told to set one: " << unconfigured;
}
