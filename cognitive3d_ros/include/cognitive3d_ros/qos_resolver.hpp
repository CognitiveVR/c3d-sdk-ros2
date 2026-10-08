#pragma once

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

#include "cognitive3d_ros/topic_plan.hpp"

namespace cognitive3d_ros {

struct QosResolution {
  rclcpp::QoS qos{rclcpp::KeepLast(10)};
  bool publishers_found = false;
  std::size_t publisher_count = 0;
  bool publishers_disagreed = false;
  bool overridden = false;
  std::string rationale;
};

// Resolve the QoS to request for a topic by reading what the publishers
// actually offer.
//
// This is the single highest-risk area for a passive subscriber, because the
// failure mode is silence rather than an error. The compatibility rules that
// matter:
//
//   publisher BEST_EFFORT + subscriber RELIABLE          -> no connection
//   publisher VOLATILE    + subscriber TRANSIENT_LOCAL   -> no connection
//
// So we request the weakest profile that still connects to every publisher:
// RELIABLE only when all publishers offer it, TRANSIENT_LOCAL only when all
// publishers offer it. That last one matters because /tf_static and
// /robot_description are latched, and requesting VOLATILE there means we never
// see the one message that was published before we started.
QosResolution ResolveQos(const std::vector<rclcpp::TopicEndpointInfo>& publishers,
                         const QosOverride& override_cfg);

// Convenience wrapper that queries the graph. Templated on the node type so it
// works for both rclcpp::Node and rclcpp_lifecycle::LifecycleNode without this
// header taking a dependency on the lifecycle package.
template <typename NodeT>
QosResolution ResolveQos(NodeT* node, const std::string& topic,
                         const QosOverride& override_cfg) {
  return ResolveQos(node->get_publishers_info_by_topic(topic), override_cfg);
}

std::string DescribeQos(const rclcpp::QoS& qos);

// The line that makes the subscription whose profile `override_key` sets (`topic_config.<key>` or
// `camera.<name>`) ask for no more than a weaker publisher offers on `policy`. Empty for a policy
// other than reliability and durability, and for an empty `override_key`.
std::string QosRelaxation(const std::string& override_key, rmw_qos_policy_kind_t policy);

// The error for a subscription whose requested profile a publisher cannot meet: the policy by name
// and, when `override_key` names the parameter group that sets this subscription's profile
// (`topic_config.<key>` or `camera.<name>`), the line that matches the publisher. Empty
// `override_key` is a subscription with no override group.
std::string DescribeQosIncompatibility(const std::string& topic, int total_count,
                                       rmw_qos_policy_kind_t policy,
                                       const std::string& override_key);

}  // namespace cognitive3d_ros
