#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "cognitive3d_ros/node.hpp"
#include "cognitive3d_ros/translators.hpp"

namespace cognitive3d_ros_nav2 {

// Adds the Nav2 translators to a registry.
void RegisterNav2Translators(cognitive3d_ros::TranslatorRegistry& registry);

// The ordinary Cognitive3D node with the Nav2 translators added at construction, which is before
// anything can configure it. This package's component, loaded by this class name, and the node its
// executable runs.
class Cognitive3DNode : public cognitive3d_ros::Cognitive3DNode {
 public:
  explicit Cognitive3DNode(const rclcpp::NodeOptions& options);
};

// The node this package's executable runs: the ordinary Cognitive3D node with the Nav2
// translators added before it is configured.
//
// It exists as a function rather than living inside `main` so that a test can build the SAME
// thing the entry point does. A test that registered the translators itself would pass with
// nothing wired up: a library that is built and installed but loaded by nothing leaves
// `/behavior_tree_log` subscribed and every message of it discarded.
std::shared_ptr<cognitive3d_ros::Cognitive3DNode> MakeNav2Node(const rclcpp::NodeOptions& options);

}  // namespace cognitive3d_ros_nav2
