#include "cognitive3d_ros_nav2/register.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include "cognitive3d_ros_nav2/behavior_tree_translator.hpp"

namespace cognitive3d_ros_nav2 {

void RegisterNav2Translators(cognitive3d_ros::TranslatorRegistry& registry) {
  registry.Register(std::make_unique<BehaviorTreeTranslator>());
}

Cognitive3DNode::Cognitive3DNode(const rclcpp::NodeOptions& options)
    : cognitive3d_ros::Cognitive3DNode(options) {
  AddTranslators(&RegisterNav2Translators);
}

std::shared_ptr<cognitive3d_ros::Cognitive3DNode> MakeNav2Node(
    const rclcpp::NodeOptions& options) {
  return std::make_shared<Cognitive3DNode>(options);
}

}  // namespace cognitive3d_ros_nav2

RCLCPP_COMPONENTS_REGISTER_NODE(cognitive3d_ros_nav2::Cognitive3DNode)
