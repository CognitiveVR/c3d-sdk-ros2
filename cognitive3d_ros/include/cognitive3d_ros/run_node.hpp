#pragma once

#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "cognitive3d_ros/node.hpp"

namespace cognitive3d_ros {

using NodeFactory =
    std::function<std::shared_ptr<Cognitive3DNode>(const rclcpp::NodeOptions& options)>;

// The body of every shipped `cognitive3d_node` executable. A node that cannot be constructed from
// its parameters (an empty YAML list, a mistyped value) is reported the way doctor reports it,
// naming the parameter and the fix, and exits 1 instead of aborting on an uncaught exception. An
// exception from a callback is reported too, and exits 1 after the node is destroyed in order.
int RunNode(int argc, char** argv, const NodeFactory& make);

}  // namespace cognitive3d_ros
