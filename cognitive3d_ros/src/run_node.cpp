#include "cognitive3d_ros/run_node.hpp"

#include <exception>
#include <iostream>
#include <string>
#include <vector>

#include "cognitive3d_ros/doctor.hpp"

namespace cognitive3d_ros {

int RunNode(int argc, char** argv, const NodeFactory& make) {
  const std::vector<std::string> args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  rclcpp::NodeOptions options;
  options.arguments(args);

  int status = 0;
  // Scoped so the node is destroyed before shutdown, as rclcpp_components' generated entry point
  // does.
  {
    std::shared_ptr<Cognitive3DNode> node;
    try {
      node = make(options);
    } catch (const std::exception& thrown) {
      const doctor::Check check = doctor::CheckConstruction(thrown.what());
      std::cerr << "cognitive3d_node: " << check.detail << "\n" << check.remedy << "\n";
      status = 1;
    }
    if (node) {
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node->get_node_base_interface());
      // A callback that throws ends the spin. Said, and the node is then destroyed as on a stop,
      // rather than std::terminate leaving its session for recovery to close.
      try {
        executor.spin();
      } catch (const std::exception& thrown) {
        RCLCPP_FATAL(node->get_logger(), "stopped by an exception from a callback: %s",
                     thrown.what());
        status = 1;
      } catch (...) {
        RCLCPP_FATAL(node->get_logger(), "stopped by an unknown exception from a callback");
        status = 1;
      }
      executor.remove_node(node->get_node_base_interface());
    }
  }

  rclcpp::shutdown();
  return status;
}

}  // namespace cognitive3d_ros
