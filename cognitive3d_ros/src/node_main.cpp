// `ros2 run cognitive3d_ros cognitive3d_node`.
#include <memory>

#include "cognitive3d_ros/run_node.hpp"

int main(int argc, char** argv) {
  return cognitive3d_ros::RunNode(argc, argv, [](const rclcpp::NodeOptions& options) {
    return std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
  });
}
