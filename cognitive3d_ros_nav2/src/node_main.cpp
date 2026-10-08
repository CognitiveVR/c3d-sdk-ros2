// `ros2 run cognitive3d_ros_nav2 cognitive3d_node` -- the entry point for a robot that runs Nav2.
//
// The same node as `cognitive3d_ros`'s, composed here because this is the package that may depend
// on `nav2_msgs`. Both entry points share RunNode, so they are the same program from the outside.
#include "cognitive3d_ros/run_node.hpp"
#include "cognitive3d_ros_nav2/register.hpp"

int main(int argc, char** argv) {
  return cognitive3d_ros::RunNode(argc, argv, cognitive3d_ros_nav2::MakeNav2Node);
}
