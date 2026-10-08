// `ros2 run cognitive3d_ros doctor` -- says why a run will not work, before it does not work.
//
// It checks the build of the node the launch file runs. With cognitive3d_ros_nav2 installed that
// is the Nav2 build, so this process becomes that package's doctor, with the same arguments; that
// doctor never hands over again. Only when it cannot be started does this one report on the plain
// node, and it says so.
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "cognitive3d_ros/run_doctor.hpp"

int main(int argc, char** argv) {
  std::string index_error;
  const std::string launched =
      cognitive3d_ros::LaunchedBuildDoctor([&index_error](const std::string& package) {
        return cognitive3d_ros::InstalledPackagePrefix(package, &index_error);
      });
  std::string handoff_error;
  if (!launched.empty()) {
    std::vector<char*> arguments(argv, argv + argc);
    arguments[0] = const_cast<char*>(launched.c_str());
    arguments.push_back(nullptr);
    ::execv(launched.c_str(), arguments.data());
    handoff_error = launched + ": " + std::strerror(errno);
  }
  return cognitive3d_ros::RunDoctor(
      argc, argv,
      [](const rclcpp::NodeOptions& options) {
        return std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
      },
      "cognitive3d_ros", handoff_error, index_error);
}
