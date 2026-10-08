// `ros2 run cognitive3d_ros_nav2 doctor` -- doctor for the node this package's entry point runs.
//
// `ros2 run cognitive3d_ros doctor` becomes this program whenever this package is installed,
// because the launch file then runs this package's node. It never hands over to another doctor.
#include "cognitive3d_ros/run_doctor.hpp"
#include "cognitive3d_ros_nav2/register.hpp"

int main(int argc, char** argv) {
  return cognitive3d_ros::RunDoctor(argc, argv, cognitive3d_ros_nav2::MakeNav2Node,
                                    "cognitive3d_ros_nav2");
}
