// The shipped executables' body. An exception from a callback ends the spin; RunNode says so and
// exits 1 after destroying the node in order, where an uncaught one would terminate the process
// and leave its session for recovery to close. RunNode initialises and shuts rclcpp down itself,
// so this binary does neither.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "cognitive3d_ros/node.hpp"
#include "cognitive3d_ros/run_node.hpp"

namespace {

// The node, with one more callback, which throws the first time it runs.
class ThrowingNode : public cognitive3d_ros::Cognitive3DNode {
 public:
  explicit ThrowingNode(const rclcpp::NodeOptions& options) : Cognitive3DNode(options) {
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [] {
      throw std::runtime_error("a callback threw, as a graph query racing shutdown can");
    });
  }

 private:
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

TEST(RunNode, AnExceptionFromACallbackIsReportedAndExitsOne) {
  char program[] = "cognitive3d_node";
  char* argv[] = {program, nullptr};
  ::testing::internal::CaptureStderr();
  const int status = cognitive3d_ros::RunNode(1, argv, [](const rclcpp::NodeOptions& options) {
    return std::make_shared<ThrowingNode>(options);
  });
  const std::string said = ::testing::internal::GetCapturedStderr();
  EXPECT_EQ(status, 1);
  EXPECT_NE(said.find("stopped by an exception from a callback: a callback threw"),
            std::string::npos)
      << said;
}
