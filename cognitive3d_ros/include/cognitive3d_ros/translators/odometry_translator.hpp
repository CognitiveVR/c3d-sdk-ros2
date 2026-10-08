#pragma once

#include <nav_msgs/msg/odometry.hpp>

#include "cognitive3d_ros/translators.hpp"

namespace cognitive3d_ros {

// nav_msgs/Odometry to derived scalars. The pose is deliberately not recorded
// here; tf2 is the single source of pose, so that two subsystems cannot
// disagree about where the robot was.
//
// Note for aggregate analysis: `odom` poses are smooth but drift unboundedly,
// while `map` poses are absolute but jump on every relocalisation. Distance
// travelled must be integrated from odom, and a naive integral over map would
// be inflated by every localisation jump.
class OdometryTranslator : public ITranslator {
 public:
  const char* type_name() const override { return "nav_msgs/msg/Odometry"; }
  std::string fixed_series() const override { return "odom.*"; }
  void Translate(const rclcpp::SerializedMessage& message, const TranslationContext& ctx,
                 SessionRunner& runner) override;
  void Reset() override;

 private:
  RateGate gate_;
  bool have_last_position_ = false;
  double last_x_ = 0.0;
  double last_y_ = 0.0;
  double distance_travelled_ = 0.0;
};

}  // namespace cognitive3d_ros
