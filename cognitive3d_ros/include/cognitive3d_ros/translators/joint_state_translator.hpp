#pragma once

#include <sensor_msgs/msg/joint_state.hpp>

#include <map>
#include <string>

#include "cognitive3d_ros/translators.hpp"

namespace cognitive3d_ros {

// sensor_msgs/JointState to Cognitive3D sensors, decimated hard.
//
// Joint states publish at 20 to 100 Hz with one array entry per joint. A
// 12-joint arm at 50 Hz is 600 scalar samples per second, which is more than
// the entire XR session model was designed to carry. The core's change gating
// removes most of it; this decimates before that so we do not even build the
// samples.
//
// Effort is the interesting channel for failure analysis, not position: a joint
// working harder than usual to hold the same pose is a bearing about to go.
class JointStateTranslator : public ITranslator {
 public:
  const char* type_name() const override { return "sensor_msgs/msg/JointState"; }
  std::string fixed_series() const override { return "joint.* and joints.*"; }
  void Translate(const rclcpp::SerializedMessage& message, const TranslationContext& ctx,
                 SessionRunner& runner) override;
  void Reset() override;

  // Per-joint channels are opt-in. On by default: aggregate effort magnitude,
  // which is one scalar regardless of joint count.
  bool record_per_joint_effort = false;
  bool record_per_joint_position = false;

 private:
  RateGate gate_;
};

}  // namespace cognitive3d_ros
