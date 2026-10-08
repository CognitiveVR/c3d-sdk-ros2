#include "cognitive3d_ros/translators/joint_state_translator.hpp"

#include <cmath>

namespace cognitive3d_ros {

void JointStateTranslator::Reset() { gate_.Reset(); }

void JointStateTranslator::Translate(const rclcpp::SerializedMessage& message,
                                     const TranslationContext& ctx, SessionRunner& runner) {
  // Decimate before deserializing. At 100 Hz this skips most of the work
  // entirely rather than paying for a deserialize and then throwing it away.
  if (!gate_.Accept(ctx.gate_time, ctx.rate_hz)) return;

  sensor_msgs::msg::JointState msg;
  if (!Deserialize(message, &msg)) return;
  auto* session = runner.session();
  if (session == nullptr) return;

  // Arrays may legitimately be empty and need not be the same length as `name`.
  const std::size_t joints = msg.name.size();

  double effort_sum_sq = 0.0;
  double max_effort = 0.0;
  std::size_t effort_count = 0;
  for (std::size_t i = 0; i < msg.effort.size(); ++i) {
    const double e = msg.effort[i];
    if (!std::isfinite(e)) continue;
    effort_sum_sq += e * e;
    max_effort = std::max(max_effort, std::abs(e));
    ++effort_count;
    if (record_per_joint_effort && i < joints) {
      session->RecordSensor("joint.effort." + msg.name[i], ctx.wire_time, e);
    }
  }

  double velocity_max = 0.0;
  for (double v : msg.velocity) {
    if (std::isfinite(v)) velocity_max = std::max(velocity_max, std::abs(v));
  }

  if (record_per_joint_position) {
    for (std::size_t i = 0; i < msg.position.size() && i < joints; ++i) {
      if (std::isfinite(msg.position[i])) {
        session->RecordSensor("joint.position." + msg.name[i], ctx.wire_time, msg.position[i]);
      }
    }
  }

  // Aggregates are one scalar each regardless of joint count, which is what
  // makes this affordable on a 12-joint arm at 50 Hz.
  if (effort_count > 0) {
    session->RecordSensor("joints.effort_rms", ctx.wire_time,
                          std::sqrt(effort_sum_sq / static_cast<double>(effort_count)));
    session->RecordSensor("joints.effort_max", ctx.wire_time, max_effort);
  }
  if (!msg.velocity.empty()) {
    session->RecordSensor("joints.velocity_max", ctx.wire_time, velocity_max);
  }
  session->RecordSensor("joints.count", ctx.wire_time, static_cast<double>(joints));
}

}  // namespace cognitive3d_ros
