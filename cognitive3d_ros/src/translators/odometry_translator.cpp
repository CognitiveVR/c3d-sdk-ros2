#include "cognitive3d_ros/translators/odometry_translator.hpp"

#include <cmath>

namespace cognitive3d_ros {

void OdometryTranslator::Reset() {
  gate_.Reset();
  have_last_position_ = false;
  distance_travelled_ = 0.0;
}

void OdometryTranslator::Translate(const rclcpp::SerializedMessage& message,
                                   const TranslationContext& ctx, SessionRunner& runner) {
  if (!gate_.Accept(ctx.gate_time, ctx.rate_hz)) return;

  nav_msgs::msg::Odometry msg;
  if (!Deserialize(message, &msg)) return;
  auto* session = runner.session();
  if (session == nullptr) return;

  const double vx = msg.twist.twist.linear.x;
  const double vy = msg.twist.twist.linear.y;
  const double vz = msg.twist.twist.linear.z;
  const double wz = msg.twist.twist.angular.z;

  if (std::isfinite(vx) && std::isfinite(vy) && std::isfinite(vz)) {
    session->RecordSensor("odom.speed", ctx.wire_time, std::sqrt(vx * vx + vy * vy + vz * vz));
  }
  if (std::isfinite(wz)) session->RecordSensor("odom.yaw_rate", ctx.wire_time, wz);

  // Distance travelled is integrated from odom on purpose. Integrating it from
  // a map-frame pose instead would add the length of every relocalisation jump
  // to the total.
  const double x = msg.pose.pose.position.x;
  const double y = msg.pose.pose.position.y;
  if (std::isfinite(x) && std::isfinite(y)) {
    if (have_last_position_) {
      const double dx = x - last_x_;
      const double dy = y - last_y_;
      const double step = std::sqrt(dx * dx + dy * dy);
      // A large step between samples is a reset or a teleport, not motion.
      if (step < 10.0) distance_travelled_ += step;
    }
    last_x_ = x;
    last_y_ = y;
    have_last_position_ = true;
    session->RecordSensor("odom.distance_travelled", ctx.wire_time, distance_travelled_);
  }
}

}  // namespace cognitive3d_ros
