#include "cognitive3d_ros/translators/imu_translator.hpp"

#include <cmath>

namespace cognitive3d_ros {
namespace {

// REP-145: a covariance whose first element is -1 means "this estimate is not
// produced", and the matching data fields are then meaningless zeros.
bool EstimateIsPresent(double covariance_first_element) {
  return covariance_first_element != -1.0;
}

double Degrees(double radians) { return radians * 180.0 / M_PI; }

}  // namespace

void ImuTranslator::Reset() { gate_.Reset(); }

void ImuTranslator::Translate(const rclcpp::SerializedMessage& message,
                              const TranslationContext& ctx, SessionRunner& runner) {
  if (!gate_.Accept(ctx.gate_time, ctx.rate_hz)) return;

  sensor_msgs::msg::Imu msg;
  if (!Deserialize(message, &msg)) return;
  auto* session = runner.session();
  if (session == nullptr) return;

  if (EstimateIsPresent(msg.orientation_covariance[0])) {
    const double x = msg.orientation.x, y = msg.orientation.y;
    const double z = msg.orientation.z, w = msg.orientation.w;
    const double norm = std::sqrt(x * x + y * y + z * z + w * w);
    // A degenerate quaternion would put a NaN on the wire, which is accepted
    // with a 200 and renders as a gap nobody can trace back to here.
    if (std::isfinite(norm) && norm > 1e-9) {
      const double nx = x / norm, ny = y / norm, nz = z / norm, nw = w / norm;
      const double sin_pitch = std::max(-1.0, std::min(1.0, 2.0 * (nw * ny - nz * nx)));
      session->RecordSensor("imu.roll_degrees", ctx.wire_time,
                            Degrees(std::atan2(2.0 * (nw * nx + ny * nz),
                                               1.0 - 2.0 * (nx * nx + ny * ny))));
      session->RecordSensor("imu.pitch_degrees", ctx.wire_time, Degrees(std::asin(sin_pitch)));
    }
  }

  // Yaw is deliberately absent even though the quaternion carries one: an IMU's
  // heading is in its own arbitrary reference, so it is not comparable between
  // runs, while the RATE is a genuine measurement of what the robot did.
  if (EstimateIsPresent(msg.angular_velocity_covariance[0]) &&
      std::isfinite(msg.angular_velocity.z)) {
    session->RecordSensor("imu.yaw_rate_degrees_per_second", ctx.wire_time,
                          Degrees(msg.angular_velocity.z));
  }

  if (EstimateIsPresent(msg.linear_acceleration_covariance[0])) {
    const double ax = msg.linear_acceleration.x;
    const double ay = msg.linear_acceleration.y;
    const double az = msg.linear_acceleration.z;
    if (std::isfinite(ax) && std::isfinite(ay) && std::isfinite(az)) {
      // Magnitude, so the channel means the same thing whatever the sensor's
      // mounting orientation. It includes gravity, as the message defines it.
      session->RecordSensor("imu.acceleration_meters_per_second_squared", ctx.wire_time,
                            std::sqrt(ax * ax + ay * ay + az * az));
    }
  }
}

}  // namespace cognitive3d_ros
