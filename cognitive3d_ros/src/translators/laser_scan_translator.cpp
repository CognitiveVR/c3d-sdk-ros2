#include "cognitive3d_ros/translators/laser_scan_translator.hpp"

#include <cmath>

namespace cognitive3d_ros {

void LaserScanTranslator::Reset() { gate_.Reset(); }

void LaserScanTranslator::Translate(const rclcpp::SerializedMessage& message,
                                    const TranslationContext& ctx, SessionRunner& runner) {
  if (!gate_.Accept(ctx.gate_time, ctx.rate_hz)) return;

  sensor_msgs::msg::LaserScan msg;
  if (!Deserialize(message, &msg)) return;
  auto* session = runner.session();
  if (session == nullptr) return;
  if (msg.ranges.empty()) return;

  // The message's own limits, not fixed ones: a return outside them is what the
  // driver means by "nothing there", and inf is only the commonest spelling of
  // it. Taking those readings as distances puts a 0.0 m obstacle in the record
  // of a robot standing in an empty room.
  double nearest = 0.0;
  double sum = 0.0;
  std::size_t valid = 0;
  for (float raw : msg.ranges) {
    const double r = raw;
    if (!std::isfinite(r) || r < msg.range_min || r > msg.range_max) continue;
    if (valid == 0 || r < nearest) nearest = r;
    sum += r;
    ++valid;
  }

  if (valid > 0) {
    session->RecordSensor("scan.range_min_meters", ctx.wire_time, nearest);
    session->RecordSensor("scan.range_mean_meters", ctx.wire_time,
                          sum / static_cast<double>(valid));
  }
  // Recorded even when nothing was valid, because zero IS the reading: a scan
  // that returns nothing is the case this channel exists to make visible.
  session->RecordSensor("scan.valid_fraction", ctx.wire_time,
                        static_cast<double>(valid) / static_cast<double>(msg.ranges.size()));
}

}  // namespace cognitive3d_ros
