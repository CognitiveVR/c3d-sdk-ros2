#include "cognitive3d_ros/translators/int32_multi_array_translator.hpp"

#include <string>

namespace cognitive3d_ros {

void Int32MultiArrayTranslator::Reset() { gates_.clear(); }

void Int32MultiArrayTranslator::Translate(const rclcpp::SerializedMessage& message,
                                          const TranslationContext& ctx, SessionRunner& runner) {
  if (!gates_[ctx.topic].Accept(ctx.gate_time, ctx.rate_hz)) return;

  std_msgs::msg::Int32MultiArray msg;
  if (!Deserialize(message, &msg)) return;
  auto* session = runner.session();
  if (session == nullptr) return;

  const std::string base = SeriesNameFor(ctx, "");
  session->RecordSensor(base + ".count", ctx.wire_time, static_cast<double>(msg.data.size()));

  const std::size_t n = msg.data.size() < kMaxElements ? msg.data.size() : kMaxElements;
  for (std::size_t i = 0; i < n; ++i) {
    session->RecordSensor(base + "." + std::to_string(i), ctx.wire_time,
                          static_cast<double>(msg.data[i]));
  }
}

}  // namespace cognitive3d_ros
