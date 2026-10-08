#include "cognitive3d_ros/translators/float32_translator.hpp"

#include <cmath>

namespace cognitive3d_ros {

void Float32Translator::Reset() { gates_.clear(); }

void Float32Translator::Translate(const rclcpp::SerializedMessage& message,
                                  const TranslationContext& ctx, SessionRunner& runner) {
  // Translators gate themselves; nothing upstream applies `rate_hz`.
  if (!gates_[ctx.topic].Accept(ctx.gate_time, ctx.rate_hz)) return;

  std_msgs::msg::Float32 msg;
  if (!Deserialize(message, &msg)) return;
  auto* session = runner.session();
  if (session == nullptr) return;

  const double value = msg.data;
  // NaN and inf are not readings. Recording them puts a hole in the series that
  // every consumer downstream has to special-case.
  if (!std::isfinite(value)) return;

  session->RecordSensor(SeriesNameFor(ctx, "value"), ctx.wire_time, value);
}

}  // namespace cognitive3d_ros
