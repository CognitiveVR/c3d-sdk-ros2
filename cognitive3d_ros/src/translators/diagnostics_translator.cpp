#include "cognitive3d_ros/translators/diagnostics_translator.hpp"

namespace cognitive3d_ros {

const char* DiagnosticsTranslator::LevelName(unsigned char level) {
  using D = diagnostic_msgs::msg::DiagnosticStatus;
  switch (level) {
    case D::OK: return "OK";
    case D::WARN: return "WARN";
    case D::ERROR: return "ERROR";
    case D::STALE: return "STALE";
    default: return "UNKNOWN";
  }
}

void DiagnosticsTranslator::Reset() { last_level_.clear(); }

void DiagnosticsTranslator::Translate(const rclcpp::SerializedMessage& message,
                                      const TranslationContext& ctx, SessionRunner& runner) {
  diagnostic_msgs::msg::DiagnosticArray msg;
  if (!Deserialize(message, &msg)) return;
  auto* session = runner.session();
  if (session == nullptr) return;

  auto& levels = last_level_[ctx.topic];
  for (const auto& status : msg.status) {
    const auto it = levels.find(status.name);
    const bool first_sight = (it == levels.end());
    const unsigned char previous = first_sight ? diagnostic_msgs::msg::DiagnosticStatus::OK
                                               : it->second;

    // Emit on transitions only. A robot publishes /diagnostics at 1 Hz with
    // every status in every message; forwarding all of it produces thousands of
    // identical events per hour and buries the three that mattered.
    const bool changed = !first_sight && status.level != previous;
    // First sight of a status that is already unhealthy is worth an event too,
    // otherwise a fault that predates the session is invisible.
    const bool born_unhealthy =
        first_sight && status.level != diagnostic_msgs::msg::DiagnosticStatus::OK;

    // Advanced only once the transition has actually been recorded. The
    // per-source rate cap discards events, and a level remembered for one that
    // was discarded loses the degradation outright: the matching `recovered`
    // still fires, and the session says a component recovered from nothing.
    bool remember = true;
    if (changed || born_unhealthy) {
      c3d::Properties props;
      props.emplace_back("component", status.name);
      props.emplace_back("level", std::string(LevelName(status.level)));
      if (changed) props.emplace_back("previous_level", std::string(LevelName(previous)));
      props.emplace_back("message", status.message);
      if (!status.hardware_id.empty()) props.emplace_back("hardware_id", status.hardware_id);
      props.emplace_back("topic", ctx.topic);

      // Diagnostic key/value pairs are arbitrary strings from the driver. Take a
      // bounded number so one chatty component cannot dominate the payload.
      int taken = 0;
      for (const auto& kv : status.values) {
        if (taken++ >= 8) break;
        props.emplace_back("kv." + kv.key, kv.value);
      }

      const bool recovering = status.level == diagnostic_msgs::msg::DiagnosticStatus::OK;
      remember = RecordTranslatorEvent(
          runner, ctx, recovering ? "robot.diagnostic.recovered" : "robot.diagnostic.degraded",
          std::move(props));
    }

    if (remember) levels[status.name] = status.level;
  }
}

}  // namespace cognitive3d_ros
