#include "cognitive3d_ros/translators/battery_translator.hpp"

namespace cognitive3d_ros {

const char* BatteryTranslator::PowerSupplyStatusName(int status) {
  using B = sensor_msgs::msg::BatteryState;
  switch (status) {
    case B::POWER_SUPPLY_STATUS_CHARGING: return "charging";
    case B::POWER_SUPPLY_STATUS_DISCHARGING: return "discharging";
    case B::POWER_SUPPLY_STATUS_NOT_CHARGING: return "not_charging";
    case B::POWER_SUPPLY_STATUS_FULL: return "full";
    default: return "unknown";
  }
}

const char* BatteryTranslator::PowerSupplyHealthName(int health) {
  using B = sensor_msgs::msg::BatteryState;
  switch (health) {
    case B::POWER_SUPPLY_HEALTH_GOOD: return "good";
    case B::POWER_SUPPLY_HEALTH_OVERHEAT: return "overheat";
    case B::POWER_SUPPLY_HEALTH_DEAD: return "dead";
    case B::POWER_SUPPLY_HEALTH_OVERVOLTAGE: return "overvoltage";
    case B::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE: return "unspecified_failure";
    case B::POWER_SUPPLY_HEALTH_COLD: return "cold";
    case B::POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE: return "watchdog_expired";
    case B::POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE: return "safety_timer_expired";
    default: return "unknown";
  }
}

std::string BatteryTranslator::SeriesPrefix(const std::string& topic, bool first_seen) {
  if (first_seen) return "battery.";
  std::string qualified;
  qualified.reserve(topic.size());
  for (const char c : topic) {
    if (c != '/') {
      qualified.push_back(c);
    } else if (!qualified.empty()) {
      qualified.push_back('.');
    }
  }
  if (qualified.empty()) return "battery.";
  return "battery." + qualified + ".";
}

void BatteryTranslator::Reset() {
  by_topic_.clear();
  gates_.clear();
}

void BatteryTranslator::Translate(const rclcpp::SerializedMessage& message,
                                  const TranslationContext& ctx, SessionRunner& runner) {
  // A status or health change on a message the gate drops is recorded from the next message it
  // passes, if it still holds then: each is compared with the last value recorded.
  if (!gates_[ctx.topic].Accept(ctx.gate_time, ctx.rate_hz)) return;

  sensor_msgs::msg::BatteryState msg;
  if (!Deserialize(message, &msg)) return;
  auto* session = runner.session();
  if (session == nullptr) return;

  auto it = by_topic_.find(ctx.topic);
  if (it == by_topic_.end()) {
    State state;
    state.prefix = SeriesPrefix(ctx.topic, by_topic_.empty());
    it = by_topic_.emplace(ctx.topic, std::move(state)).first;
  }
  State& state = it->second;

  // percentage is 0.0 to 1.0 in the message definition, not 0 to 100. Scaling
  // it here rather than at the dashboard means the sensor track is readable
  // without anyone having to know that.
  if (std::isfinite(msg.percentage)) {
    session->RecordSensor(state.prefix + "percent", ctx.wire_time, msg.percentage * 100.0);
  }
  if (std::isfinite(msg.voltage)) {
    session->RecordSensor(state.prefix + "voltage", ctx.wire_time, msg.voltage);
  }
  if (std::isfinite(msg.current)) {
    session->RecordSensor(state.prefix + "current", ctx.wire_time, msg.current);
  }
  if (std::isfinite(msg.temperature)) {
    session->RecordSensor(state.prefix + "temperature", ctx.wire_time, msg.temperature);
  }

  // Status and health are enums, so they belong in events rather than in a
  // numeric track where the value would be meaningless.
  //
  // Each field advances only once its own event has been recorded, for the reason
  // DiagnosticsTranslator gives: the per-source cap discards, and a value
  // remembered for an event that was discarded loses the change outright -- the
  // next message finds nothing to say, and the two fields of one message do not
  // share a fate.
  if (!state.primed || msg.power_supply_status != state.last_status) {
    bool remember = true;
    if (state.primed) {
      c3d::Properties props;
      props.emplace_back("from", std::string(PowerSupplyStatusName(state.last_status)));
      props.emplace_back("to", std::string(PowerSupplyStatusName(msg.power_supply_status)));
      props.emplace_back("percent", static_cast<double>(msg.percentage) * 100.0);
      props.emplace_back("topic", ctx.topic);
      remember = RecordTranslatorEvent(runner, ctx, "robot.power.status_changed",
                                       std::move(props));
    }
    if (remember) state.last_status = msg.power_supply_status;
  }

  if (!state.primed || msg.power_supply_health != state.last_health) {
    // A health transition away from GOOD is a genuine incident, not telemetry.
    bool remember = true;
    if (state.primed) {
      c3d::Properties props;
      props.emplace_back("from", std::string(PowerSupplyHealthName(state.last_health)));
      props.emplace_back("to", std::string(PowerSupplyHealthName(msg.power_supply_health)));
      props.emplace_back("topic", ctx.topic);
      remember = RecordTranslatorEvent(runner, ctx, "robot.power.health_changed",
                                       std::move(props));
    }
    if (remember) state.last_health = msg.power_supply_health;
  }

  state.primed = true;
}

}  // namespace cognitive3d_ros
