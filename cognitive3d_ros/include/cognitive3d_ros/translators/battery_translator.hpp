#pragma once

#include <sensor_msgs/msg/battery_state.hpp>

#include <map>
#include <string>

#include "cognitive3d_ros/translators.hpp"

namespace cognitive3d_ros {

// sensor_msgs/BatteryState to Cognitive3D sensors, plus an event on any change
// of power supply status.
//
// One trap worth naming: `percentage` is 0.0 to 1.0, NOT 0 to 100. Publishing
// it raw produces a battery track that never leaves the bottom of the chart.
//
// ONE instance serves every BatteryState topic on the robot, because the
// registry keys translators by TYPE and `auto_subscribe_standard_types` takes
// the type wherever it appears. So all of this translator's state is keyed by
// topic: a shared `last_status_` would make two batteries at different charge
// states emit `robot.power.status_changed` on every single message, and a shared
// rate gate would let one battery's messages starve the other's.
class BatteryTranslator : public ITranslator {
 public:
  const char* type_name() const override { return "sensor_msgs/msg/BatteryState"; }
  void Translate(const rclcpp::SerializedMessage& message, const TranslationContext& ctx,
                 SessionRunner& runner) override;
  void Reset() override;

  static const char* PowerSupplyStatusName(int status);
  static const char* PowerSupplyHealthName(int health);

  // The sensor-series prefix a topic's values are recorded under. The FIRST
  // BatteryState topic a session sees keeps the bare `battery.` names, so a
  // single-battery robot -- which is nearly all of them -- records unqualified
  // series; every further topic is qualified by its own name, because
  // interleaved values on one series describe neither battery.
  static std::string SeriesPrefix(const std::string& topic, bool first_seen);

 private:
  struct State {
    int last_status = -1;
    int last_health = -1;
    bool primed = false;
    std::string prefix;
  };
  std::map<std::string, State> by_topic_;
  std::map<std::string, RateGate> gates_;
};

}  // namespace cognitive3d_ros
