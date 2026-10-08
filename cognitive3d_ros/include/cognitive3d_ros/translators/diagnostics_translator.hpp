#pragma once

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <map>
#include <string>

#include "cognitive3d_ros/translators.hpp"

namespace cognitive3d_ros {

// diagnostic_msgs/DiagnosticArray to Cognitive3D custom events, ON STATE CHANGE
// ONLY.
//
// This is the highest signal-to-bandwidth source on a robot, and also the
// easiest to ruin. /diagnostics publishes at 1 Hz with every status in every
// message, so streaming it verbatim produces thousands of identical events per
// hour and buries the three that mattered. We emit only when a named status
// changes level, which turns it into an event stream about failures rather than
// a log of everything being fine.
//
// ONE instance serves every DiagnosticArray topic on the robot, because the
// registry keys translators by TYPE and `auto_subscribe_standard_types` takes the
// type wherever it appears. A component name is unique only within its own
// publisher -- `motor_driver` at WARN on one arm and at OK on another is two
// facts, not a transition -- so the level is remembered per (topic, name).
class DiagnosticsTranslator : public ITranslator {
 public:
  const char* type_name() const override { return "diagnostic_msgs/msg/DiagnosticArray"; }
  void Translate(const rclcpp::SerializedMessage& message, const TranslationContext& ctx,
                 SessionRunner& runner) override;
  void Reset() override;

  static const char* LevelName(unsigned char level);

 private:
  // topic to status name to last observed level.
  std::map<std::string, std::map<std::string, unsigned char>> last_level_;
};

}  // namespace cognitive3d_ros
