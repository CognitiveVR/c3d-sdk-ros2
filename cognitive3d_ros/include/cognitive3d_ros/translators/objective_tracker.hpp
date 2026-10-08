#pragma once

#include <action_msgs/msg/goal_status_array.hpp>

#include <array>
#include <cstdint>
#include <map>
#include <string>

#include "cognitive3d_ros/action_discovery.hpp"
#include "cognitive3d_ros/translators.hpp"

namespace cognitive3d_ros {

// action_msgs/GoalStatusArray to Cognitive3D objectives.
//
// Objectives need no dedicated API: the dashboard matches them by custom event
// NAME, so a well-named event is an objective. The name carries the ACTION, not a
// fixed word, because a robot runs several actions at once and an objective that
// merged docking with navigation would measure neither.
//
// The reason this is a state machine rather than a mapping: the action status
// QoS is KEEP_LAST with DEPTH 1, and goals leave the array entirely once the
// server's result_timeout elapses. So a momentarily slow subscriber can miss
// intermediate publications, and a goal can vanish rather than terminate. We
// keep our own per-UUID record and emit transitions, including a synthesised
// outcome for a goal that disappears while still executing.
class ObjectiveTracker : public ITranslator {
 public:
  const char* type_name() const override { return "action_msgs/msg/GoalStatusArray"; }
  void Translate(const rclcpp::SerializedMessage& message, const TranslationContext& ctx,
                 SessionRunner& runner) override;
  void Reset() override;

  static const char* StatusName(std::int8_t status);
  // The word the event NAME carries, which is not always the word the `status`
  // property carries: an ACCEPTED goal is an objective that STARTED.
  static const char* StateName(std::int8_t status);
  static bool IsTerminal(std::int8_t status);
  static std::string FormatUuid(const std::array<std::uint8_t, 16>& uuid);

  // `<action>.<state>`, the action taken from the status topic's own path. A
  // topic that names no action falls back to `action.<state>` rather than to
  // nothing, so an event is never emitted with a leading dot.
  static std::string EventName(const std::string& status_topic, const std::string& state);

  // How long a goal may be absent from its OWN topic's arrays before the absence
  // is reported. Silence on that topic is not absence: see the sweep.
  double vanished_goal_timeout_s = 30.0;

  struct GoalState {
    std::int8_t status = 0;
    double first_seen_wire = 0.0;
    double last_seen_wire = 0.0;
    bool reported_terminal = false;
    // The status topic this goal arrived on. ONE tracker instance serves every
    // action on the robot, and the sweep below runs on whichever topic published
    // last -- so without this a vanished docking goal is reported as a vanished
    // navigation goal, from the name down.
    std::string topic;
  };

  const std::map<std::string, GoalState>& goals() const { return goals_; }

 private:
  void EmitTransition(const std::string& uuid, std::int8_t from, std::int8_t to,
                      const GoalState& state, const TranslationContext& ctx,
                      SessionRunner& runner);
  // Keyed on the UUID alone although one instance serves every action topic: rcl
  // draws a goal id at random per goal, so two servers cannot mint the same one,
  // and each goal carries the topic it arrived on. The other translators of a
  // shared type key their state by topic because their keys -- a component name,
  // a battery's own status -- are unique only within one publisher.
  std::map<std::string, GoalState> goals_;
  // When each status topic last published anything. A goal is absent only from
  // an array its own server actually sent, so this is what the sweep ages a goal
  // against -- never the clock, and never the topic that triggered the sweep.
  std::map<std::string, double> topic_last_wire_;
};

}  // namespace cognitive3d_ros
