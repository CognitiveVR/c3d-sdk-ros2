#pragma once

#include <nav2_msgs/msg/behavior_tree_log.hpp>

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <type_traits>
#include <utility>

#include "cognitive3d_ros/translators.hpp"

namespace cognitive3d_ros_nav2 {

using cognitive3d_ros::ITranslator;
using cognitive3d_ros::SessionRunner;
using cognitive3d_ros::TranslationContext;

// Humble's BehaviorTreeStatusChange has no `uid`; every later distro's does. Detected on the type
// itself, because no compile-time macro names the distro and the build environment's ROS_DISTRO is
// wrong for a cross-build.
template <typename T, typename = void>
struct HasUid : std::false_type {};
template <typename T>
struct HasUid<T, std::void_t<decltype(std::declval<T&>().uid)>> : std::true_type {};
inline constexpr bool kStatusChangeHasUid =
    HasUid<nav2_msgs::msg::BehaviorTreeStatusChange>::value;

template <typename T>
std::optional<std::int64_t> UidOf(const T& change) {
  if constexpr (HasUid<T>::value) {
    return static_cast<std::int64_t>(change.uid);
  } else {
    return std::nullopt;
  }
}

// nav2_msgs/BehaviorTreeLog to Cognitive3D custom events.
//
// This is the single richest Nav2 telemetry source. bt_navigator publishes every
// behaviour tree node transition, by name, with previous and current status. The
// recovery behaviours (Spin, BackUp, ClearCostmap, Wait, DriveOnHeading,
// AssistedTeleop) appear directly as nodes going RUNNING to SUCCESS or FAILURE,
// which is exactly the "what did the robot try before it gave up" question that
// autonomy failure analysis is about.
//
// Two things this has to handle:
//
// 1. Volume. A behaviour tree ticks at 10 to 30 Hz and a Nav2 tree has dozens of
//    nodes, so the raw log is thousands of transitions per minute: a condition
//    node flapping between SUCCESS and FAILURE, and -- because the default tree
//    replans at 1 Hz -- the planner and controller nodes cycling
//    IDLE>RUNNING>SUCCESS for as long as the robot drives. None of that is a
//    thing that happened, and as events it buries the recoveries in its own
//    session. It is recorded as the `nav.bt.transitions_per_second` sensor
//    series instead, and the events are kept for a recovery behaviour running
//    and for any node failing.
//
// 2. Time. The per-event timestamps inside BehaviorTreeLog are WALL CLOCK, set
//    by BehaviorTree.CPP, while the outer header stamp is ROS time. Under
//    use_sim_time these disagree by decades. We use the context's wire time and
//    record the internal stamp separately rather than trusting it.
class BehaviorTreeTranslator : public ITranslator {
 public:
  const char* type_name() const override { return "nav2_msgs/msg/BehaviorTreeLog"; }
  void Translate(const rclcpp::SerializedMessage& message, const TranslationContext& ctx,
                 SessionRunner& runner) override;
  void Reset() override;

  // The recovery behaviours, which produce an event for the run and exactly one
  // for the outcome -- finished, failed or halted. The control nodes that wrap
  // them are not among them.
  static bool IsRecoveryNode(const std::string& node_name);

  // The Nav2 condition plugins, which are excluded from `nav.bt.node_failed`.
  // A condition answers a question every tick and FAILURE is the answer "no".
  static bool IsConditionNode(const std::string& node_name);

  static bool IsInterestingTransition(const std::string& previous, const std::string& current);

  // The series a topic's churn is counted into. The FIRST BehaviorTreeLog topic a
  // session sees keeps the bare `nav.bt.transitions_per_second`, so a robot with
  // one navigator -- nearly all of them -- records the unqualified series, and every
  // further topic is qualified by its own name, because two trees interleaved on
  // one series describe neither.
  static std::string SeriesName(const std::string& topic, bool first_seen);

  std::uint64_t emitted() const { return emitted_; }
  std::uint64_t suppressed() const { return suppressed_; }

 private:
  // ONE instance serves every BehaviorTreeLog topic, because the registry keys
  // translators by TYPE, so everything that describes one tree is keyed by topic.
  struct State {
    // Suppresses an identical transition repeating inside one tick burst. Keyed by
    // node name AND uid: Nav2's default tree names a RecoveryNode `FollowPath`
    // around the FollowPath action, so the name alone is two nodes.
    //
    // Where the message has no uid (Humble), the key is the name alone and two
    // nodes sharing a name cannot be told apart. When the FollowPath action fails
    // and the wrapper of the same name then gives up, the wrapper's failure is an
    // identical repeat of the action's and is dropped: one `nav.bt.node_failed` is
    // lost. The outer recovery's failure and the navigation action's own outcome
    // are still recorded. Not deduplicating instead would let one node failing at
    // tick rate spend the topic's whole event budget.
    std::map<std::string, std::string> last_transition;
    // The open sensor window: transitions counted so far and when it started, with
    // a negative start meaning no message has arrived yet.
    std::uint64_t window_transitions = 0;
    double window_start = -1.0;
    std::string series;
  };

  std::uint64_t emitted_ = 0;
  std::uint64_t suppressed_ = 0;
  std::map<std::string, State> by_topic_;
};

}  // namespace cognitive3d_ros_nav2
