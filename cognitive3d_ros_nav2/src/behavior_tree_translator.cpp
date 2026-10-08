#include "cognitive3d_ros_nav2/behavior_tree_translator.hpp"

#include <algorithm>
#include <string>

namespace cognitive3d_ros_nav2 {
namespace {

// How long an unsampled window may stand before what is in it is read as the
// previous goal's leftovers rather than as this one's rate.
constexpr double kStaleWindowPeriods = 3.0;

}  // namespace

bool BehaviorTreeTranslator::IsRecoveryNode(const std::string& name) {
  // The Nav2 recovery BEHAVIOURS, and only those. The control nodes that wrap
  // them -- RecoveryNode, RoundRobin, ReactiveFallback, PipelineSequence -- run
  // and succeed on every cycle of a healthy tree, so counting them here reports a
  // robot recovering continuously from nothing, and finishing recoveries it never
  // started.
  //
  // BehaviorTreeLog carries the node's INSTANCE name, which is the registration
  // id only for a node the tree did not rename. Both spellings are therefore
  // listed: the ids nav2_behavior_tree registers, and the instance names Nav2's
  // own shipped trees give those nodes. The match is exact and not a substring
  // because Nav2 also ships CancelSpin, CancelWait and a CancelingControlAndWait
  // sequence, none of which is a recovery running.
  static const char* kRecoveryBehaviours[] = {
      "Spin",
      "BackUp",
      "Wait",
      "DriveOnHeading",
      "AssistedTeleop",
      "ClearEntireCostmap",
      "ClearCostmapExceptRegion",
      "ClearCostmapAroundRobot",
      "ClearCostmapAroundPose",
      "SpinRecovery",
      "BackUpRecovery",
      "WaitRecovery",
      "ClearLocalCostmap-Context",
      "ClearLocalCostmap-Subtree",
      "ClearGlobalCostmap-Context",
      "ClearGlobalCostmap-Subtree",
  };
  for (const char* candidate : kRecoveryBehaviours) {
    if (name == candidate) return true;
  }
  return false;
}

bool BehaviorTreeTranslator::IsConditionNode(const std::string& name) {
  // The Nav2 CONDITION plugins: the `<Condition>` section of
  // `nav2_behavior_tree/nav2_tree_nodes.xml`, which is the registration ids each
  // `libnav2_*_condition_bt_node.so` registers. A condition answers a question on
  // every tick and FAILURE is the answer "no", not a node failing: a tree asking
  // `GoalUpdated` at tick rate reports one `nav.bt.node_failed` for a goal that
  // was never updated, which is the question being answered correctly.
  //
  // Nav2's own shipped trees instantiate these unrenamed, so unlike the recovery
  // list there is no second spelling; the match is exact for the same reason.
  // Control and decorator nodes are deliberately absent -- a RecoveryNode or a
  // RateController failing is the tree giving up, which is the event.
  static const char* kConditions[] = {
      "AreErrorCodesPresent",
      "ArePosesNear",
      "DistanceTraveled",
      "GlobalUpdatedGoal",
      "GoalReached",
      "GoalUpdated",
      "InitialPoseReceived",
      "IsBatteryCharging",
      "IsBatteryLow",
      "IsPathValid",
      "IsStuck",
      "PathExpiringTimer",
      "TimeExpired",
      "TransformAvailable",
      "WouldAControllerRecoveryHelp",
      "WouldAPlannerRecoveryHelp",
      "WouldASmootherRecoveryHelp",
  };
  for (const char* candidate : kConditions) {
    if (name == candidate) return true;
  }
  return false;
}

bool BehaviorTreeTranslator::IsInterestingTransition(const std::string& previous,
                                                     const std::string& current) {
  // A condition node flapping SUCCESS/FAILURE at tick rate is noise. A node
  // failing, or starting to run after not running, is signal.
  if (current == "FAILURE") return true;
  if (previous == "RUNNING" && current == "SUCCESS") return true;
  if (previous == "IDLE" && current == "RUNNING") return true;
  return false;
}

std::string BehaviorTreeTranslator::SeriesName(const std::string& topic, bool first_seen) {
  static const std::string kBare = "nav.bt.transitions_per_second";
  if (first_seen) return kBare;
  std::string qualified;
  qualified.reserve(topic.size());
  for (const char c : topic) {
    if (c != '/') {
      qualified.push_back(c);
    } else if (!qualified.empty()) {
      qualified.push_back('.');
    }
  }
  if (qualified.empty()) return kBare;
  return "nav.bt." + qualified + ".transitions_per_second";
}

void BehaviorTreeTranslator::Reset() {
  emitted_ = 0;
  suppressed_ = 0;
  by_topic_.clear();
}

void BehaviorTreeTranslator::Translate(const rclcpp::SerializedMessage& message,
                                       const TranslationContext& ctx, SessionRunner& runner) {
  nav2_msgs::msg::BehaviorTreeLog msg;
  if (!cognitive3d_ros::Deserialize(message, &msg)) return;
  auto* session = runner.session();
  if (session == nullptr) return;

  auto it = by_topic_.find(ctx.topic);
  if (it == by_topic_.end()) {
    State fresh;
    fresh.series = SeriesName(ctx.topic, by_topic_.empty());
    it = by_topic_.emplace(ctx.topic, std::move(fresh)).first;
  }
  State& state = it->second;

  // How busy the tree is, as a rate. The default Nav2 tree replans at 1 Hz, so
  // the planner and controller nodes cycle IDLE>RUNNING>SUCCESS for as long as
  // the robot drives: that is a load channel and not a set of things that
  // happened. It is divided by the window it actually covered rather than
  // assumed to be one, because the publisher decides when a window closes.
  //
  // But bt_navigator publishes nothing at all between goals, so an open window
  // can span a silence no tree was ticking through. Whatever is left in it then
  // belongs to the goal that ended, and dividing the next goal's first burst by
  // the silence reports a tree that has just started as idle. Past a few periods
  // the count is dropped and the window re-anchored to this message, which is
  // also what a clock that steps backwards gets: measured against the burst, not
  // against the gap.
  const double period = ctx.rate_hz > 0.0 ? 1.0 / ctx.rate_hz : 0.0;
  const double stale_after = kStaleWindowPeriods * period;
  const bool reanchor =
      state.window_start < 0.0 || ctx.wire_time < state.window_start ||
      (stale_after > 0.0 && ctx.wire_time - state.window_start > stale_after);
  if (reanchor) {
    state.window_transitions = 0;
    state.window_start = ctx.wire_time;
  }
  state.window_transitions += msg.event_log.size();
  const double window = ctx.wire_time - state.window_start;
  if (window > 0.0 && window >= period) {
    session->RecordSensor(state.series, ctx.wire_time,
                          static_cast<double>(state.window_transitions) / window);
    state.window_transitions = 0;
    state.window_start = ctx.wire_time;
  }

  for (const auto& change : msg.event_log) {
    const std::string& node = change.node_name;
    const std::string& previous = change.previous_status;
    const std::string& current = change.current_status;

    const bool recovery = IsRecoveryNode(node);
    // A recovery behaviour is reset to IDLE once it is done and BT.CPP logs that
    // reset, SUCCESS>IDLE and FAILURE>IDLE alike. Read as an outcome it gives
    // every recovery a second one, and reports a Spin that failed as finished.
    //
    // RUNNING>IDLE is not that reset. BT.CPP logs it when the tree HALTS a node
    // that is still running -- the goal was cancelled or preempted mid-recovery --
    // and it is the only record of how that recovery ended. Dropped with the rest,
    // a started recovery would have no outcome at all.
    const bool halted = recovery && current == "IDLE" && previous == "RUNNING";
    if (recovery && current == "IDLE" && !halted) {
      ++suppressed_;
      continue;
    }

    // Name-only where the distro's message has no uid; the header says what that loses.
    const std::optional<std::int64_t> uid = UidOf(change);
    const std::string key = uid ? node + "#" + std::to_string(*uid) : node;
    const std::string transition = previous + ">" + current;

    if (!recovery) {
      // Dropped whole rather than at the FAILURE test below: `node_failed` is the
      // only event a non-recovery node can produce, so a condition has nothing
      // left to emit, and carrying it through the dedup map costs a key per node
      // on the tree's busiest transition.
      if (IsConditionNode(node)) {
        ++suppressed_;
        continue;
      }
      if (!IsInterestingTransition(previous, current)) {
        ++suppressed_;
        continue;
      }
      // Suppress an identical repeat of the same transition for the same node.
      auto seen = state.last_transition.find(key);
      if (seen != state.last_transition.end() && seen->second == transition) {
        ++suppressed_;
        continue;
      }
    }
    state.last_transition[key] = transition;

    // What is left of an ordinary node's cycle is the rate above. Only a failure
    // is an occurrence, and the state machine still tracks the rest of the cycle
    // so that a node failing again after running again is a second failure.
    if (!recovery && current != "FAILURE") {
      ++suppressed_;
      continue;
    }

    c3d::Properties props;
    props.emplace_back("node", node);
    props.emplace_back("from", previous);
    props.emplace_back("to", current);
    if (uid) props.emplace_back("uid", *uid);
    props.emplace_back("topic", ctx.topic);
    // The internal stamp is wall clock set by BehaviorTree.CPP, not ROS time.
    // Under use_sim_time it disagrees with everything else in the session, so it
    // is recorded as data rather than used as the event time.
    props.emplace_back("bt_stamp",
                       static_cast<double>(change.timestamp.sec) +
                           static_cast<double>(change.timestamp.nanosec) * 1e-9);

    std::string name = "nav.bt.node_failed";
    if (recovery) {
      if (current == "RUNNING") {
        name = "nav.recovery.started";
      } else if (halted) {
        name = "nav.recovery.halted";
      } else {
        name = (current == "FAILURE") ? "nav.recovery.failed" : "nav.recovery.finished";
      }
    }

    // Only what actually reached the session. The per-source cap discards, and a
    // count that included the discards would report a session as carrying
    // transitions it does not have. The dedup is rolled back with it: remembering
    // a transition the session never got swallows the repeat that would have
    // carried it, and a node stuck failing goes unreported from then on.
    if (cognitive3d_ros::RecordTranslatorEvent(runner, ctx, name, std::move(props))) {
      ++emitted_;
    } else {
      state.last_transition.erase(key);
    }
  }
}

}  // namespace cognitive3d_ros_nav2
