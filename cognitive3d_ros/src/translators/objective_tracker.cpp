#include "cognitive3d_ros/translators/objective_tracker.hpp"

#include <cstdio>

namespace cognitive3d_ros {

using GS = action_msgs::msg::GoalStatus;

const char* ObjectiveTracker::StatusName(std::int8_t status) {
  switch (status) {
    case GS::STATUS_ACCEPTED: return "accepted";
    case GS::STATUS_EXECUTING: return "executing";
    case GS::STATUS_CANCELING: return "canceling";
    case GS::STATUS_SUCCEEDED: return "succeeded";
    case GS::STATUS_CANCELED: return "canceled";
    case GS::STATUS_ABORTED: return "aborted";
    default: return "unknown";
  }
}

const char* ObjectiveTracker::StateName(std::int8_t status) {
  switch (status) {
    case GS::STATUS_ACCEPTED: return "started";
    case GS::STATUS_EXECUTING: return "executing";
    case GS::STATUS_CANCELING: return "canceling";
    case GS::STATUS_SUCCEEDED: return "succeeded";
    case GS::STATUS_CANCELED: return "canceled";
    case GS::STATUS_ABORTED: return "aborted";
    default: return "unknown";
  }
}

std::string ObjectiveTracker::EventName(const std::string& status_topic,
                                        const std::string& state) {
  const std::string action = ActionNameFromStatusTopic(status_topic);
  return (action.empty() ? std::string("action") : action) + "." + state;
}

bool ObjectiveTracker::IsTerminal(std::int8_t status) {
  return status == GS::STATUS_SUCCEEDED || status == GS::STATUS_CANCELED ||
         status == GS::STATUS_ABORTED;
}

std::string ObjectiveTracker::FormatUuid(const std::array<std::uint8_t, 16>& uuid) {
  char buffer[37];
  std::snprintf(buffer, sizeof(buffer),
                "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
                uuid[0], uuid[1], uuid[2], uuid[3], uuid[4], uuid[5], uuid[6], uuid[7],
                uuid[8], uuid[9], uuid[10], uuid[11], uuid[12], uuid[13], uuid[14], uuid[15]);
  return std::string(buffer);
}

void ObjectiveTracker::Reset() {
  goals_.clear();
  topic_last_wire_.clear();
}

void ObjectiveTracker::EmitTransition(const std::string& uuid, std::int8_t from,
                                      std::int8_t to, const GoalState& state,
                                      const TranslationContext& ctx, SessionRunner& runner) {
  c3d::Properties props;
  props.emplace_back("goal_id", uuid);
  // The full topic and the name derived from it. The topic identifies the server
  // on a namespaced fleet; the name is what the event is called, and a reader that
  // has only the name should not have to re-derive it.
  props.emplace_back("action", state.topic);
  props.emplace_back("action_name", ActionNameFromStatusTopic(state.topic));
  props.emplace_back("status", std::string(StatusName(to)));
  if (from != 0) props.emplace_back("previous_status", std::string(StatusName(from)));
  props.emplace_back("duration_s", ctx.wire_time - state.first_seen_wire);

  // Objectives need no dedicated backend API: the dashboard matches them by
  // custom event name. So the name is the contract, and it has to be stable.
  RecordTranslatorEvent(runner, ctx, EventName(state.topic, StateName(to)), std::move(props));
}

void ObjectiveTracker::Translate(const rclcpp::SerializedMessage& message,
                                 const TranslationContext& ctx, SessionRunner& runner) {
  action_msgs::msg::GoalStatusArray msg;
  if (!Deserialize(message, &msg)) return;

  topic_last_wire_[ctx.topic] = ctx.wire_time;

  for (const auto& entry : msg.status_list) {
    const std::string uuid = FormatUuid(entry.goal_info.goal_id.uuid);
    auto it = goals_.find(uuid);

    if (it == goals_.end()) {
      GoalState state;
      state.status = 0;
      state.first_seen_wire = ctx.wire_time;
      state.last_seen_wire = ctx.wire_time;
      state.topic = ctx.topic;
      // A goal whose FIRST sighting is already terminal reached that outcome before
      // this session could see it: the status topic is transient_local, so
      // subscribing replays the last goal the server published, from a run that may
      // predate this node. Seeded to the outcome and tracked, never emitted -- as a
      // transition it would put another run's `succeeded` in this session, where the
      // dashboard counts it as an objective that happened here. A goal first seen
      // ACCEPTED, EXECUTING or CANCELING is genuinely in progress and emits.
      if (IsTerminal(entry.status)) {
        state.status = entry.status;
        state.reported_terminal = true;
      }
      it = goals_.emplace(uuid, state).first;
    }

    GoalState& state = it->second;
    state.last_seen_wire = ctx.wire_time;

    if (entry.status != state.status) {
      const std::int8_t previous = state.status;
      // Advanced whether or not the event survives the rate cap. A status array
      // publishes a status, not a change, so there is no later message that would
      // re-offer this transition -- a suppressed one is lost rather than retried,
      // and holding the state back would only re-emit it as a transition from the
      // status the goal has already left.
      state.status = entry.status;
      EmitTransition(uuid, previous, entry.status, state, ctx, runner);
      if (IsTerminal(entry.status)) state.reported_terminal = true;
    }
  }

  // Sweep for goals that stopped appearing. The status topic is KEEP_LAST depth
  // 1 and a server drops a goal from the array once its result_timeout elapses,
  // so a goal can simply vanish mid-execution. Without this, an aborted
  // navigation that we happened to miss the terminal publication for stays
  // "executing" forever in the session record.
  //
  // A goal is absent only from an array ITS OWN server sent. A status topic
  // publishes when one of its goals changes state and is silent in between, so a
  // navigation goal that is merely still running leaves its topic quiet for the
  // whole drive while every other action on the robot keeps publishing. Aged
  // against the clock, those goals all vanish on the first unrelated
  // publication: reported `outcome_unknown` and erased, after which the real
  // terminal publication arrives as a first sighting that is already terminal
  // and seeds silently -- so the outcome is lost twice over, and a dashboard
  // objective on `<action>.succeeded` counts none of it.
  for (auto it = goals_.begin(); it != goals_.end();) {
    GoalState& state = it->second;
    const auto published = topic_last_wire_.find(state.topic);
    const bool absent_from_own_topic =
        published != topic_last_wire_.end() && published->second > state.last_seen_wire;
    const bool stale = absent_from_own_topic &&
                       (ctx.wire_time - state.last_seen_wire) > vanished_goal_timeout_s;
    if (!stale) {
      ++it;
      continue;
    }
    if (!state.reported_terminal && state.status != 0) {
      // The goal's OWN topic, never the one that happened to trigger the sweep.
      // Any action's publication ages out every action's goals, so the triggering
      // topic is unrelated to the goal being reported -- including for the
      // per-source rate cap, which is keyed on `ctx.topic` and would otherwise
      // spend the docking budget reporting a vanished navigation goal.
      //
      // The TIME and the POSITION stay the triggering message's: this event is a
      // conclusion drawn now, not a record of where the goal was lost.
      TranslationContext source = ctx;
      source.topic = state.topic;
      c3d::Properties props;
      props.emplace_back("goal_id", it->first);
      props.emplace_back("action", state.topic);
      props.emplace_back("action_name", ActionNameFromStatusTopic(state.topic));
      props.emplace_back("last_status", std::string(StatusName(state.status)));
      props.emplace_back("duration_s", state.last_seen_wire - state.first_seen_wire);
      props.emplace_back(
          "note",
          std::string("goal left the status array without reaching a terminal state; "
                      "the terminal publication was missed or result_timeout elapsed"));
      RecordTranslatorEvent(runner, source, EventName(state.topic, "outcome_unknown"),
                            std::move(props));
    }
    it = goals_.erase(it);
  }
}

}  // namespace cognitive3d_ros
