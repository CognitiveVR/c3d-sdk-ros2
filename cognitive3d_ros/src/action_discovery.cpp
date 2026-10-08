#include "cognitive3d_ros/action_discovery.hpp"

#include <algorithm>

namespace cognitive3d_ros {
namespace {

const char kStatusSuffix[] = "/_action/status";
// sizeof counts the terminator.
constexpr std::size_t kStatusSuffixLen = sizeof(kStatusSuffix) - 1;

}  // namespace

std::string ActionNameFromStatusTopic(const std::string& topic) {
  // Strictly longer, not merely as long: a bare `/_action/status` names no action.
  if (topic.size() <= kStatusSuffixLen) return std::string();
  if (topic.compare(topic.size() - kStatusSuffixLen, kStatusSuffixLen, kStatusSuffix) != 0) {
    return std::string();
  }
  const std::string path = topic.substr(0, topic.size() - kStatusSuffixLen);
  const std::string::size_type slash = path.rfind('/');
  return slash == std::string::npos ? path : path.substr(slash + 1);
}

const std::vector<std::string>& DefaultActionNames() {
  static const std::vector<std::string> kNames = {
      // Nav2. `follow_path` is absent for the same reason the planner and smoother
      // actions are: the default behaviour tree replans about once a second and
      // every replan preempts the running FollowPath goal, which the action server
      // terminates as ABORTED. A healthy run produces hundreds of
      // `follow_path.aborted` and no failure.
      "navigate_to_pose", "navigate_through_poses", "follow_waypoints",
      "spin", "backup", "drive_on_heading", "wait", "assisted_teleop", "dock_robot",
      "undock_robot",
      // iRobot Create 3.
      "dock", "undock", "drive_distance", "drive_arc", "rotate_angle",
      "navigate_to_position", "wall_follow",
  };
  return kNames;
}

const std::vector<std::string>& ReplanActionNames() {
  static const std::vector<std::string> kNames = {
      "compute_path_to_pose", "compute_path_through_poses", "smooth_path", "follow_path"};
  return kNames;
}

bool InDiscoveryNamespace(const std::string& topic, const std::string& node_namespace) {
  if (node_namespace.empty() || node_namespace == "/") return true;
  return topic.rfind(node_namespace + "/", 0) == 0;
}

ActionTopicVerdict ClassifyActionTopic(const std::string& topic, bool allowlisted,
                                       bool ingest_refused, bool discovery_enabled,
                                       const std::vector<std::string>& action_names,
                                       const std::string& node_namespace) {
  ActionTopicVerdict verdict;
  verdict.topic = topic;
  verdict.action_name = ActionNameFromStatusTopic(topic);

  // First, because it beats every rule below: `ingest: refuse` creates no
  // subscription, so nothing downstream can record this action however it is
  // named.
  if (ingest_refused) {
    verdict.reason = "ingest is `refuse` in `topic_config`; no subscription is created";
    return verdict;
  }
  // The allowlist wins over every switch below, so an operator who named a topic
  // gets it whatever the discovery settings say.
  if (allowlisted) {
    verdict.enrolled = true;
    verdict.reason = "named in `topics`";
    return verdict;
  }
  if (verdict.action_name.empty()) {
    verdict.reason = "not an <action>/_action/status topic; only `topics` reaches it";
    return verdict;
  }
  if (!discovery_enabled) {
    verdict.reason = "action_discovery is false";
    return verdict;
  }
  if (!InDiscoveryNamespace(topic, node_namespace)) {
    verdict.reason = "outside this node's namespace " + node_namespace +
                     "; only `topics` reaches it";
    return verdict;
  }
  verdict.enrolled = std::find(action_names.begin(), action_names.end(),
                               verdict.action_name) != action_names.end();
  if (verdict.enrolled) {
    verdict.reason = "'" + verdict.action_name + "' is in `action_names`";
    return verdict;
  }
  const auto& replan = ReplanActionNames();
  verdict.replan_action =
      action_names == DefaultActionNames() &&
      std::find(replan.begin(), replan.end(), verdict.action_name) != replan.end();
  verdict.reason = "'" + verdict.action_name + "' is not in `action_names`" +
                   (verdict.replan_action
                        ? std::string(": Nav2 runs it on every replan, so the built-in list "
                                      "leaves it out")
                        : std::string());
  return verdict;
}

}  // namespace cognitive3d_ros
