#pragma once

#include <string>
#include <vector>

namespace cognitive3d_ros {

// The type every ROS 2 action server publishes its goal states on.
inline constexpr const char* kGoalStatusArrayType = "action_msgs/msg/GoalStatusArray";

// The action NAME a status topic belongs to: the last path segment before
// `/_action/status`, or empty when the topic is not an action status topic at all.
//
// The NAME rather than the full path, because a fleet namespaces its actions:
// `/robot1/dock/_action/status` and `/robot2/dock/_action/status` are both `dock`,
// and a cross-robot aggregate is the reason to record them.
std::string ActionNameFromStatusTopic(const std::string& topic);

// The actions recorded without anyone naming them: Nav2's and the iRobot Create 3's.
//
// It is a list of names and not of topics because a topic is a deployment detail
// and a name is not. Nav2's PLANNER, SMOOTHER and CONTROLLER actions are
// deliberately absent: they fire on every replan, several times a minute. Naming
// one in `action_names` still records it.
const std::vector<std::string>& DefaultActionNames();

// The Nav2 actions DefaultActionNames leaves out because they run on every replan: the planner,
// smoother and controller actions.
const std::vector<std::string>& ReplanActionNames();

// Whether `topic` lies under `node_namespace`, which bounds what a node takes without an allowlist
// entry: one SDK per robot, each in its robot's namespace, records only its own robot. At the root
// namespace every topic does.
bool InDiscoveryNamespace(const std::string& topic, const std::string& node_namespace);

// Whether an action status topic on the graph will be recorded, and why.
//
// The negative half is the one worth carrying: an action nobody enrolled is
// subscribed by nothing and emits no events, which is invisible during the run and
// indistinguishable afterwards from an action that never fired.
struct ActionTopicVerdict {
  std::string topic;
  std::string action_name;
  bool enrolled = false;
  std::string reason;
  // Not enrolled only because the built-in `action_names` leaves out one of ReplanActionNames.
  bool replan_action = false;
};

// `ingest_refused` is the resolved ingest class for this topic being `refuse`,
// which means no subscription is created at all. It has to reach the verdict
// rather than be applied after it: a refused topic is not allowlisted either --
// `TopicPlan::Allowed` says so -- so without it the topic falls through to the
// name rules, is enrolled, and is then skipped by the subscribe loop, leaving the
// startup line and `doctor` both reporting an action that emits nothing.
//
// `node_namespace` scopes the name rules: outside it an action is recorded only when the
// allowlist names it.
ActionTopicVerdict ClassifyActionTopic(const std::string& topic, bool allowlisted,
                                       bool ingest_refused, bool discovery_enabled,
                                       const std::vector<std::string>& action_names,
                                       const std::string& node_namespace = "/");

}  // namespace cognitive3d_ros
