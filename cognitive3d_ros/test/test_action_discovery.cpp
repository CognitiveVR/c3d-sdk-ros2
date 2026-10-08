// Which action servers a run records, decided without a graph.
//
// The interesting half is the NEGATIVE one. An action that is not enrolled emits
// nothing, and a session missing a docking objective looks exactly like a robot
// that never docked -- so the rule that decides enrolment is worth pinning where
// it can be read.

#include <gtest/gtest.h>

#include <algorithm>

#include "cognitive3d_ros/action_discovery.hpp"

using namespace cognitive3d_ros;

TEST(ActionDiscovery, NameIsTheSegmentBeforeTheActionSuffix) {
  EXPECT_EQ(ActionNameFromStatusTopic("/navigate_to_pose/_action/status"), "navigate_to_pose");
}

TEST(ActionDiscovery, NameIgnoresTheNamespace) {
  // A fleet namespaces every action, and the point of matching on the name is
  // that one entry covers every robot.
  EXPECT_EQ(ActionNameFromStatusTopic("/robot1/dock/_action/status"), "dock");
  EXPECT_EQ(ActionNameFromStatusTopic("/fleet/a/b/c/undock/_action/status"), "undock");
}

TEST(ActionDiscovery, NonActionTopicsHaveNoName) {
  EXPECT_EQ(ActionNameFromStatusTopic("/navigate_to_pose/_action/feedback"), "");
  EXPECT_EQ(ActionNameFromStatusTopic("/diagnostics"), "");
  EXPECT_EQ(ActionNameFromStatusTopic(""), "");
  // The suffix with nothing in front of it names no action, so it must not be
  // read as one with an empty name.
  EXPECT_EQ(ActionNameFromStatusTopic("/_action/status"), "");
}

TEST(ActionDiscovery, DefaultListCoversNav2AndCreate3ButNotThePlanner) {
  const auto& names = DefaultActionNames();
  auto has = [&names](const char* name) {
    return std::find(names.begin(), names.end(), name) != names.end();
  };
  EXPECT_TRUE(has("navigate_to_pose"));
  EXPECT_TRUE(has("follow_waypoints"));
  EXPECT_TRUE(has("dock"));
  EXPECT_TRUE(has("undock"));
  // The planner and smoother fire on every replan, several times a minute, and
  // their outcomes describe the planner rather than the robot.
  EXPECT_FALSE(has("compute_path_to_pose"));
  EXPECT_FALSE(has("compute_path_through_poses"));
  EXPECT_FALSE(has("smooth_path"));
  // The CONTROLLER is on the same list for a worse reason: the default behaviour
  // tree replans about once a second, each replan preempts the running FollowPath
  // goal, and the server terminates the preempted one as ABORTED. Baked in, a
  // healthy run carries hundreds of `follow_path.aborted` and no failure.
  EXPECT_FALSE(has("follow_path"));
}

TEST(ActionDiscovery, ABakedInActionIsEnrolledWithoutAnAllowlistEntry) {
  const ActionTopicVerdict verdict = ClassifyActionTopic(
      "/robot1/navigate_to_pose/_action/status", /*allowlisted=*/false,
      /*ingest_refused=*/false, /*discovery_enabled=*/true, DefaultActionNames());
  EXPECT_TRUE(verdict.enrolled);
  EXPECT_EQ(verdict.action_name, "navigate_to_pose");
}

TEST(ActionDiscovery, AnUnlistedActionIsNotEnrolledAndSaysWhy) {
  const ActionTopicVerdict verdict =
      ClassifyActionTopic("/compute_path_to_pose/_action/status", /*allowlisted=*/false,
                          /*ingest_refused=*/false, /*discovery_enabled=*/true,
                          DefaultActionNames());
  EXPECT_FALSE(verdict.enrolled);
  EXPECT_NE(verdict.reason.find("compute_path_to_pose"), std::string::npos);
  EXPECT_NE(verdict.reason.find("action_names"), std::string::npos);
}

// The planner, smoother and controller actions are left out of the built-in list on purpose, and
// the verdict says so, so doctor can report them as information rather than as an omission.
TEST(ActionDiscovery, ANav2ReplanActionIsMarkedAsLeftOutByDesign) {
  for (const std::string name : ReplanActionNames()) {
    const ActionTopicVerdict verdict =
        ClassifyActionTopic("/" + name + "/_action/status", /*allowlisted=*/false,
                            /*ingest_refused=*/false, /*discovery_enabled=*/true,
                            DefaultActionNames());
    EXPECT_FALSE(verdict.enrolled) << name;
    EXPECT_TRUE(verdict.replan_action) << name;
    EXPECT_NE(verdict.reason.find("every replan"), std::string::npos) << verdict.reason;
  }
  const ActionTopicVerdict custom = ClassifyActionTopic(
      "/arm/pick_object/_action/status", false, false, true, DefaultActionNames());
  EXPECT_FALSE(custom.replan_action);
  EXPECT_EQ(custom.reason.find("replan"), std::string::npos) << custom.reason;
  // Recorded, refused or out of discovery's reach, it is not "left out by design".
  EXPECT_FALSE(ClassifyActionTopic("/follow_path/_action/status", true, false, true,
                                   DefaultActionNames()).replan_action);
  EXPECT_FALSE(ClassifyActionTopic("/follow_path/_action/status", false, true, true,
                                   DefaultActionNames()).replan_action);
  EXPECT_FALSE(ClassifyActionTopic("/follow_path/_action/status", false, false, false,
                                   DefaultActionNames()).replan_action);
  EXPECT_TRUE(ClassifyActionTopic("/follow_path/_action/status", false, false, true,
                                  std::vector<std::string>{"follow_path"}).enrolled);
  // An operator's own list leaving one out is that list's choice, not the built-in one's.
  const ActionTopicVerdict own_list = ClassifyActionTopic(
      "/follow_path/_action/status", false, false, true,
      std::vector<std::string>{"navigate_to_pose"});
  EXPECT_FALSE(own_list.enrolled);
  EXPECT_FALSE(own_list.replan_action);
  EXPECT_EQ(own_list.reason.find("built-in"), std::string::npos) << own_list.reason;
}

TEST(ActionDiscovery, AnOperatorsListReplacesTheBakedOne) {
  // Setting `action_names` replaces the default rather than adding to it, which
  // is the whole reason a user's list can turn a baked-in action OFF.
  const std::vector<std::string> mine{"pick_object"};
  EXPECT_FALSE(ClassifyActionTopic("/navigate_to_pose/_action/status", false, false, true, mine)
                   .enrolled);
  EXPECT_TRUE(ClassifyActionTopic("/arm/pick_object/_action/status", false, false, true, mine)
                  .enrolled);
}

TEST(ActionDiscovery, TheAllowlistWinsOverEverySwitch) {
  // An operator who named the topic gets it even with discovery off and the name
  // absent from the list -- including the planner actions nothing bakes in.
  const ActionTopicVerdict verdict =
      ClassifyActionTopic("/compute_path_to_pose/_action/status", /*allowlisted=*/true,
                          /*ingest_refused=*/false, /*discovery_enabled=*/false,
                          std::vector<std::string>{});
  EXPECT_TRUE(verdict.enrolled);
  EXPECT_NE(verdict.reason.find("topics"), std::string::npos);
}

TEST(ActionDiscovery, ARefusedTopicIsNotEnrolledByAnyOtherRule) {
  // `ingest: refuse` creates no subscription, and it also makes the topic read as
  // NOT allowlisted -- so a refused `/navigate_to_pose` falls to the name rules,
  // which would enrol it. Reported enrolled, subscribed by nothing: the exact
  // shape this whole verdict exists to prevent.
  for (const bool allowlisted : {false, true}) {
    const ActionTopicVerdict verdict = ClassifyActionTopic(
        "/navigate_to_pose/_action/status", allowlisted, /*ingest_refused=*/true,
        /*discovery_enabled=*/true, DefaultActionNames());
    EXPECT_FALSE(verdict.enrolled) << "allowlisted=" << allowlisted;
    EXPECT_NE(verdict.reason.find("refuse"), std::string::npos) << verdict.reason;
  }
}

TEST(ActionDiscovery, DiscoveryOffLeavesOnlyTheAllowlist) {
  const ActionTopicVerdict verdict =
      ClassifyActionTopic("/navigate_to_pose/_action/status", /*allowlisted=*/false,
                          /*ingest_refused=*/false, /*discovery_enabled=*/false,
                          DefaultActionNames());
  EXPECT_FALSE(verdict.enrolled);
  EXPECT_NE(verdict.reason.find("action_discovery"), std::string::npos);
}

// One SDK per robot, each in its robot's namespace, records only its own robot's actions; the
// allowlist still reaches anything it names.
TEST(ActionDiscovery, ANamespacedNodeEnrolsActionsOnlyUnderItsNamespace) {
  const std::vector<std::string> names = {"navigate_to_pose"};
  const auto mine = ClassifyActionTopic("/robot1/navigate_to_pose/_action/status", false, false,
                                        true, names, "/robot1");
  EXPECT_TRUE(mine.enrolled) << mine.reason;
  const auto theirs = ClassifyActionTopic("/robot2/navigate_to_pose/_action/status", false, false,
                                          true, names, "/robot1");
  EXPECT_FALSE(theirs.enrolled);
  EXPECT_NE(theirs.reason.find("namespace"), std::string::npos) << theirs.reason;
  // A prefix that is not a path segment is not the namespace.
  EXPECT_FALSE(ClassifyActionTopic("/robot10/navigate_to_pose/_action/status", false, false, true,
                                   names, "/robot1")
                   .enrolled);
  EXPECT_TRUE(ClassifyActionTopic("/robot2/navigate_to_pose/_action/status", true, false, true,
                                  names, "/robot1")
                  .enrolled)
      << "the allowlist names it, so it is taken wherever it is";
  EXPECT_TRUE(ClassifyActionTopic("/robot2/navigate_to_pose/_action/status", false, false, true,
                                  names, "/")
                  .enrolled)
      << "at the root namespace the whole graph is in scope";
}
