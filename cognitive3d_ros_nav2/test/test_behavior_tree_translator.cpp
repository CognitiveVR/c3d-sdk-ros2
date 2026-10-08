// /behavior_tree_log is the richest Nav2 source and the easiest to drown in.
// These tests pin all three halves: recovery behaviours always survive, tick-rate
// condition flapping never does, and the tree's own cycling is a sensor series
// rather than events.

#include <gtest/gtest.h>
#include <rclcpp/serialization.hpp>
#include <unistd.h>

#include <filesystem>

#include "c3d/spool.hpp"
#include "cognitive3d_ros/session_runner.hpp"
#include "cognitive3d_ros_nav2/behavior_tree_translator.hpp"
#include "cognitive3d_ros_nav2/register.hpp"

using namespace cognitive3d_ros_nav2;
using cognitive3d_ros::PoseConvention;
using cognitive3d_ros::SessionRunner;
using cognitive3d_ros::TranslationContext;
namespace fs = std::filesystem;

namespace {

class CapturingTransport : public c3d::ITransport {
 public:
  c3d::HttpResponse Post(const std::string&, const std::string& body,
                         const std::vector<std::string>&) override {
    bodies.push_back(body);
    c3d::HttpResponse ok;
    ok.status = 200;
    ok.saw_request_time_header = true;
    return ok;
  }
  std::vector<std::string> bodies;
  int CountIn(const std::string& needle) const {
    int n = 0;
    for (const auto& b : bodies) {
      std::size_t pos = 0;
      while ((pos = b.find(needle, pos)) != std::string::npos) {
        ++n;
        pos += needle.size();
      }
    }
    return n;
  }
};

// A template, because a discarded `if constexpr` branch is still type-checked outside one, and
// Humble's message has no `uid` to assign.
template <typename T>
void SetUid(T& change, std::uint16_t uid) {
  if constexpr (cognitive3d_ros_nav2::HasUid<T>::value) change.uid = uid;
}

nav2_msgs::msg::BehaviorTreeStatusChange MakeChange(const std::string& node,
                                                    const std::string& from,
                                                    const std::string& to,
                                                    std::uint16_t uid = 7) {
  nav2_msgs::msg::BehaviorTreeStatusChange change;
  change.node_name = node;
  change.previous_status = from;
  change.current_status = to;
  SetUid(change, uid);
  change.timestamp.sec = 1787000000;
  return change;
}

// The distro this suite was built against, from the build environment rather than from the
// translator's own detection: an expectation derived from the code under test would agree with
// that code whatever it did.
bool BuiltForHumble() {
  const std::string distro = C3D_TEST_ROS_DISTRO;
  EXPECT_FALSE(distro.empty()) << "ROS_DISTRO was not set when this test was built";
  return distro == "humble";
}

rclcpp::SerializedMessage Serialize(const nav2_msgs::msg::BehaviorTreeLog& msg) {
  rclcpp::SerializedMessage out;
  static rclcpp::Serialization<nav2_msgs::msg::BehaviorTreeLog> serializer;
  serializer.serialize_message(&msg, &out);
  return out;
}

class BtTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() /
           ("c3d_bt_" + std::to_string(::getpid()) + "_" + std::to_string(counter_++));
    fs::create_directories(dir_);
    c3d::Config cfg;
    cfg.application_key = "K";
    cfg.scene_id = "S";
    cfg.scene_version = "1";
    cfg.device_id = "robot";
    cfg.spool_dir = dir_.string();
    cfg.auto_send_interval_s = 3600.0;
    cfg.worker_tick_s = 0.01;
    cfg.retry_min_s = 0.02;
    cfg.retry_max_s = 0.05;
    transport_ = std::make_shared<CapturingTransport>();
    spool_ = std::make_shared<c3d::DiskSpool>(dir_.string(), 64ull << 20,
                                              std::make_shared<c3d::NullLogger>());
    runner_ = std::make_unique<SessionRunner>(cfg, PoseConvention::kRep103, transport_,
                                              spool_, std::make_shared<c3d::NullLogger>());
    ASSERT_TRUE(runner_->Begin());
  }
  void TearDown() override {
    if (runner_) runner_->End("test");
    runner_.reset();
    std::error_code ec;
    fs::remove_all(dir_, ec);
  }
  void Flush() {
    runner_->session()->Flush();
    runner_->session()->PumpUploads();
  }
  TranslationContext Ctx(double t) {
    TranslationContext ctx;
    ctx.topic = "/behavior_tree_log";
    ctx.wire_time = t;
    return ctx;
  }
  fs::path dir_;
  std::shared_ptr<CapturingTransport> transport_;
  std::shared_ptr<c3d::DiskSpool> spool_;
  std::unique_ptr<SessionRunner> runner_;
  static int counter_;
};
int BtTest::counter_ = 0;

}  // namespace

TEST_F(BtTest, RecoveryBehavioursAlwaysProduceEvents) {
  // This is the reason to consume this topic at all: "what did the robot try
  // before it gave up" is the autonomy failure-analysis question.
  BehaviorTreeTranslator t;
  nav2_msgs::msg::BehaviorTreeLog log;
  log.event_log = {MakeChange("Spin", "IDLE", "RUNNING"),
                   MakeChange("Spin", "RUNNING", "SUCCESS"),
                   MakeChange("BackUp", "IDLE", "RUNNING"),
                   MakeChange("BackUp", "RUNNING", "FAILURE"),
                   MakeChange("ClearEntireCostmap", "IDLE", "RUNNING")};
  t.Translate(Serialize(log), Ctx(1000.0), *runner_);
  Flush();
  EXPECT_EQ(transport_->CountIn("nav.recovery.started"), 3);
  EXPECT_EQ(transport_->CountIn("nav.recovery.finished"), 1);
  EXPECT_EQ(transport_->CountIn("nav.recovery.failed"), 1);
}

TEST_F(BtTest, ConditionNodeFlappingIsSuppressed) {
  // A Nav2 tree ticks at 10 to 30 Hz with dozens of nodes. Forwarding every
  // transition would be thousands of events per minute, almost all of them a
  // condition node oscillating, and it would bury the recoveries. A condition
  // the list below does not know -- an integrator's own -- still reaches the
  // dedup, which collapses the flapping to the one failure.
  BehaviorTreeTranslator t;
  nav2_msgs::msg::BehaviorTreeLog log;
  for (int i = 0; i < 200; ++i) {
    log.event_log.push_back(MakeChange("IsPayloadSecure", "SUCCESS", "FAILURE"));
    log.event_log.push_back(MakeChange("IsPayloadSecure", "FAILURE", "SUCCESS"));
  }
  t.Translate(Serialize(log), Ctx(1000.0), *runner_);
  Flush();
  EXPECT_GT(t.suppressed(), 300u);
  EXPECT_EQ(t.emitted(), 1u) << "condition flapping was not suppressed";
}

TEST_F(BtTest, ANav2ConditionNodeFailingIsTheAnswerNoAndNotAFailure) {
  // A condition plugin is asked a question on every tick and answers FAILURE for
  // "no". Unfiltered, `GoalUpdated` and `WouldAControllerRecoveryHelp` read as
  // failed nodes for a goal that was never updated and a controller recovery
  // that would not have helped -- both of them the tree working. The control and decorator nodes are not excluded with them:
  // those fail only when the tree gives up, which is the event.
  BehaviorTreeTranslator t;
  nav2_msgs::msg::BehaviorTreeLog log;
  log.event_log = {MakeChange("GoalUpdated", "IDLE", "FAILURE"),
                   MakeChange("WouldAControllerRecoveryHelp", "RUNNING", "FAILURE"),
                   MakeChange("IsPathValid", "SUCCESS", "FAILURE"),
                   MakeChange("RateController", "RUNNING", "FAILURE"),
                   MakeChange("NavigateWithReplanning", "RUNNING", "FAILURE")};
  t.Translate(Serialize(log), Ctx(1000.0), *runner_);
  Flush();
  EXPECT_EQ(transport_->CountIn("\"GoalUpdated\""), 0);
  EXPECT_EQ(transport_->CountIn("\"WouldAControllerRecoveryHelp\""), 0);
  EXPECT_EQ(transport_->CountIn("\"IsPathValid\""), 0);
  EXPECT_EQ(transport_->CountIn("nav.bt.node_failed"), 2)
      << "a control or decorator node failing is the tree giving up and is still an event";
  EXPECT_EQ(t.emitted(), 2u);
}

TEST_F(BtTest, TheReplanningCycleIsASensorSeriesAndNotEvents) {
  // The default Nav2 tree replans at 1 Hz, so the planner and controller nodes
  // cycle IDLE>RUNNING>SUCCESS for as long as the robot drives. As events that
  // cycle is the overwhelming majority of a navigating session's events, and it
  // says only that the tree is running -- which is what a rate says, cheaply.
  BehaviorTreeTranslator t;
  nav2_msgs::msg::BehaviorTreeLog log;
  for (const char* node : {"RateController", "ComputePathToPose", "FollowPath"}) {
    log.event_log.push_back(MakeChange(node, "IDLE", "RUNNING"));
    log.event_log.push_back(MakeChange(node, "RUNNING", "SUCCESS"));
  }
  // The first message opens the window; the second closes it two seconds later.
  t.Translate(Serialize(log), Ctx(1000.0), *runner_);
  t.Translate(Serialize(log), Ctx(1002.0), *runner_);
  Flush();
  EXPECT_EQ(t.emitted(), 0u) << "a healthy tree cycling produced events";
  EXPECT_EQ(transport_->CountIn("nav.bt.transitions_per_second"), 1);
  // Twelve transitions over the two seconds the window covered: six a second.
  EXPECT_EQ(transport_->CountIn("[1002.0,6.0]"), 1) << "the series is a rate, not a count";
}

TEST_F(BtTest, AWindowLeftOpenAcrossTheSilenceBetweenGoalsIsNotDividedByIt) {
  // bt_navigator publishes nothing while no goal is active. A window carried
  // across that silence divides the next goal's first burst by the gap, and
  // reports a tree that has just started ticking as very nearly idle.
  BehaviorTreeTranslator t;
  nav2_msgs::msg::BehaviorTreeLog burst;
  for (const char* node : {"RateController", "ComputePathToPose", "FollowPath"}) {
    burst.event_log.push_back(MakeChange(node, "IDLE", "RUNNING"));
    burst.event_log.push_back(MakeChange(node, "RUNNING", "SUCCESS"));
  }
  t.Translate(Serialize(burst), Ctx(1000.0), *runner_);
  t.Translate(Serialize(burst), Ctx(1002.0), *runner_);
  // The goal ends, thirty seconds pass with nothing published, the next starts.
  t.Translate(Serialize(burst), Ctx(1032.0), *runner_);
  t.Translate(Serialize(burst), Ctx(1033.0), *runner_);
  Flush();
  EXPECT_EQ(transport_->CountIn("[1032.0,"), 0) << "the stale window was sampled";
  EXPECT_EQ(transport_->CountIn("[1033.0,12.0]"), 1)
      << "the resumed burst was divided by the silence before it";
}

TEST_F(BtTest, ASecondTreeGetsItsOwnSeriesAndItsOwnWindow) {
  // One translator instance serves every BehaviorTreeLog topic on the graph. Two
  // navigators interleaved on one series describe neither.
  BehaviorTreeTranslator t;
  nav2_msgs::msg::BehaviorTreeLog log;
  log.event_log = {MakeChange("FollowPath", "IDLE", "RUNNING")};
  TranslationContext first = Ctx(1000.0);
  TranslationContext second = Ctx(1000.0);
  second.topic = "/robot2/behavior_tree_log";
  t.Translate(Serialize(log), first, *runner_);
  t.Translate(Serialize(log), second, *runner_);
  first.wire_time = 1002.0;
  second.wire_time = 1002.0;
  t.Translate(Serialize(log), first, *runner_);
  t.Translate(Serialize(log), second, *runner_);
  Flush();
  EXPECT_EQ(transport_->CountIn("\"nav.bt.transitions_per_second\""), 1);
  EXPECT_EQ(transport_->CountIn("nav.bt.robot2.behavior_tree_log.transitions_per_second"), 1);
}

TEST_F(BtTest, ARecoveryPairsStartedWithFinishedAndItsWrapperIsSilent) {
  // RecoveryNode is the control node that runs a recovery subtree, and it ticks
  // on every navigation whether or not anything recovered. As a recovery it
  // reports a robot recovering continuously, and finishing recoveries nothing
  // ever started.
  BehaviorTreeTranslator t;
  nav2_msgs::msg::BehaviorTreeLog log;
  log.event_log = {MakeChange("RecoveryNode", "IDLE", "RUNNING"),
                   MakeChange("RoundRobin", "IDLE", "RUNNING"),
                   MakeChange("Spin", "IDLE", "RUNNING"),
                   MakeChange("Spin", "RUNNING", "SUCCESS"),
                   MakeChange("RoundRobin", "RUNNING", "SUCCESS"),
                   MakeChange("RecoveryNode", "RUNNING", "SUCCESS")};
  t.Translate(Serialize(log), Ctx(1000.0), *runner_);
  Flush();
  EXPECT_EQ(transport_->CountIn("nav.recovery.started"), 1);
  EXPECT_EQ(transport_->CountIn("nav.recovery.finished"), 1);
  EXPECT_EQ(t.emitted(), 2u) << "a control node was counted as a recovery";
}

TEST_F(BtTest, ARecoveryThatFailedIsNotAlsoReportedFinished) {
  // A recovery behaviour is reset to IDLE once it is done, and BT.CPP logs that
  // reset. Read as an outcome it gives every recovery two endings, and the one it
  // adds to a failure says the recovery finished.
  BehaviorTreeTranslator t;
  nav2_msgs::msg::BehaviorTreeLog log;
  log.event_log = {MakeChange("Spin", "IDLE", "RUNNING"),
                   MakeChange("Spin", "RUNNING", "FAILURE"),
                   MakeChange("Spin", "FAILURE", "IDLE")};
  t.Translate(Serialize(log), Ctx(1000.0), *runner_);
  Flush();
  EXPECT_EQ(transport_->CountIn("nav.recovery.started"), 1);
  EXPECT_EQ(transport_->CountIn("nav.recovery.failed"), 1);
  EXPECT_EQ(transport_->CountIn("nav.recovery.finished"), 0)
      << "a recovery that failed was also reported finished";
  EXPECT_EQ(t.emitted(), 2u);
}

TEST_F(BtTest, ARecoveryThatSucceededIsReportedFinishedOnce) {
  BehaviorTreeTranslator t;
  nav2_msgs::msg::BehaviorTreeLog log;
  log.event_log = {MakeChange("Spin", "IDLE", "RUNNING"),
                   MakeChange("Spin", "RUNNING", "SUCCESS"),
                   MakeChange("Spin", "SUCCESS", "IDLE")};
  t.Translate(Serialize(log), Ctx(1000.0), *runner_);
  Flush();
  EXPECT_EQ(transport_->CountIn("nav.recovery.started"), 1);
  EXPECT_EQ(transport_->CountIn("nav.recovery.finished"), 1);
  EXPECT_EQ(t.emitted(), 2u) << "the reset back to IDLE was counted as an outcome";
}

TEST_F(BtTest, ARecoveryTheTreeHaltedStillHasAnOutcome) {
  // RUNNING>IDLE is not the reset that follows an outcome: it is BT.CPP halting a
  // node that was still running, which is how a recovery ends when the goal is
  // cancelled or preempted. Suppressed with the resets, a started recovery has no
  // ending at all and the pair `started` promises is broken.
  BehaviorTreeTranslator t;
  nav2_msgs::msg::BehaviorTreeLog log;
  log.event_log = {MakeChange("Spin", "IDLE", "RUNNING"),
                   MakeChange("Spin", "RUNNING", "IDLE")};
  t.Translate(Serialize(log), Ctx(1000.0), *runner_);
  Flush();
  EXPECT_EQ(transport_->CountIn("nav.recovery.started"), 1);
  EXPECT_EQ(transport_->CountIn("nav.recovery.halted"), 1)
      << "a recovery the tree cut short was reported as nothing at all";
  EXPECT_EQ(transport_->CountIn("nav.recovery.finished"), 0)
      << "a recovery that was halted is not a recovery that finished";
  EXPECT_EQ(transport_->CountIn("nav.recovery.failed"), 0);
  EXPECT_EQ(t.emitted(), 2u);
}

TEST_F(BtTest, AControlNodeIsSilentUntilItFails) {
  // The other half of the same decision: a control node emits nothing while it
  // cycles, and a failure is still a failure whoever reported it.
  BehaviorTreeTranslator t;
  nav2_msgs::msg::BehaviorTreeLog log;
  log.event_log = {MakeChange("NavigateRecovery", "RUNNING", "FAILURE")};
  t.Translate(Serialize(log), Ctx(1000.0), *runner_);
  Flush();
  EXPECT_EQ(transport_->CountIn("nav.bt.node_failed"), 1);
  EXPECT_EQ(transport_->CountIn("nav.recovery."), 0) << "a wrapper failing is not a recovery";
}

TEST_F(BtTest, TheMessageCarriesAUidOnEveryDistroButHumble) {
  EXPECT_EQ(kStatusChangeHasUid, !BuiltForHumble())
      << "the uid detection disagrees with the distro this was built for";
}

TEST_F(BtTest, TwoNodesSharingANameAreTwoNodesWhereverTheMessageCarriesAUid) {
  // Nav2's default tree wraps the FollowPath action in a RecoveryNode NAMED
  // FollowPath, so one node_name is two nodes and the uid is what tells them
  // apart. The action's last attempt fails, then the wrapper gives up.
  //
  // Humble's message has no uid, so there the two share one key and the
  // wrapper's failure is an identical repeat of the action's: it is dropped, and
  // the session has one failure where the tree had two. That loss is the price of
  // keeping the dedup, and this pins it as the price rather than letting it
  // drift either way.
  BehaviorTreeTranslator t;
  nav2_msgs::msg::BehaviorTreeLog log;
  log.event_log = {MakeChange("FollowPath", "RUNNING", "FAILURE", 11),
                   MakeChange("FollowPath", "RUNNING", "FAILURE", 12)};
  t.Translate(Serialize(log), Ctx(1000.0), *runner_);
  Flush();
  if (BuiltForHumble()) {
    EXPECT_EQ(transport_->CountIn("nav.bt.node_failed"), 1)
        << "without a uid the wrapper's failure is a repeat of the action's";
    EXPECT_EQ(t.emitted(), 1u);
    EXPECT_EQ(t.suppressed(), 1u);
    EXPECT_EQ(transport_->CountIn("\"uid\""), 0) << "a uid the message does not carry was recorded";
  } else {
    EXPECT_EQ(transport_->CountIn("nav.bt.node_failed"), 2);
    EXPECT_EQ(t.emitted(), 2u) << "a wrapper and the node it wraps were one node";
    EXPECT_EQ(transport_->CountIn("\"uid\":11"), 1);
    EXPECT_EQ(transport_->CountIn("\"uid\":12"), 1);
  }
}

TEST_F(BtTest, ANodeFailingAtTickRateIsOneEventOnEveryDistro) {
  // The other side of the same trade, on every distro: one node failing at tick
  // rate must not become one event per tick, uid or not.
  BehaviorTreeTranslator t;
  nav2_msgs::msg::BehaviorTreeLog log;
  for (int i = 0; i < 50; ++i) {
    log.event_log.push_back(MakeChange("ComputePathToPose", "RUNNING", "FAILURE"));
  }
  t.Translate(Serialize(log), Ctx(1000.0), *runner_);
  Flush();
  EXPECT_EQ(transport_->CountIn("nav.bt.node_failed"), 1);
  EXPECT_EQ(t.emitted(), 1u);
}

TEST_F(BtTest, AFailureTheCapDiscardedIsOfferedAgainRatherThanDeduplicatedAway) {
  // The cap discards events, and the dedup is there to drop a repeat of one that
  // was RECORDED. Advanced past a discarded event it drops the repeat that would
  // have carried it instead, and a node stuck failing is never reported at all.
  cognitive3d_ros::EventRateLimiter limiter;
  limiter.set_limit_hz(1.0);
  BehaviorTreeTranslator t;
  auto at = [this, &limiter](double when) {
    TranslationContext ctx = Ctx(when);
    ctx.event_limiter = &limiter;
    return ctx;
  };

  // One token, two failures at the same instant: the second is discarded.
  nav2_msgs::msg::BehaviorTreeLog burst;
  burst.event_log = {MakeChange("ComputePathToPose", "RUNNING", "FAILURE"),
                     MakeChange("FollowPath", "RUNNING", "FAILURE")};
  t.Translate(Serialize(burst), at(1000.0), *runner_);
  // Still failing, and now there is budget for it.
  nav2_msgs::msg::BehaviorTreeLog again;
  again.event_log = {MakeChange("FollowPath", "RUNNING", "FAILURE")};
  t.Translate(Serialize(again), at(1002.0), *runner_);
  Flush();

  ASSERT_EQ(limiter.suppressed_for("/behavior_tree_log"), 1u)
      << "the case under test did not happen: nothing was capped";
  EXPECT_EQ(transport_->CountIn("FollowPath"), 1)
      << "the discarded failure was never re-offered, so the session lost it";
  EXPECT_EQ(transport_->CountIn("nav.bt.node_failed"), 2);
}

TEST_F(BtTest, TheRecoveryNamesAreTheInstanceNamesTheLogActuallyCarries) {
  // BehaviorTreeLog carries the node's instance name, and Nav2's shipped trees
  // rename the costmap clearers. The match has to be exact: those same trees ship
  // CancelSpin and a CancelingControlAndWait sequence, which a substring match
  // reads as a Spin and a Wait running.
  BehaviorTreeTranslator t;
  nav2_msgs::msg::BehaviorTreeLog log;
  log.event_log = {MakeChange("ClearLocalCostmap-Subtree", "IDLE", "RUNNING"),
                   MakeChange("SpinRecovery", "IDLE", "RUNNING"),
                   MakeChange("CancelingControlAndWait", "IDLE", "RUNNING"),
                   MakeChange("CancelSpin", "RUNNING", "SUCCESS")};
  t.Translate(Serialize(log), Ctx(1000.0), *runner_);
  Flush();
  EXPECT_EQ(transport_->CountIn("nav.recovery.started"), 2);
  EXPECT_EQ(t.emitted(), 2u) << "a cancel node was counted as the behaviour it cancels";
}

TEST_F(BtTest, NodeFailureIsAlwaysInteresting) {
  BehaviorTreeTranslator t;
  nav2_msgs::msg::BehaviorTreeLog log;
  log.event_log = {MakeChange("ComputePathToPose", "RUNNING", "FAILURE")};
  t.Translate(Serialize(log), Ctx(1000.0), *runner_);
  Flush();
  EXPECT_EQ(transport_->CountIn("nav.bt.node_failed"), 1);
  EXPECT_EQ(transport_->CountIn("ComputePathToPose"), 1);
}

TEST_F(BtTest, InternalWallClockStampIsRecordedAsDataNotUsedAsEventTime) {
  // BehaviorTree.CPP stamps these with wall clock while the outer header is ROS
  // time. Under use_sim_time they disagree by decades, so the internal stamp is
  // carried as a property and the session uses the wire time.
  BehaviorTreeTranslator t;
  nav2_msgs::msg::BehaviorTreeLog log;
  log.event_log = {MakeChange("Spin", "IDLE", "RUNNING")};
  t.Translate(Serialize(log), Ctx(1787099999.0), *runner_);
  Flush();
  EXPECT_EQ(transport_->CountIn("bt_stamp"), 1);
  EXPECT_EQ(transport_->CountIn("1787099999"), 1) << "event time should be the wire time";
}

TEST_F(BtTest, RegistrationAddsTheTypeToACoreRegistry) {
  // The layering claim, tested: the core registry does not know about Nav2, and
  // this optional package is what adds it.
  cognitive3d_ros::TranslatorRegistry registry;
  cognitive3d_ros::RegisterCoreTranslators(registry);
  EXPECT_FALSE(registry.Handles("nav2_msgs/msg/BehaviorTreeLog"));
  RegisterNav2Translators(registry);
  EXPECT_TRUE(registry.Handles("nav2_msgs/msg/BehaviorTreeLog"));
}
