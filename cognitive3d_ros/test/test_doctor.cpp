// Tests for the preflight checks.
//
// Every check is tested on its FAILURE path as well as its passing one, because a check whose
// failure message nobody has read is a hollow gate -- and here the message IS the product. A
// `doctor` that detects a problem and describes it uselessly generates the support conversation it
// was built to end.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "c3d/config.hpp"
#include "c3d/environment.hpp"
#include "c3d/spool.hpp"
#include "c3d/version.hpp"
#include "c3d/wall_clock.hpp"
#include "cognitive3d_ros/doctor.hpp"
#include "cognitive3d_ros/media_config.hpp"
#include "cognitive3d_ros/qos_resolver.hpp"
#include "cognitive3d_ros/run_doctor.hpp"
#include "throttled_http_server.hpp"

using namespace cognitive3d_ros::doctor;
using cognitive3d_ros::EstimatedCameraBytesPerSecond;

namespace {

bool Mentions(const Check& check, const std::string& needle) {
  return (check.detail + " " + check.remedy).find(needle) != std::string::npos;
}

}  // namespace

// --- the secret mask, which is the one place a bug leaks a credential ------------------------

TEST(MaskSecret, NeverEchoesAShortValue) {
  // Four characters of a short key are a large part of it, so short values report LENGTH only.
  //
  // The keys here are deliberately UPPERCASE: the "too short to mask safely" message is lowercase
  // prose, so a lowercase key such as "a" occurs in it by accident -- "characters" contains an
  // "a". A substring assertion against a message is only meaningful when the needle cannot occur
  // in the message by accident.
  for (const std::string key : {"Q", "QZXW", "QZXWVBJK", "QZXWVBJKQZXWVBJ"}) {
    const std::string masked = MaskSecret(key);
    EXPECT_EQ(masked.find(key), std::string::npos) << "leaked a " << key.size()
                                                   << "-char value: " << masked;
    EXPECT_NE(masked.find(std::to_string(key.size())), std::string::npos) << masked;
  }
}

TEST(MaskSecret, MasksALongValueToItsLastFourAlone) {
  const std::string key = "abcdEFGH0123456789wxyz";
  EXPECT_EQ(MaskSecret(key), "ending ...wxyz");
}

TEST(MaskSecret, AnEmptyValueSaysSoRatherThanPrintingNothing) {
  EXPECT_EQ(MaskSecret(""), "<empty>");
}

// --- the verdict arithmetic ------------------------------------------------------------------

TEST(Aggregate, FatalBeatsAdvisoryBeatsOk) {
  EXPECT_EQ(Aggregate({}), Severity::kOk);
  EXPECT_EQ(Aggregate({{"a", Severity::kOk, "", ""}}), Severity::kOk);
  EXPECT_EQ(Aggregate({{"a", Severity::kOk, "", ""}, {"b", Severity::kAdvisory, "", "x"}}),
            Severity::kAdvisory);
  EXPECT_EQ(Aggregate({{"a", Severity::kAdvisory, "", "x"}, {"b", Severity::kFatal, "", "x"}}),
            Severity::kFatal);
}

TEST(ExitCode, AdvisoriesDoNotFailTheCommand) {
  // "no map yet" is normal on a robot that has not localised. A tool that exits non-zero for it
  // gets muted within a day, and then reports nothing anyone reads.
  EXPECT_EQ(ExitCode({{"a", Severity::kAdvisory, "", "x"}}), 0);
  EXPECT_EQ(ExitCode({{"a", Severity::kOk, "", ""}}), 0);
  EXPECT_EQ(ExitCode({{"a", Severity::kFatal, "", "x"}}), 1);
}

TEST(ExitCode, NoChecksIsAFailureAndNotAPass) {
  // THE DEFECT THIS PREVENTS: a doctor that gathered nothing reporting success. An empty list means
  // the caller is broken, and 0 would say "everything is fine" about a tool that looked at nothing.
  EXPECT_EQ(ExitCode({}), 1);
}

// --- every check, failing and passing --------------------------------------------------------

// rclcpp's two construction failures, in both distros' wording, each naming the parameter.
TEST(Construction, AnEmptyYamlListNamesTheLineToDelete) {
  const Check check = CheckConstruction(
      "parameter_value_from failed for parameter 'session_properties': No parameter value set");
  EXPECT_EQ(check.severity, Severity::kFatal);
  EXPECT_TRUE(Mentions(check, "session_properties: []"));
  EXPECT_TRUE(Mentions(check, "Delete"));
  EXPECT_FALSE(Mentions(check, "Quote")) << "an empty list is not a type error";
}

TEST(Construction, AnEmptyActionListNamesTheSwitchThatMeansNone) {
  const Check check = CheckConstruction(
      "parameter_value_from failed for parameter 'action_names': No parameter value set");
  EXPECT_TRUE(Mentions(check, "action_discovery: false"));
}

TEST(Construction, AMistypedValueNamesTheTypeInEitherWording) {
  for (const char* what :
       {"parameter 'scene_version' has invalid type: expected [string] got [integer]",
        "parameter 'scene_version' has invalid type: Wrong parameter type, parameter "
        "{scene_version} is of type {string}, setting it to {integer} is not allowed."}) {
    const Check check = CheckConstruction(what);
    EXPECT_EQ(check.severity, Severity::kFatal) << what;
    EXPECT_TRUE(Mentions(check, "scene_version must be a string")) << check.detail;
    EXPECT_TRUE(Mentions(check, "an integer")) << check.detail;
    EXPECT_TRUE(Mentions(check, "Quote")) << check.remedy;
  }
  const Check flag =
      CheckConstruction("parameter 'dry_run' has invalid type: expected [bool] got [string]");
  EXPECT_TRUE(Mentions(flag, "true or false")) << flag.remedy;
}

TEST(Construction, AnythingElseIsReportedAsThrown) {
  const Check check = CheckConstruction("something rclcpp has not said before");
  EXPECT_EQ(check.severity, Severity::kFatal);
  EXPECT_EQ(check.detail, "something rclcpp has not said before");
  EXPECT_FALSE(check.remedy.empty());
}

TEST(ApplicationKey, AbsentIsFatalAndNamesTheVariable) {
  const Check check = CheckApplicationKey("");
  EXPECT_EQ(check.severity, Severity::kFatal);
  EXPECT_TRUE(Mentions(check, "C3D_APPLICATION_API_KEY"));
}

TEST(ApplicationKey, PresentReportsOnlyAMask) {
  const std::string key = "abcdEFGH0123456789wxyz";
  const Check check = CheckApplicationKey(key);
  EXPECT_EQ(check.severity, Severity::kOk);
  EXPECT_EQ(check.detail, "set (ending ...wxyz)");
}

TEST(SpoolDirectory, UnwritableIsFatalAndUnsetIsFatalForADifferentReason) {
  EXPECT_EQ(CheckSpoolDirectory("", true, 0).severity, Severity::kFatal);
  EXPECT_TRUE(Mentions(CheckSpoolDirectory("", true, 0), "spool_dir"));
  const Check unwritable = CheckSpoolDirectory("/nope", false, 0);
  EXPECT_EQ(unwritable.severity, Severity::kFatal);
  EXPECT_TRUE(Mentions(unwritable, "/nope"));
}

TEST(SpoolDirectory, OrphansAreWorthMentioningButNotWorthFailing) {
  const Check check = CheckSpoolDirectory("/tmp/spool", true, 2);
  EXPECT_EQ(check.severity, Severity::kAdvisory);
  EXPECT_TRUE(Mentions(check, "2"));
  EXPECT_EQ(CheckSpoolDirectory("/tmp/spool", true, 0).severity, Severity::kOk);
}

// The holder is usually the node these parameters configure, already running, so a held lock is
// worth naming and not worth failing; a lock the node would refuse outright is.
TEST(SpoolDirectory, AHeldLockIsNamedAndALinkedOneIsFatal) {
  SpoolLockFacts held;
  held.held = true;
  held.holder = "4242";
  const Check in_use = CheckSpoolDirectory("/tmp/spool", true, 3, held);
  EXPECT_EQ(in_use.severity, Severity::kAdvisory);
  EXPECT_TRUE(Mentions(in_use, "/tmp/spool is in use by process 4242")) << in_use.detail;
  EXPECT_TRUE(Mentions(in_use, "its own spool_dir")) << in_use.remedy;

  SpoolLockFacts linked;
  linked.refused = "/tmp/spool/.lock is a symbolic link";
  const Check refused = CheckSpoolDirectory("/tmp/spool", true, 0, linked);
  EXPECT_EQ(refused.severity, Severity::kFatal);
  EXPECT_TRUE(Mentions(refused, "/tmp/spool/.lock is a symbolic link")) << refused.detail;
}

namespace {

std::string ReadFile(const std::filesystem::path& path) {
  std::ifstream in(path);
  return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

}  // namespace

// The probe asks the question a DiskSpool's construction would, without becoming the answer: the
// node holding the directory keeps it, and the probe leaves no lock of its own behind.
TEST(SpoolLock, TheProbeSeesTheHolderAndLeavesItTheLock) {
  namespace fs = std::filesystem;
  const fs::path dir =
      fs::temp_directory_path() / ("c3d_doctor_lock_" + std::to_string(::getpid()));
  fs::remove_all(dir);
  fs::create_directories(dir);
  const auto quiet = std::make_shared<c3d::NullLogger>();

  const SpoolLockFacts nothing = ProbeSpoolLock(dir.string());
  EXPECT_FALSE(nothing.held);
  EXPECT_FALSE(fs::exists(dir / ".lock")) << "the probe created the lock file";

  auto node = std::make_shared<c3d::DiskSpool>(dir.string(), 1 << 20, quiet);
  ASSERT_TRUE(node->error().empty()) << node->error();
  const SpoolLockFacts seen = ProbeSpoolLock(dir.string());
  EXPECT_TRUE(seen.held);
  EXPECT_EQ(seen.holder, std::to_string(::getpid()));
  EXPECT_TRUE(seen.refused.empty()) << seen.refused;
  c3d::DiskSpool second(dir.string(), 1 << 20, quiet);
  EXPECT_FALSE(second.error().empty()) << "the probe took the lock from its holder";

  node.reset();
  EXPECT_FALSE(ProbeSpoolLock(dir.string()).held);
  c3d::DiskSpool next(dir.string(), 1 << 20, quiet);
  EXPECT_TRUE(next.error().empty()) << "the probe kept the lock: " << next.error();

  fs::remove_all(dir);
  fs::create_directories(dir);
  const fs::path victim = dir / "victim";
  { std::ofstream(victim) << "precious"; }
  fs::create_symlink(victim, dir / ".lock");
  const SpoolLockFacts linked = ProbeSpoolLock(dir.string());
  EXPECT_NE(linked.refused.find("symbolic link"), std::string::npos) << linked.refused;
  EXPECT_EQ(ReadFile(victim), "precious");
  fs::remove_all(dir);
}

TEST(SharedSeries, TopicsSharingASeriesAreNamedAsAnAdvisory) {
  const Check shared = CheckSharedSeries({"/b and /a are both nav_msgs/msg/Odometry, ..."});
  EXPECT_EQ(shared.severity, Severity::kAdvisory);
  EXPECT_TRUE(Mentions(shared, "/b and /a are both nav_msgs/msg/Odometry")) << shared.detail;
  EXPECT_TRUE(Mentions(shared, "one topic of each")) << shared.remedy;
  EXPECT_EQ(CheckSharedSeries({}).severity, Severity::kOk);
}

// A described sensor's topic of another type than its kind reads records a series the description
// does not name, which only this check says before a run: fatal, naming both parameters and both
// types. A topic with no publisher is a note; one of the kind's type passes, naming the series.
TEST(SensorTopic, ATopicOfAnotherTypeThanItsKindIsFatalAndNoPublisherIsANote) {
  SensorTopicFacts facts;
  facts.sensor.name = "front_sonar";
  facts.sensor.kind = "cone_rangefinder";
  facts.sensor.topic = "/sonar";
  facts.sensor.series = {"sonar.value"};
  facts.type = "sensor_msgs/msg/Range";
  facts.publishers = 1;
  const Check range = CheckSensorTopic(facts);
  EXPECT_EQ(range.name, "sensor/front_sonar");
  EXPECT_EQ(range.severity, Severity::kFatal);
  for (const std::string said : {"sensor.front_sonar.kind", "sensor.front_sonar.topic",
                                 "sensor_msgs/msg/Range", "std_msgs/msg/Float32"}) {
    EXPECT_TRUE(Mentions(range, said)) << said << ": " << range.detail;
  }

  facts.sensor.kind = "binary_array";
  facts.type = "std_msgs/msg/Float32";
  const Check line = CheckSensorTopic(facts);
  EXPECT_EQ(line.severity, Severity::kFatal);
  EXPECT_TRUE(Mentions(line, "std_msgs/msg/Int32MultiArray")) << line.detail;
  facts.publishers = 0;
  EXPECT_EQ(CheckSensorTopic(facts).severity, Severity::kFatal)
      << "a subscriber of the wrong type still names the type the node would subscribe";

  facts.sensor.kind = "cone_rangefinder";
  const Check silent = CheckSensorTopic(facts);
  EXPECT_EQ(silent.severity, Severity::kAdvisory);
  EXPECT_TRUE(Mentions(silent, "/sonar has no publisher")) << silent.detail;
  facts.type = "";
  EXPECT_EQ(CheckSensorTopic(facts).severity, Severity::kAdvisory) << "absent from the graph";

  facts.type = "std_msgs/msg/Float32";
  facts.publishers = 2;
  const Check fine = CheckSensorTopic(facts);
  EXPECT_EQ(fine.severity, Severity::kOk) << fine.detail;
  EXPECT_TRUE(Mentions(fine, "recorded as sonar.value")) << fine.detail;
}

TEST(WallClock, AClockThatHasNotBeenSetIsFatalAndSaysWhatToWaitFor) {
  const Check unset = CheckWallClock(86400.0);
  EXPECT_EQ(unset.severity, Severity::kFatal);
  EXPECT_TRUE(Mentions(unset, "1970-01-02")) << unset.detail;
  EXPECT_TRUE(Mentions(unset, "NTP")) << unset.detail;
  EXPECT_TRUE(Mentions(unset, "refuses every session start")) << unset.remedy;
  EXPECT_EQ(CheckWallClock(c3d::kWallClockFloorEpochSeconds + 1.0).severity, Severity::kOk);
}

TEST(SimTime, BothDisagreementsAreReportedAndTheyAreNotTheSameSeverity) {
  // sim_time with no clock is fatal: ROS time never leaves zero.
  const Check starved = CheckSimTime(true, false);
  EXPECT_EQ(starved.severity, Severity::kFatal);
  EXPECT_TRUE(Mentions(starved, "/clock"));
  // A clock nobody asked for is only advisory: wall clock still goes on the wire, so telemetry
  // lands in the right year either way. Distinguishing these two is the point of the check.
  const Check ignored = CheckSimTime(false, true);
  EXPECT_EQ(ignored.severity, Severity::kAdvisory);
  EXPECT_EQ(CheckSimTime(true, true).severity, Severity::kOk);
  EXPECT_EQ(CheckSimTime(false, false).severity, Severity::kOk);
}

// Which timeline goes on the wire is the flag's to say, so every message that names it names the
// flag's position rather than asserting wall clock regardless.
TEST(SimTime, EveryMessageSaysWhetherSimTimeOnWireIsOn) {
  for (const bool clock : {true, false}) {
    const Check on = CheckSimTime(true, clock, true);
    const Check off = CheckSimTime(true, clock, false);
    EXPECT_TRUE(Mentions(on, "sim_time_on_wire is on")) << on.detail << on.remedy;
    EXPECT_TRUE(Mentions(off, "sim_time_on_wire is off")) << off.detail << off.remedy;
    EXPECT_EQ(on.severity, off.severity) << "the flag changes the wording, not the verdict";
  }
  EXPECT_FALSE(Mentions(CheckSimTime(true, false, true), "wall clock goes on the wire"));
}

TEST(Frames, NoPoseAtAllIsFatalAndSaysWhyItMatters) {
  const Check check = CheckFrames("map", "odom", "base_link", false, false);
  EXPECT_EQ(check.severity, Severity::kFatal);
  EXPECT_TRUE(Mentions(check, "map"));
  EXPECT_TRUE(Mentions(check, "odom"));
  EXPECT_TRUE(Mentions(check, "base_link"));
  // The consequence, not just the fact: a session with no pose still uploads and still appears.
  EXPECT_TRUE(Mentions(check, "SPATIAL TIMELINE"));
}

TEST(Frames, TheOdomFallbackIsAdvisoryBecauseItIsNormalAndStillWorks) {
  const Check check = CheckFrames("map", "odom", "base_link", false, true);
  EXPECT_EQ(check.severity, Severity::kAdvisory);
  EXPECT_TRUE(Mentions(check, "odom"));
  EXPECT_EQ(CheckFrames("map", "odom", "base_link", true, true).severity, Severity::kOk);
}

// view_frames listens for 5 seconds by default, which on a simulation slower than real time is too
// short to see every frame, and a frame it misses reads as one the robot does not have.
TEST(Frames, EveryRemedyThatSuggestsViewFramesGivesItTimeToHearEveryFrame) {
  for (const Check& check : {CheckFrames("map", "odom", "base_link", false, false),
                             CheckFrames("map", "", "base_link", false, false)}) {
    EXPECT_EQ(check.severity, Severity::kFatal);
    EXPECT_TRUE(Mentions(check, "`ros2 run tf2_tools view_frames --wait-time 15`")) << check.remedy;
  }
}

TEST(FrameWait, TheFallbackResolvingIsNotAnAnswerYet) {
  // On a healthy, fully localised robot `odom -> base_link` resolves on the first poll. A wait
  // that stopped there would report that map does not resolve -- while `tf2_echo map base_link`
  // beside it answers fine.
  EXPECT_FALSE(FrameWaitSettled(false, true, true, 0.0, 5.0));
  EXPECT_FALSE(FrameWaitSettled(false, true, true, 4.9, 5.0));
  EXPECT_TRUE(FrameWaitSettled(false, true, true, 5.0, 5.0))
      << "a robot with no localiser must still reach a verdict";
}

TEST(FrameWait, ThePreferredFrameEndsTheWaitAtOnceAndNothingElseDoes) {
  EXPECT_TRUE(FrameWaitSettled(true, false, true, 0.0, 5.0));
  EXPECT_TRUE(FrameWaitSettled(true, true, true, 0.0, 5.0));
  // Nothing resolves: there is no answer to settle on, and the caller's own ceiling ends it.
  EXPECT_FALSE(FrameWaitSettled(false, false, true, 60.0, 5.0));
  // An empty `reference_frame` is a deliberate fallback-only run, with nothing to wait for.
  EXPECT_TRUE(FrameWaitSettled(false, true, false, 0.0, 5.0));
}

TEST(GazeFrame, ConfiguredButUnresolvableIsFatal_UnsetIsFine) {
  const Check broken = CheckGazeFrame("oakd_rgb_camera_optical_frame", false);
  EXPECT_EQ(broken.name, "gaze frame");
  EXPECT_EQ(broken.severity, Severity::kFatal);
  EXPECT_TRUE(Mentions(broken, "oakd_rgb_camera_optical_frame"));
  EXPECT_TRUE(Mentions(broken, "`gaze_frame`"));
  // This is the node's quietest misconfiguration, so the message has to say what goes wrong.
  EXPECT_TRUE(Mentions(broken, "height"));
  EXPECT_EQ(CheckGazeFrame("", false).severity, Severity::kOk);
  EXPECT_EQ(CheckGazeFrame("cam", true).severity, Severity::kOk);
}

TEST(Gateway, UnreachableCarriesTheTransportsOwnMessage) {
  const Check check = CheckGateway("https", "data.cognitive3d.com", 0, false,
                                   "Could not resolve host");
  EXPECT_EQ(check.severity, Severity::kFatal);
  EXPECT_TRUE(Mentions(check, "Could not resolve host"));
}

TEST(Gateway, AnUnauthenticatedProbeRefusedAtAllIsStillFatal) {
  // The probe sends no application key -- the key could not be validated there anyway -- so a
  // refusal is fatal without being the credential's fault. That the message does not blame the
  // credential is asserted in Gateway.TheProbeIsUnauthenticatedSoTheMessagesMustNotClaimOtherwise.
  for (const long status : {401L, 403L}) {
    EXPECT_EQ(CheckGateway("https", "h", status, true, "").severity, Severity::kFatal) << status;
  }
}

TEST(Gateway, TwoHundredWithoutTheRequestTimeHeaderIsACaptivePortalAndIsFATAL) {
  // The trap: a hotel or corporate proxy answers 200 on the collector's behalf, payloads are
  // "accepted", and everything is lost. The core transport already treats this as retryable; here
  // it has to be said out loud, because to an operator it looks like success.
  const Check check = CheckGateway("https", "data.cognitive3d.com", 200, false, "");
  EXPECT_EQ(check.severity, Severity::kFatal);
  EXPECT_TRUE(Mentions(check, "cvr-request-time"));
  EXPECT_TRUE(Mentions(check, "portal"));
  // ...and the same 200 WITH the header is the healthy case.
  EXPECT_EQ(CheckGateway("https", "data.cognitive3d.com", 200, true, "").severity, Severity::kOk);
}

TEST(Gateway, AnOrdinaryFourOhFourOnTheProbeIsNotAFailureAndSaysSo) {
  // The probe posts to /health, which the collector does not route. Reachability is all this check
  // claims, and a bare "HTTP 404" beside `ok` reads as an error.
  const Check check = CheckGateway("https", "h", 404, true, "");
  EXPECT_EQ(check.severity, Severity::kOk);
  EXPECT_EQ(check.detail, "https://h reachable (HTTP 404, the expected answer to this probe)");
  // Only the answer the collector gives is called expected.
  EXPECT_EQ(CheckGateway("https", "h", 405, true, "").detail, "https://h reachable (HTTP 405)");
}

TEST(Topics, EveryMissingTopicIsNamed_NotJustTheFirst) {
  // THE POINT OF THIS CHECK. The node's own runtime warning uses RCLCPP_WARN_THROTTLE inside the
  // loop over rules, and that macro throttles per CALL SITE -- so the first missing topic consumes
  // the 30 s window and every other one is suppressed. Reordering the allowlist on a live robot
  // shows it: whichever topic comes first is the only one ever mentioned.
  // `doctor` has to name them all, because the case that matters has several missing at once.
  TopicFacts facts;
  facts.allowlist = {"/odom", "/battery_state", "/scan", "/joint_states"};
  facts.missing = {"/odom", "/battery_state", "/scan"};
  const Check check = CheckTopics(facts);
  EXPECT_EQ(check.severity, Severity::kAdvisory);
  for (const std::string topic : {"/odom", "/battery_state", "/scan"}) {
    EXPECT_TRUE(Mentions(check, topic)) << topic << " was not named";
  }
  EXPECT_TRUE(Mentions(check, "3"));
  EXPECT_TRUE(Mentions(check, "4"));
}

TEST(Topics, MissingIsNeverFatalBecauseBringUpIsSlow) {
  // A preflight that fails while the robot is still starting gets muted, and then reports nothing
  // anyone reads.
  TopicFacts one_missing;
  one_missing.allowlist = {"/a"};
  one_missing.missing = {"/a"};
  EXPECT_EQ(CheckTopics(one_missing).severity, Severity::kAdvisory);
  EXPECT_EQ(CheckTopics(TopicFacts{}).severity, Severity::kAdvisory);
  TopicFacts all_present;
  all_present.allowlist = {"/a", "/b"};
  EXPECT_EQ(CheckTopics(all_present).severity, Severity::kOk);
}

TEST(Topics, AnEmptyAllowlistSaysWhatItStillRecords) {
  // "nothing will be observed" is wrong in three ways at once, and it is the sentence an operator
  // acts on: it would send them hunting a broken subscription in a session that is recording
  // battery, diagnostics, objectives and every configured camera. The claim also has to agree with
  // what the node logs at configure, or the preflight and the run contradict each other about one
  // config.
  TopicFacts facts;
  facts.standard_types_enabled = true;
  facts.action_discovery_enabled = true;
  facts.media_count = 2;
  const Check check = CheckTopics(facts);
  EXPECT_EQ(check.severity, Severity::kAdvisory);
  EXPECT_TRUE(Mentions(check, "battery and diagnostics"));
  EXPECT_TRUE(Mentions(check, "action_discovery"));
  EXPECT_TRUE(Mentions(check, "cameras and lidars"));
  EXPECT_FALSE(Mentions(check, "nothing will be observed"));
  EXPECT_FALSE(Mentions(check, "only the pose timeline"));
}

TEST(Topics, AnEmptyAllowlistClaimsNothingTheConfigurationIsNotDoing) {
  // Every clause is conditional on the switch that produces it. Listing battery with
  // `auto_subscribe_standard_types` off promises a series the session will not carry, which is the
  // same defect as "nothing will be observed", pointing the other way.
  TopicFacts facts;
  facts.standard_types_enabled = false;
  facts.action_discovery_enabled = false;
  facts.media_count = 0;
  const Check check = CheckTopics(facts);
  EXPECT_FALSE(Mentions(check, "battery"));
  EXPECT_FALSE(Mentions(check, "action_discovery"));
  EXPECT_FALSE(Mentions(check, "cameras"));
  EXPECT_TRUE(Mentions(check, "pose timeline"))
      << "the one thing an empty allowlist always records went unsaid";
}

TEST(Gateway, ACollectorThatSaidItIsDownIsNotReportedAsFine) {
  // A 503 must not read as "reachable (HTTP 503)" with the whole run exiting 0. A load balancer
  // that has just said it is refusing work is not a passing check.
  for (const long status : {429L, 500L, 502L, 503L}) {
    const Check check = CheckGateway("https", "h", status, true, "");
    EXPECT_EQ(check.severity, Severity::kAdvisory) << status;
    EXPECT_TRUE(Mentions(check, "spool")) << status;
  }
}

TEST(Gateway, TheProbeIsUnauthenticatedSoTheMessagesMustNotClaimOtherwise) {
  // The probe sends NO Authorization header, because it could never have validated one: the
  // collector's router answers 404 before authentication runs, so a wrong key would pass as
  // `[ok]`. A 401 here is therefore something in FRONT of the collector, not a bad key, and the
  // text must not send someone off to check their credentials.
  const Check check = CheckGateway("https", "h", 401, true, "");
  EXPECT_EQ(check.severity, Severity::kFatal);
  EXPECT_TRUE(Mentions(check, "proxy"));
  EXPECT_EQ((check.detail + check.remedy).find("application key"), std::string::npos)
      << "the message still blames the credential the probe does not send";
}

TEST(Frames, AnEmptyFallbackFrameDoesNotProduceAMessageWithAHoleInIt) {
  // An empty frame must not print as "nor  -> base_link", which reads as a truncated sentence
  // rather than a setting the operator has to fill in.
  const Check check = CheckFrames("map", "", "base_link", false, false);
  EXPECT_EQ(check.severity, Severity::kFatal);
  EXPECT_TRUE(Mentions(check, "fallback_reference_frame"));
  EXPECT_EQ(check.detail.find("nor  ->"), std::string::npos);
}

TEST(SimTime, TheFatalMessageMustNotContradictTheClockPolicy) {
  // It must not claim "the session's own timeline never advances", which this SDK's whole clock
  // policy exists to prevent: wall clock goes on the wire, the sample timer is a WALL timer, and
  // the pose sampler asks for TimePointZero. The honest reason it is fatal is that the simulator is
  // not running, so there is no robot to record.
  const Check check = CheckSimTime(true, false);
  EXPECT_EQ(check.severity, Severity::kFatal);
  EXPECT_EQ((check.detail + check.remedy).find("timeline never advances"), std::string::npos);
  EXPECT_TRUE(Mentions(check, "not there") || Mentions(check, "not running"));
}

// --- the report -------------------------------------------------------------------------------

// A report pasted into a support conversation says which SDK, distro and RMW produced it.
TEST(Render, TheHeaderNamesTheSdkVersionAndTheRuntime) {
  const std::string report = Render({CheckApplicationKey("")}, "ROS 2 humble, rmw_fastrtps_cpp");
  const std::string header = report.substr(0, report.find('\n'));
  EXPECT_NE(header.find(c3d::kSdkVersion), std::string::npos) << header;
  EXPECT_NE(header.find("ROS 2 humble, rmw_fastrtps_cpp"), std::string::npos) << header;
}

TEST(Render, EveryNonOkCheckPrintsARemedy) {
  // The invariant that makes this tool worth running: a problem is never reported without an
  // action. Enforced over the real constructors rather than over hand-built structs.
  const std::vector<Check> checks = {
      CheckApplicationKey(""),
      CheckSpoolDirectory("", false, 0),
      CheckSimTime(true, false),
      CheckFrames("map", "odom", "base_link", false, false),
      CheckGazeFrame("cam", false),
      CheckGateway("https", "h", 0, false, "refused"),
      CheckGateway("https", "h", 200, false, ""),
      CheckSpoolDirectory("/tmp/x", true, 3),
      CheckSimTime(false, true),
      CheckFrames("map", "odom", "base_link", false, true),
  };
  for (const auto& check : checks) {
    EXPECT_NE(check.severity, Severity::kOk) << check.name;
    EXPECT_FALSE(check.remedy.empty()) << check.name << " reports a problem with no action";
  }
  const std::string report = Render(checks);
  EXPECT_NE(report.find("FAIL"), std::string::npos);
  for (const auto& check : checks) {
    EXPECT_NE(report.find(check.name), std::string::npos) << check.name << " missing from report";
  }
}

TEST(Render, AnEmptyRunSaysItIsADefectRatherThanPrintingAPass) {
  const std::string report = Render({});
  EXPECT_NE(report.find("no checks ran"), std::string::npos);
  EXPECT_EQ(report.find("  OK."), std::string::npos);
}

TEST(Render, AdvisoriesOnlyReadsAsPassingWithNotes) {
  const std::string report = Render({CheckFrames("map", "odom", "base_link", false, true)});
  EXPECT_NE(report.find("with notes"), std::string::npos);
  EXPECT_EQ(report.find("FAIL"), std::string::npos);
}

// The camera encoder is the host's ffmpeg. With a camera configured, one that cannot run is FATAL
// -- no camera could record -- and names the package; with none, nothing runs it, so nothing is
// checked. A pass names the binary and the version it answered with. Configure refuses for the same
// reason, and doctor reads that refusal back into this same check.
TEST(DoctorChecks, TheCameraEncoderCheckIsFatalOnlyWhenACameraIsConfigured) {
  CameraEncoderFacts facts;
  facts.ffmpeg_path = "/opt/ffmpeg/bin/ffmpeg";
  facts.problem = "cannot run /opt/ffmpeg/bin/ffmpeg: No such file or directory";

  const Check no_camera = CheckCameraEncoder(facts);
  EXPECT_EQ(no_camera.severity, Severity::kOk);
  EXPECT_NE(no_camera.name.find("(NOT CHECKED)"), std::string::npos) << no_camera.name;

  facts.cameras = 2;
  const Check fatal = CheckCameraEncoder(facts);
  EXPECT_EQ(fatal.name, "camera encoder");
  EXPECT_EQ(fatal.severity, Severity::kFatal);
  EXPECT_NE(fatal.detail.find("ffmpeg_path '/opt/ffmpeg/bin/ffmpeg'"), std::string::npos)
      << fatal.detail;
  EXPECT_NE(fatal.detail.find("No such file or directory"), std::string::npos) << fatal.detail;
  EXPECT_NE(fatal.remedy.find("sudo apt install ffmpeg"), std::string::npos) << fatal.remedy;

  facts.problem.clear();
  facts.version = "ffmpeg version 6.1.1-3ubuntu5 Copyright (c) 2000-2023 the FFmpeg developers";
  const Check pass = CheckCameraEncoder(facts);
  EXPECT_EQ(pass.name, "camera encoder");
  EXPECT_EQ(pass.severity, Severity::kOk);
  EXPECT_NE(pass.detail.find("/opt/ffmpeg/bin/ffmpeg"), std::string::npos) << pass.detail;
  EXPECT_NE(pass.detail.find("ffmpeg version 6.1.1"), std::string::npos) << pass.detail;

  const auto refused = ChecksForConfigurationRefusal(
      cognitive3d_ros::FfmpegProblem("/nonexistent", "cannot run /nonexistent") + ". " +
      cognitive3d_ros::kFfmpegRemedy);
  ASSERT_EQ(refused.size(), 1u);
  EXPECT_EQ(refused[0].name, "camera encoder");
  EXPECT_EQ(refused[0].severity, Severity::kFatal);
  EXPECT_EQ(refused[0].detail,
            cognitive3d_ros::FfmpegProblem("/nonexistent", "cannot run /nonexistent"));
  EXPECT_NE(refused[0].remedy.find("sudo apt install ffmpeg"), std::string::npos);
}

// A camera's bytes cannot be measured from one frame, so they are estimated: 5120 bytes a frame at
// 640x480 and crf 28, in proportion to the pixels, doubling every 4 crf steps lower -- the slope
// the recorded footage showed -- times the rate. The check says it is an estimate.
TEST(DoctorChecks, TheCameraSizeIsAnEstimateFromResolutionAndCrf) {
  const double base = EstimatedCameraBytesPerSecond(640, 480, 28, 30.0);
  EXPECT_DOUBLE_EQ(base, 153600.0);
  EXPECT_DOUBLE_EQ(EstimatedCameraBytesPerSecond(640, 480, 24, 30.0), 2.0 * base);
  EXPECT_DOUBLE_EQ(EstimatedCameraBytesPerSecond(640, 480, 32, 30.0), 0.5 * base);
  EXPECT_DOUBLE_EQ(EstimatedCameraBytesPerSecond(320, 240, 28, 30.0), 0.25 * base);
  EXPECT_DOUBLE_EQ(EstimatedCameraBytesPerSecond(640, 480, 28, 15.0), 0.5 * base);

  MediaSensorFacts camera;
  camera.kind = "camera";
  camera.name = "front";
  camera.topic = "/image";
  camera.rate_hz = 30.0;
  camera.topic_present = true;
  camera.type = "sensor_msgs/msg/Image";
  camera.type_handled = true;
  camera.qos = "best_effort/volatile/keep_last(1)";
  camera.frame = "camera_link";
  camera.frame_resolves = true;
  camera.sample_seen = true;
  camera.prepared_width = 640;
  camera.prepared_height = 480;
  camera.crf = 24;
  const Check check = CheckMediaSensor(camera);
  EXPECT_EQ(check.severity, Severity::kOk);
  EXPECT_NE(check.detail.find("about 300.0 KB/s estimated"), std::string::npos) << check.detail;
  EXPECT_NE(check.detail.find("crf 24"), std::string::npos) << check.detail;
  EXPECT_NE(check.detail.find("chunks of 3 s, about 900.0 KB each"), std::string::npos)
      << check.detail;
}

// Doctor sizes a camera's chunks from its live sample by the node's own rule: the chunk length it
// prints is the one the encoder would start with, and a sample at which not even a 1-second chunk
// fits fails the check, as the node would refuse the camera at its first frame.
TEST(DoctorChecks, TheCameraCheckPrintsTheChunkLengthAndFailsWhenNoneFits) {
  MediaSensorFacts camera;
  camera.kind = "camera";
  camera.name = "front";
  camera.topic = "/image";
  camera.rate_hz = 30.0;
  camera.topic_present = true;
  camera.type = "sensor_msgs/msg/Image";
  camera.type_handled = true;
  camera.qos = "best_effort/volatile/keep_last(1)";
  camera.frame = "camera_link";
  camera.frame_resolves = true;
  camera.sample_seen = true;
  camera.crf = 24;
  camera.prepared_width = 2560;
  camera.prepared_height = 1440;
  Check check = CheckMediaSensor(camera);
  EXPECT_EQ(check.severity, Severity::kOk);
  EXPECT_NE(check.detail.find("chunks of 1.7 s"), std::string::npos) << check.detail;

  camera.prepared_width = 3840;
  camera.prepared_height = 2160;
  check = CheckMediaSensor(camera);
  EXPECT_EQ(check.severity, Severity::kFatal);
  for (const char* lever : {"3840x2160", "max_width_px", "crf", "max_bitrate_kbps"}) {
    EXPECT_NE(check.detail.find(lever), std::string::npos) << lever << ": " << check.detail;
  }

  camera.max_bitrate_kbps = 20000;
  check = CheckMediaSensor(camera);
  EXPECT_EQ(check.severity, Severity::kOk);
  EXPECT_NE(check.detail.find("capped by max_bitrate_kbps"), std::string::npos) << check.detail;
  EXPECT_NE(check.detail.find("chunks of 2.5 s"), std::string::npos) << check.detail;
  EXPECT_NE(check.detail.find("the bitrate is capped at max_bitrate_kbps 20000"),
            std::string::npos)
      << check.detail;
}

// Doctor names the regime the node would run the camera in: no cap while the replay store holds
// what a size restart re-encodes, a default cap where it cannot.
TEST(DoctorChecks, TheCameraCheckNamesTheBitrateRegime) {
  MediaSensorFacts camera;
  camera.kind = "camera";
  camera.name = "front";
  camera.topic = "/image";
  camera.rate_hz = 15.0;
  camera.topic_present = true;
  camera.type = "sensor_msgs/msg/Image";
  camera.type_handled = true;
  camera.qos = "best_effort/volatile/keep_last(1)";
  camera.frame = "camera_link";
  camera.frame_resolves = true;
  camera.sample_seen = true;
  camera.crf = 28;
  camera.replay_max_mb = 192;
  camera.prepared_width = 1920;
  camera.prepared_height = 1080;
  Check check = CheckMediaSensor(camera);
  EXPECT_NE(check.detail.find("no bitrate cap"), std::string::npos) << check.detail;
  camera.prepared_width = 3840;
  camera.prepared_height = 2160;
  check = CheckMediaSensor(camera);
  EXPECT_EQ(check.severity, Severity::kOk);
  EXPECT_NE(check.detail.find("overflow replay_max_mb 192, so the bitrate is capped at 16777 kbps"),
            std::string::npos)
      << check.detail;
}

// `Config::Validate` reports every missing mandatory value in ONE string, so a refusal naming
// several problems must reach several checks. Dispatching on the joined string names the first
// and drops the rest, which is the round trip per unset field that joining them removes.
TEST(ConfigurationRefusal, ASingleProblemStillReachesItsOwnCheck) {
  const auto checks = ChecksForConfigurationRefusal("application_key is empty");
  ASSERT_EQ(checks.size(), 1u);
  EXPECT_EQ(checks[0].name, "application key");
  EXPECT_EQ(checks[0].severity, Severity::kFatal);
  EXPECT_NE(checks[0].remedy.find("C3D_APPLICATION_API_KEY"), std::string::npos);
}

TEST(ConfigurationRefusal, EveryProblemInAJoinedRefusalIsReported) {
  const auto checks = ChecksForConfigurationRefusal(
      "3 configuration problems: application_key is empty | spool_dir is empty | app_name is "
      "empty");
  ASSERT_EQ(checks.size(), 3u);
  EXPECT_EQ(checks[0].name, "application key");
  EXPECT_EQ(checks[1].name, "spool directory");
  EXPECT_EQ(checks[2].name, "configuration");
  // The count prefix belongs to the join, not to any one problem.
  EXPECT_EQ(checks[2].detail, "app_name is empty");
  for (const auto& check : checks) {
    EXPECT_EQ(check.severity, Severity::kFatal);
    EXPECT_FALSE(check.remedy.empty()) << check.name << " reports a problem with no action";
  }
}

TEST(ConfigurationRefusal, UnrecognisedProblemsAreReportedTogetherRatherThanRepeatingOneRemedy) {
  const auto checks = ChecksForConfigurationRefusal(
      "3 configuration problems: scene_id is empty | scene_version is empty | device_id is empty");
  ASSERT_EQ(checks.size(), 1u);
  EXPECT_EQ(checks[0].name, "configuration");
  EXPECT_NE(checks[0].detail.find("scene_id"), std::string::npos);
  EXPECT_NE(checks[0].detail.find("scene_version"), std::string::npos);
  EXPECT_NE(checks[0].detail.find("device_id"), std::string::npos);
}

TEST(ConfigurationRefusal, ARefusalWithNoMessageIsStillFatal) {
  const auto checks = ChecksForConfigurationRefusal("");
  ASSERT_EQ(checks.size(), 1u);
  EXPECT_EQ(checks[0].severity, Severity::kFatal);
  EXPECT_EQ(checks[0].detail, "rejected");
  EXPECT_FALSE(checks[0].remedy.empty());
}

// The count prefix is stripped only when it IS a count. A message that happens to contain the
// words must survive whole, or a real problem gets truncated out of the report.
TEST(ConfigurationRefusal, TextResemblingTheCountPrefixIsNotStripped) {
  const auto checks =
      ChecksForConfigurationRefusal("scene_id names configuration problems: check the state file");
  ASSERT_EQ(checks.size(), 1u);
  EXPECT_EQ(checks[0].detail, "scene_id names configuration problems: check the state file");
}

// The joined format is produced in core and consumed here, and the tests above type it by hand.
// So this one asks the real producer for the string: change the separator or the count grammar in
// `Config::Validate` and this fails, where a hand-typed fixture would keep passing against a
// string that no longer occurs.
TEST(ConfigurationRefusal, TheSplitterIsPinnedToWhatValidateActuallyProduces) {
  c3d::Config config;
  config.application_key.clear();
  config.spool_dir.clear();
  config.app_name.clear();
  const std::string refusal = config.Validate();
  ASSERT_FALSE(refusal.empty()) << "Validate accepted a config with three mandatory values unset";

  const auto checks = ChecksForConfigurationRefusal(refusal);
  bool saw_key = false;
  bool saw_spool = false;
  bool saw_app_name = false;
  for (const auto& check : checks) {
    if (check.name == "application key") saw_key = true;
    if (check.name == "spool directory") saw_spool = true;
    if (check.detail.find("app_name") != std::string::npos) saw_app_name = true;
    EXPECT_FALSE(check.remedy.empty()) << check.name << " reports a problem with no action";
  }
  EXPECT_TRUE(saw_key) << refusal;
  EXPECT_TRUE(saw_spool) << refusal;
  EXPECT_TRUE(saw_app_name) << refusal;
  // No problem may be reported twice: a duplicated check reads as two faults to fix.
  EXPECT_EQ(checks.size(), 3u) << refusal;
}

// The application key belongs to a project, and a key from the wrong one ingests there completely
// with no error. FATAL path first, then every way the check declines -- declining must never read
// as a pass, which is why the skipped cases carry a different NAME, not just a different detail.
namespace {
constexpr const char* kKey = "ABCDEFGHIJKLMNOPQRSTUVWXYZ012345";  // 32 chars, maskable

ProjectKeyFacts Answered(bool listed) {
  ProjectKeyFacts f;
  f.application_key = kKey;
  f.expected_project = "90005";
  f.org_key_present = true;
  f.destination_permitted = true;
  f.status = 200;
  f.body_is_a_key_list = true;
  f.key_listed = listed;
  return f;
}
}  // namespace

TEST(ProjectKey, AKeyThatIsNotOneOfTheProjectsKeysIsFatalAndSaysWhereTheDataWouldGo) {
  const Check check = CheckProjectKey(Answered(false));
  EXPECT_EQ(check.severity, Severity::kFatal);
  EXPECT_NE(check.detail.find("90005"), std::string::npos);
  EXPECT_NE(check.remedy.find("no error"), std::string::npos);
  EXPECT_EQ(check.detail.find(kKey), std::string::npos) << "the key must be masked, never echoed";
  EXPECT_EQ(check.remedy.find(kKey), std::string::npos);
}

TEST(ProjectKey, AKeyThatIsOneOfTheProjectsKeysPasses) {
  const Check check = CheckProjectKey(Answered(true));
  EXPECT_EQ(check.severity, Severity::kOk);
  EXPECT_EQ(check.detail.find(kKey), std::string::npos);
}

// Every skipped path. The NAME must not assert membership, because a reader skims names.
TEST(ProjectKey, EveryWayOfDecliningIsLabelledNotCheckedRatherThanOk) {
  ProjectKeyFacts no_project = Answered(false); no_project.expected_project = "";
  ProjectKeyFacts no_org = Answered(false); no_org.org_key_present = false;
  ProjectKeyFacts bad_dest = Answered(false); bad_dest.destination_permitted = false;
  ProjectKeyFacts short_key = Answered(false); short_key.application_key = "tiny";
  for (const auto& facts : {no_project, no_org, bad_dest, short_key}) {
    const Check check = CheckProjectKey(facts);
    EXPECT_NE(check.severity, Severity::kFatal);
    EXPECT_NE(check.name.find("NOT CHECKED"), std::string::npos)
        << "a skipped check must not be skim-readable as a passing one: " << check.name;
    EXPECT_EQ(check.name.find("belongs to"), std::string::npos);
  }
}

// doctor's output is pasted whole into public support channels. Of the application key it carries
// the last four characters, on one line, and no other run of the key's characters on any line, in
// both of key/project's verdicts. The key is made of runs no English word or check name contains.
TEST(ProjectKey, AReportCarriesTheLastFourOfTheKeyOnceAndNothingElseOfIt) {
  const std::string key = "Qx7Zk2Wv9Bj4Mn8Pr3Ty6Lg5";
  const std::string last_four = key.substr(key.size() - 4);
  for (const bool listed : {true, false}) {
    ProjectKeyFacts facts = Answered(listed);
    facts.application_key = key;
    const std::string report =
        Render({CheckApplicationKey(key), CheckProjectKey(facts)}, "ROS 2 jazzy, rmw_fastrtps_cpp");
    std::istringstream lines(report);
    std::string line;
    int carrying_last_four = 0;
    while (std::getline(lines, line)) {
      for (std::size_t at = 0; at + 4 <= key.size(); ++at) {
        const std::string run = key.substr(at, 4);
        if (line.find(run) == std::string::npos) continue;
        EXPECT_EQ(run, last_four) << "four characters of the key other than its last: " << line;
        if (run == last_four) {
          ++carrying_last_four;
          EXPECT_NE(line.find("application key"), std::string::npos) << line;
        }
      }
    }
    EXPECT_EQ(carrying_last_four, 1) << report;
  }
}

// The distinction the name alone cannot carry to a severity-only consumer: NOBODY ASKED is a pass,
// ASKED AND UNANSWERED is a warning. Both of these decline; only one of them is fine.
TEST(ProjectKey, NotApplicableIsOkButAskedAndUnverifiedIsAdvisory) {
  ProjectKeyFacts nobody_asked = Answered(false);
  nobody_asked.expected_project = "";
  EXPECT_EQ(CheckProjectKey(nobody_asked).severity, Severity::kOk);

  for (auto mutate : std::vector<std::function<void(ProjectKeyFacts&)>>{
           [](ProjectKeyFacts& f) { f.org_key_present = false; },
           [](ProjectKeyFacts& f) { f.destination_permitted = false; },
           [](ProjectKeyFacts& f) { f.application_key = "tiny"; }}) {
    ProjectKeyFacts asked = Answered(false);
    mutate(asked);
    const Check check = CheckProjectKey(asked);
    EXPECT_EQ(check.severity, Severity::kAdvisory)
        << "an unanswered check that a severity-only consumer reads as kOk reads as verified";
    EXPECT_FALSE(check.remedy.empty()) << "anything not kOk must name the action";
  }
}

// A short key cannot be told from an ordinary JSON token, so containment would answer about the
// wrong thing -- `id` would match the field name and pass.
TEST(ProjectKey, AKeyTooShortToIdentifySafelyIsNotCheckedRatherThanPassed) {
  ProjectKeyFacts facts = Answered(true);
  facts.application_key = "id";
  const Check check = CheckProjectKey(facts);
  EXPECT_NE(check.name.find("NOT CHECKED"), std::string::npos);
  EXPECT_EQ(check.severity, Severity::kAdvisory);
}

// An org key unlocks every project in the org. It goes to an allowlisted host over https or the
// check does not run.
TEST(ProjectKey, AnImpermissibleDestinationSkipsRatherThanSendingTheOrganisationKey) {
  ProjectKeyFacts facts = Answered(true);
  facts.destination_permitted = false;
  const Check check = CheckProjectKey(facts);
  EXPECT_NE(check.name.find("NOT CHECKED"), std::string::npos);
  EXPECT_NE(check.detail.find("https"), std::string::npos);
}

// A key list for the project proves the organisation key, whichever keys are on it.
TEST(ProjectKey, OnlyAKeyListProvesTheOrganisationKey) {
  EXPECT_TRUE(OrganizationKeyProven(Answered(true)));
  EXPECT_TRUE(OrganizationKeyProven(Answered(false)))
      << "an application key from another project says nothing against the organisation key";

  ProjectKeyFacts not_a_list = Answered(true);
  not_a_list.body_is_a_key_list = false;
  ProjectKeyFacts refused = Answered(true);
  refused.status = 401;
  ProjectKeyFacts failed = Answered(true);
  failed.status = 503;
  ProjectKeyFacts unasked;
  for (const auto& facts : {not_a_list, refused, failed, unasked}) {
    EXPECT_FALSE(OrganizationKeyProven(facts)) << facts.status;
  }
}

// The api host comes from the process environment, through the node; the gateway never moves it.
// Asserted on the function the probes ACTUALLY CALL, with every hostile gateway beside the api host
// the environment resolved.
TEST(ApiDestination, NoGatewayValueCanMoveWhereAnOrganisationKeyIsSent) {
  c3d::Config baseline;
  baseline.protocol = "https";
  const std::string api = "api.example.test";
  const ApiDestination expected = ApiDestinationFor(baseline, api);
  ASSERT_TRUE(expected.permitted);
  ASSERT_EQ(expected.host, api);

  for (const char* hostile : {"data.somewhere-else.example", "data.cognitive3d.com.evil.example",
                              "api.cognitive3d.com", "localhost:8080", "",
                              "data.cognitive3d.com@evil.example", "api.example.test.evil"}) {
    c3d::Config config = baseline;
    config.gateway = hostile;
    const ApiDestination actual = ApiDestinationFor(config, api);
    EXPECT_EQ(actual.host, expected.host) << "gateway moved the api host: " << hostile;
    EXPECT_EQ(actual.permitted, expected.permitted) << hostile;
  }
}

TEST(ApiDestination, ProtocolDecidesWhetherToSendAtAllAndNoHostPermitsNothing) {
  c3d::Config capture;
  capture.protocol = "http";
  // A loopback capture must not reach out to Cognitive3D with a key that unlocks the whole org.
  EXPECT_FALSE(ApiDestinationFor(capture, "api.example.test").permitted);
  EXPECT_EQ(ApiDestinationFor(capture, "api.example.test").host, "api.example.test")
      << "the host is still known; what changes is whether anything may be sent to it";

  c3d::Config https;
  https.protocol = "https";
  EXPECT_TRUE(ApiDestinationFor(https, "").host.empty());
  EXPECT_FALSE(ApiDestinationFor(https, "").permitted);
}

TEST(SceneLookupBase, TheSceneIsReadFromTheResolvedApiHostOverHttpsAndNoGatewayMovesIt) {
  c3d::Config baseline;
  baseline.protocol = "https";
  for (const char* hostile : {"data.somewhere-else.example", "data.cognitive3d.com.evil.example",
                              "api.cognitive3d.com", "localhost:8080", "",
                              "data.cognitive3d.com@evil.example", "api.example.test.evil"}) {
    c3d::Config config = baseline;
    config.gateway = hostile;
    EXPECT_EQ(SceneLookupBase(config, "api.example.test"), "https://api.example.test") << hostile;
  }
  EXPECT_EQ(SceneLookupBase(baseline, "127.0.0.1:8443"), "https://127.0.0.1:8443");

  c3d::Config capture = baseline;
  capture.protocol = "http";
  capture.gateway = "127.0.0.1:8080";
  EXPECT_EQ(SceneLookupBase(capture, "api.example.test"), "")
      << "a loopback capture sends no organisation key anywhere";
  EXPECT_EQ(SceneLookupBase(baseline, ""), "");
}

TEST(Environment, TheCheckReportsTheDescriptionItIsGiven) {
  const Check check = CheckEnvironment("custom hosts (C3D_DATA_HOST / C3D_API_HOST): telemetry to "
                                       "data.example.test, metadata from api.example.test");
  EXPECT_EQ(check.name, "environment");
  EXPECT_EQ(check.severity, Severity::kOk);
  EXPECT_TRUE(Mentions(check, "C3D_DATA_HOST"));
}

// The three ways the check runs and cannot conclude. All ADVISORY: a check that did not run must
// not fail a preflight, and an unreadable answer is not an absent key.
TEST(ProjectKey, AnUnreachableApiIsAdvisoryAndSaysItIsNotAVerdict) {
  ProjectKeyFacts facts = Answered(false);
  facts.status = 0;
  facts.transport_error = "Couldn't resolve host name";
  const Check check = CheckProjectKey(facts);
  EXPECT_EQ(check.severity, Severity::kAdvisory);
  EXPECT_NE(check.remedy.find("did not run"), std::string::npos);
}

TEST(ProjectKey, ARefusedOrganisationKeyIsAdvisoryAndBlamesTheOrgKeyNotTheAppKey) {
  for (const long status : {401L, 403L}) {
    ProjectKeyFacts facts = Answered(false);
    facts.status = status;
    const Check check = CheckProjectKey(facts);
    EXPECT_EQ(check.severity, Severity::kAdvisory) << status;
    EXPECT_NE(check.remedy.find("C3D_ORG_API_KEY"), std::string::npos) << status;
  }
}

// A 200 whose body is an error document or empty is NOT evidence the key is absent.
TEST(ProjectKey, ATwoHundredWithoutAKeyListIsAdvisoryNotFatal) {
  ProjectKeyFacts facts = Answered(false);
  facts.status = 200;
  facts.body_is_a_key_list = false;
  const Check check = CheckProjectKey(facts);
  EXPECT_EQ(check.severity, Severity::kAdvisory);
  EXPECT_NE(check.remedy.find("not the same as"), std::string::npos);
}

TEST(ProjectKey, AnyOtherStatusIsAdvisoryRatherThanAVerdictOnTheKey) {
  ProjectKeyFacts facts = Answered(false);
  facts.status = 500;
  facts.body_is_a_key_list = false;
  EXPECT_EQ(CheckProjectKey(facts).severity, Severity::kAdvisory);
}

TEST(OrganizationAuthorizationHeader, APrefixedKeyIsSentVerbatimRatherThanPrefixedTwice) {
  const std::string sent = OrganizationAuthorization("orgkey-abc123");
  EXPECT_EQ(sent, "orgkey-abc123");
  EXPECT_EQ(sent.find("orgkey-orgkey-"), std::string::npos);
  EXPECT_EQ(sent.find("APIKEY:ORGANIZATION"), std::string::npos);
}

TEST(OrganizationAuthorizationHeader, ALegacyKeyTakesTheOrganizationPrefix) {
  EXPECT_EQ(OrganizationAuthorization("abc123"), "APIKEY:ORGANIZATION abc123");
}

TEST(OrganizationAuthorizationHeader, AnEmptyKeyIsNotSpeciallyCasedHereButNeverReachesTheWire) {
  // CheckProjectKey refuses to run without an org key, so this only pins that the helper itself
  // does not invent a credential shape when handed nothing.
  EXPECT_EQ(OrganizationAuthorization(""), "APIKEY:ORGANIZATION ");
}

// --- the pose convention, whose misconfiguration is invisible downstream ----------------------

// There is no default, so a configured node always reports a value somebody chose, and every
// value is reported as it is with no advisory about where it came from.
TEST(PoseConvention, EveryConfiguredValueIsReportedAsItIs) {
  for (const char* convention : {"unity", "gltf_authored", "rep103"}) {
    const Check check = CheckPoseConvention(convention);
    EXPECT_EQ(check.name, "pose/convention");
    EXPECT_EQ(check.severity, Severity::kOk) << convention;
    EXPECT_TRUE(Mentions(check, convention)) << convention;
  }
}

TEST(PoseConvention, UnsetIsFatalAndTheRemedyNamesTheChoice) {
  const Check check = CheckPoseConvention("");
  EXPECT_EQ(check.severity, Severity::kFatal);
  EXPECT_TRUE(Mentions(check, "gltf_authored"));
  EXPECT_TRUE(Mentions(check, "unity"));
  // That a mismatch cannot be caught downstream is the entire reason the value is required, so
  // the message has to say it. Otherwise the reader picks one at random.
  EXPECT_TRUE(Mentions(check, "NOTHING downstream"));
}

// The node's refusal reaches the same check, joined with whatever else is missing.
TEST(PoseConvention, TheNodesRefusalMapsToTheConventionCheck) {
  const auto checks = ChecksForConfigurationRefusal(
      "2 configuration problems: pose_convention is not set, and it has no default: whatever "
      "follows | scene_id is empty; every payload would be discarded");
  ASSERT_EQ(checks.size(), 2u);
  EXPECT_EQ(checks[0].name, "pose/convention");
  EXPECT_EQ(checks[0].severity, Severity::kFatal);
  EXPECT_EQ(checks[1].name, "configuration");
  EXPECT_TRUE(Mentions(checks[1], "scene_id"));
}

// --- the robot mesh, whose absence renders a replay with no robot in it ----------------------

TEST(MeshNames, ExtractsEveryValueAndReportsAReadableList) {
  std::vector<std::string> names;
  EXPECT_TRUE(StringFieldIn(
      R"([{"id":1,"meshName":"turtlebot4","name":"TurtleBot 4 Lite"},)"
      R"({"id":2,"meshName":"robot_placeholder"}])", "meshName", &names));
  ASSERT_EQ(names.size(), 2u);
  EXPECT_EQ(names[0], "turtlebot4");
  EXPECT_EQ(names[1], "robot_placeholder");
}

TEST(MeshNames, AnEmptyArrayIsReadableAndHasNoNames) {
  // The exact state a discarded upload leaves behind, and the one case where absence is PROVEN.
  std::vector<std::string> names;
  EXPECT_TRUE(StringFieldIn("[]", "meshName", &names));
  EXPECT_TRUE(names.empty());
}

TEST(MeshNames, AnythingThatIsNotAnArrayIsUNREADABLERatherThanEmpty) {
  // The distinction the whole signature exists for. An error object that merely mentions a bracket
  // must stay unreadable, not become a definitive absence: that would turn an advisory into a
  // FATAL.
  for (const char* body : {R"({"error":"invalid request [project 90001]"})",
                           R"({"error":"upstream returned [] while listing objects"})",
                           R"([{"meshName":"other"})",           // truncated: not valid JSON
                           "not json at all", ""}) {
    std::vector<std::string> names;
    EXPECT_FALSE(StringFieldIn(body, "meshName", &names)) << body;
  }
}

TEST(MeshNames, JsonEscapesResolveSoAnExactCompareIsActuallyExact) {
  // A hand-rolled scan keeps these verbatim, and the exact comparison this function exists to
  // enable then reports a mesh that IS present as absent -- which is fatal. `\u0034` is the
  // sharpest case: a scanner yields "turtlebot\u0034", a parser yields "turtlebot4", and only
  // one of those equals the parameter the operator typed.
  std::vector<std::string> names;
  ASSERT_TRUE(StringFieldIn(
      R"([{"meshName":"turtlebot\u0034"},{"meshName":"with\\backslash"},)"
      R"({"meshName":"with\"quote"}])", "meshName", &names));
  ASSERT_EQ(names.size(), 3u);
  EXPECT_EQ(names[0], "turtlebot4");
  EXPECT_EQ(names[1], std::string("with") + "\\" + "backslash");
  EXPECT_EQ(names[2], std::string("with") + '"' + "quote");
}

TEST(MeshNames, ObjectsWithoutAMeshNameAreSkippedNotFatal) {
  std::vector<std::string> names;
  EXPECT_TRUE(StringFieldIn(R"([{"id":1},{"meshName":"only"},{"meshName":42}])", "meshName", &names));
  ASSERT_EQ(names.size(), 1u);
  EXPECT_EQ(names[0], "only");
}

TEST(RobotMesh, AMeshThatIsNotThereIsFatalAndNamesWhatIs) {
  cognitive3d_ros::doctor::RobotMeshFacts facts;
  facts.robot_mesh = "robot_placeholder";
  facts.project = "90001";
  facts.org_key_present = true;
  facts.destination_permitted = true;
  facts.status = 200;
  facts.body_is_an_object_list = true;
  facts.available = {"turtlebot4"};
  facts.mesh_listed = false;
  const Check check = CheckRobotMesh(facts);
  EXPECT_EQ(check.severity, Severity::kFatal);
  EXPECT_TRUE(Mentions(check, "robot_placeholder"));
  // Naming what the project DOES have is the difference between ending a support conversation
  // and starting one.
  EXPECT_TRUE(Mentions(check, "turtlebot4"));
  // The reason it is fatal rather than a warning has to be in the message.
  EXPECT_TRUE(Mentions(check, "NO ROBOT IN IT"));
  // And the upload's own 200 must not be trusted as evidence against this check.
  EXPECT_TRUE(Mentions(check, ".glb"));
}

TEST(RobotMesh, AProjectWithNoObjectsAtAllSaysSo) {
  cognitive3d_ros::doctor::RobotMeshFacts facts;
  facts.robot_mesh = "turtlebot4";
  facts.project = "90001";
  facts.org_key_present = true;
  facts.destination_permitted = true;
  facts.status = 200;
  facts.body_is_an_object_list = true;
  facts.mesh_listed = false;
  const Check check = CheckRobotMesh(facts);
  EXPECT_EQ(check.severity, Severity::kFatal);
  EXPECT_TRUE(Mentions(check, "none at all"));
}

namespace {

// The exact shape of a `GET /v0/versions/<id>/objects?includeFiles=true` answer, with synthetic
// ids. The shape is the platform's own rather than a simplified one, because the field this check
// turns on -- `files` nested inside each object row -- is the one a simplified fixture gets wrong.
const char* const kObjectsWithFiles = R"([
  {"id":50001,"versionId":70001,"meshName":"robot_placeholder_v3","name":"Example Robot",
   "files":[
     {"id":60001,"type":"gltf","name":"robot_placeholder_v3.gltf",
      "location":"objects/robot_placeholder_v3/","forDynamic":true},
     {"id":60002,"type":"bin","name":"robot_placeholder_v3.bin",
      "location":"objects/robot_placeholder_v3/","forDynamic":true}],
   "sdkId":"robot"}
])";

// Call 1 without call 2: the row exists and `files` is empty, behind nothing but 200s.
const char* const kObjectsWithoutFiles = R"([
  {"id":50001,"versionId":70001,"meshName":"robot_placeholder","name":"Example Robot",
   "files":[],"sdkId":"robot"}
])";

const char* const kSceneDocument = R"({
  "id":"a1b2c3d4-0000-4000-8000-000000000001","projectId":90003,"sceneName":"World",
  "versions":[{"id":70001,"sceneId":"a1b2c3d4-0000-4000-8000-000000000001","versionNumber":1}]
})";

cognitive3d_ros::doctor::RobotMeshFilesFacts AnsweredFilesFacts() {
  cognitive3d_ros::doctor::RobotMeshFilesFacts facts;
  facts.robot_mesh = "robot_placeholder_v3";
  facts.scene_id = "a1b2c3d4-0000-4000-8000-000000000001";
  facts.scene_version = "1";
  facts.org_key_present = true;
  facts.destination_permitted = true;
  facts.scene_status = 200;
  facts.scene_body_is_readable = true;
  facts.scene_read_by_scene_project = true;
  facts.version_id = "70001";
  facts.status = 200;
  return facts;
}

}  // namespace

TEST(VersionIdForNumber, ReadableAndAbsentIsNotTheSameAnswerAsUnreadable) {
  // The distinction is the whole point: a readable scene that does not list the version is a
  // FATAL answer about where the run is pointed, while an unreadable body is merely unverified.
  // Returning one empty string for both would let a malformed 200 read as "this version does not
  // exist".
  std::string id;
  EXPECT_TRUE(cognitive3d_ros::doctor::VersionIdForNumber(kSceneDocument, "1", &id));
  EXPECT_EQ(id, "70001");

  // Compared as numbers, as configure reads scene_version.
  EXPECT_TRUE(cognitive3d_ros::doctor::VersionIdForNumber(kSceneDocument, "01", &id));
  EXPECT_EQ(id, "70001");

  // Readable, no such version: answerable.
  EXPECT_TRUE(cognitive3d_ros::doctor::VersionIdForNumber(kSceneDocument, "2", &id));
  EXPECT_EQ(id, "");
  EXPECT_TRUE(cognitive3d_ros::doctor::VersionIdForNumber(kSceneDocument, "10", &id));
  EXPECT_EQ(id, "");

  // The scene UUID is not a version number. The objects route rejects it outright, so resolving
  // it here would turn a clear 400 into a confusing 404.
  EXPECT_TRUE(cognitive3d_ros::doctor::VersionIdForNumber(
      kSceneDocument, "a1b2c3d4-0000-4000-8000-000000000001", &id));
  EXPECT_EQ(id, "");

  // Unreadable: not a claim about any version.
  for (const char* body : {"not json", "[]", "{}", R"({"versions":"soon"})"}) {
    EXPECT_FALSE(cognitive3d_ros::doctor::VersionIdForNumber(body, "1", &id)) << body;
    EXPECT_EQ(id, "");
  }
  // The version IS listed and its id is unusable. That proves nothing either way, so it must not
  // read as "no such version".
  EXPECT_FALSE(cognitive3d_ros::doctor::VersionIdForNumber(
      R"({"versions":[{"versionNumber":1}]})", "1", &id));
}

TEST(MeshFilesIn, FoundWithNoFilesIsNotTheSameAsUnreadable) {
  bool found = false;
  std::vector<std::string> files;
  ASSERT_TRUE(cognitive3d_ros::doctor::MeshFilesIn(kObjectsWithFiles, "robot_placeholder_v3",
                                                   &found, &files));
  EXPECT_TRUE(found);
  EXPECT_EQ(files.size(), 2u);

  found = false;
  files.clear();
  ASSERT_TRUE(cognitive3d_ros::doctor::MeshFilesIn(kObjectsWithoutFiles, "robot_placeholder",
                                                   &found, &files));
  EXPECT_TRUE(found) << "the ROW exists; only its files are missing";
  EXPECT_TRUE(files.empty());

  // A mesh this version does not carry: readable answer, row absent.
  found = false;
  files.clear();
  ASSERT_TRUE(cognitive3d_ros::doctor::MeshFilesIn(kObjectsWithFiles, "turtlebot4", &found,
                                                   &files));
  EXPECT_FALSE(found);

  // Unreadable is FALSE, and must not be reported as a proven absence. Each of these is a
  // schema-invalid answer that must not be accepted as a conclusive listing, or a transient
  // backend shape becomes a FATAL "no files" or a FATAL "not on this version".
  for (const char* body : {R"({"errors":["nope"]})",
                           R"([{"meshName":"robot_placeholder_v3","files":"temporarily omitted"}])",
                           R"([{"error":"backend projection failed"}])",
                           R"([{"meshName":42,"files":[]}])",
                           R"([{"meshName":"robot_placeholder_v3","files":[{"name":7}]}])",
                           R"(["robot_placeholder_v3"])"}) {
    found = false;
    files.clear();
    EXPECT_FALSE(
        cognitive3d_ros::doctor::MeshFilesIn(body, "robot_placeholder_v3", &found, &files))
        << body;
    EXPECT_FALSE(found) << body;
  }
}

TEST(RobotMeshFiles, ARowWithNoFilesIsFatalAndNamesTheCallThatIsMissing) {
  // The defect this check exists for: call 1 runs, call 2 answers 200 and stores nothing, and
  // every other check in this tool passes. `mesh/exists` passes too -- the row is really there.
  cognitive3d_ros::doctor::RobotMeshFilesFacts facts = AnsweredFilesFacts();
  facts.robot_mesh = "robot_placeholder";
  facts.body_is_an_object_list = true;
  facts.mesh_listed = true;
  facts.files.clear();

  const Check check = cognitive3d_ros::doctor::CheckRobotMeshFiles(facts);
  EXPECT_EQ(check.name, "mesh/files");
  EXPECT_EQ(check.severity, Severity::kFatal);
  EXPECT_TRUE(Mentions(check, "NO FILES"));
  // The remedy has to name the mesh-name path segment, which is the cause in practice.
  EXPECT_TRUE(Mentions(check, "MESH NAME"));
  EXPECT_TRUE(Mentions(check, "/v0/objects/<scene_id>/robot_placeholder?version=1"));
}

TEST(RobotMeshFiles, FilesPresentPassesAndSaysWhichVersionItProved) {
  cognitive3d_ros::doctor::RobotMeshFilesFacts facts = AnsweredFilesFacts();
  facts.body_is_an_object_list = true;
  facts.mesh_listed = true;
  facts.files = {"robot_placeholder_v3.gltf", "robot_placeholder_v3.bin"};

  const Check check = cognitive3d_ros::doctor::CheckRobotMeshFiles(facts);
  EXPECT_EQ(check.name, "mesh/files");
  EXPECT_EQ(check.severity, Severity::kOk);
  EXPECT_TRUE(Mentions(check, "scene version 1"));
  EXPECT_TRUE(Mentions(check, "robot_placeholder_v3.bin"));
}

TEST(RobotMeshFiles, FilesWithoutTheNamedGltfIsFatalBecauseNothingResolves) {
  // Counting files cannot stand in for the one name that resolves: a row carrying only its buffer
  // would pass while the replay still has no robot, because the viewer addresses
  // `objects/{mesh}/{mesh}.gltf` by name.
  cognitive3d_ros::doctor::RobotMeshFilesFacts facts = AnsweredFilesFacts();
  facts.body_is_an_object_list = true;
  facts.mesh_listed = true;
  facts.files = {"robot_placeholder_v3.bin"};

  const Check only_buffer = cognitive3d_ros::doctor::CheckRobotMeshFiles(facts);
  EXPECT_EQ(only_buffer.name, "mesh/files");
  EXPECT_EQ(only_buffer.severity, Severity::kFatal);
  EXPECT_TRUE(Mentions(only_buffer, "robot_placeholder_v3.gltf"));

  // A glTF under any other basename is the same defect wearing a plausible file list.
  facts.files = {"scene.gltf", "scene.bin", "cvr_object_thumbnail.png"};
  EXPECT_EQ(cognitive3d_ros::doctor::CheckRobotMeshFiles(facts).severity, Severity::kFatal);
}

TEST(RobotMeshFiles, AVersionTheSceneDoesNotHaveIsLeftToSceneProject) {
  // scene/project reads the same scene document and fails on exactly this, naming the version to
  // use. Here it is only why there is no version to look the mesh up on, so it is not a second
  // FAIL for one fact.
  cognitive3d_ros::doctor::RobotMeshFilesFacts facts = AnsweredFilesFacts();
  facts.scene_version = "2";
  facts.version_id.clear();

  const Check check = cognitive3d_ros::doctor::CheckRobotMeshFiles(facts);
  EXPECT_EQ(check.name, "mesh/files (NOT CHECKED)");
  EXPECT_EQ(check.severity, Severity::kAdvisory);
  EXPECT_TRUE(Mentions(check, "no version numbered 2"));
  EXPECT_TRUE(Mentions(check, "scene/project"));

  // When scene/project could not read the document as a scene, it reports nothing about the
  // version, so the missing version is this check's to fail on.
  facts.scene_read_by_scene_project = false;
  const Check alone = cognitive3d_ros::doctor::CheckRobotMeshFiles(facts);
  EXPECT_EQ(alone.name, "mesh/files");
  EXPECT_EQ(alone.severity, Severity::kFatal);
  EXPECT_TRUE(Mentions(alone, "no version numbered 2"));
  EXPECT_FALSE(Mentions(alone, "scene/project"));

  // The same empty version id with an UNREADABLE body says so instead.
  facts.scene_body_is_readable = false;
  const Check unreadable = cognitive3d_ros::doctor::CheckRobotMeshFiles(facts);
  EXPECT_EQ(unreadable.severity, Severity::kAdvisory);
  EXPECT_EQ(unreadable.name, "mesh/files (NOT CHECKED)");
  EXPECT_TRUE(Mentions(unreadable, "could not be read"));
}

TEST(RobotMeshFiles, AnAbsentRowOnThisVersionIsFatalEvenWhenTheProjectHasTheMesh) {
  // The version-scoped answer the project-scoped check cannot give: a mesh on some OTHER version
  // of the scene passes `mesh/exists` while the version this run writes to has no robot in it.
  cognitive3d_ros::doctor::RobotMeshFilesFacts facts = AnsweredFilesFacts();
  facts.body_is_an_object_list = true;
  facts.mesh_listed = false;

  const Check check = cognitive3d_ros::doctor::CheckRobotMeshFiles(facts);
  EXPECT_EQ(check.severity, Severity::kFatal);
  EXPECT_TRUE(Mentions(check, "version id 70001"));
}

TEST(RobotMeshFiles, EveryUnansweredShapeIsNamedAndNotSilentlyOk) {
  // Same contract as the other probes: NOT APPLICABLE is kOk, UNVERIFIED is advisory, and neither
  // may be readable as "verified" by an exit code that cannot see the name.
  cognitive3d_ros::doctor::RobotMeshFilesFacts base = AnsweredFilesFacts();
  base.body_is_an_object_list = true;
  base.mesh_listed = true;
  base.files = {"robot_placeholder_v3.gltf"};

  cognitive3d_ros::doctor::RobotMeshFilesFacts no_mesh = base;
  no_mesh.robot_mesh.clear();
  const Check nothing_to_do = cognitive3d_ros::doctor::CheckRobotMeshFiles(no_mesh);
  EXPECT_EQ(nothing_to_do.severity, Severity::kOk);
  EXPECT_EQ(nothing_to_do.name, "mesh/files (NOT CHECKED)");

  cognitive3d_ros::doctor::RobotMeshFilesFacts no_scene = base;
  no_scene.scene_id.clear();
  cognitive3d_ros::doctor::RobotMeshFilesFacts no_key = base;
  no_key.org_key_present = false;
  cognitive3d_ros::doctor::RobotMeshFilesFacts bad_host = base;
  bad_host.destination_permitted = false;
  cognitive3d_ros::doctor::RobotMeshFilesFacts scene_failed = base;
  scene_failed.scene_status = 503;
  cognitive3d_ros::doctor::RobotMeshFilesFacts unresolved = base;
  unresolved.version_id.clear();
  unresolved.scene_body_is_readable = false;
  cognitive3d_ros::doctor::RobotMeshFilesFacts unreadable = base;
  unreadable.body_is_an_object_list = false;
  // The two halves of the object-listing guard, isolated. Without this case the suite passes
  // with the status check DELETED: a 503 carrying a parseable body would be read as a
  // successful listing.
  cognitive3d_ros::doctor::RobotMeshFilesFacts listing_failed = base;
  listing_failed.status = 503;
  listing_failed.body_is_an_object_list = true;
  cognitive3d_ros::doctor::RobotMeshFilesFacts scene_unreadable = base;
  scene_unreadable.scene_body_is_readable = false;

  for (const auto& facts : {no_scene, no_key, bad_host, scene_failed, unresolved, unreadable,
                            listing_failed, scene_unreadable}) {
    const Check check = cognitive3d_ros::doctor::CheckRobotMeshFiles(facts);
    EXPECT_EQ(check.severity, Severity::kAdvisory);
    EXPECT_EQ(check.name, "mesh/files (NOT CHECKED)");
  }
}

TEST(RobotMesh, TheFatalRemedyLeavesTheOtherProjectCauseToSceneProject) {
  // A project-scoped absence has two causes. The second -- a scene owned by a different project --
  // is scene/project's to report, from the scene itself, so this names that check rather than
  // saying it a second time.
  cognitive3d_ros::doctor::RobotMeshFacts facts;
  facts.robot_mesh = "robot_placeholder";
  facts.project = "90001";
  facts.org_key_present = true;
  facts.destination_permitted = true;
  facts.status = 200;
  facts.body_is_an_object_list = true;
  facts.available = {"turtlebot4"};
  facts.mesh_listed = false;

  const Check check = CheckRobotMesh(facts);
  ASSERT_EQ(check.severity, Severity::kFatal);
  EXPECT_TRUE(Mentions(check, "scene/project above says whether that is the cause"));
  EXPECT_FALSE(Mentions(check, "/v0/scenes/"));
  EXPECT_FALSE(Mentions(check, "projectId"));
}

TEST(RobotMesh, APresentMeshPasses) {
  cognitive3d_ros::doctor::RobotMeshFacts facts;
  facts.robot_mesh = "turtlebot4";
  facts.project = "90005";
  facts.org_key_present = true;
  facts.destination_permitted = true;
  facts.status = 200;
  facts.body_is_an_object_list = true;
  facts.available = {"turtlebot4"};
  facts.mesh_listed = true;
  const Check check = CheckRobotMesh(facts);
  EXPECT_EQ(check.severity, Severity::kOk);
  EXPECT_EQ(check.name, "mesh/exists");
}

TEST(RobotMesh, NotApplicableIsOkButUnverifiedIsAdvisory) {
  // A 401 that is advisory beside a MISSING key that is kOk makes "rejected" warn and "never
  // asked" pass. A severity-only consumer -- an exit code, a CI gate -- cannot read the name, so
  // kOk for a check that could not run reads as "verified". Only NOT APPLICABLE is kOk.
  cognitive3d_ros::doctor::RobotMeshFacts base;
  base.robot_mesh = "turtlebot4";
  base.project = "90005";
  base.org_key_present = true;
  base.destination_permitted = true;

  cognitive3d_ros::doctor::RobotMeshFacts no_mesh = base;
  no_mesh.robot_mesh.clear();
  const Check nothing_to_do = CheckRobotMesh(no_mesh);
  EXPECT_EQ(nothing_to_do.severity, Severity::kOk);
  EXPECT_EQ(nothing_to_do.name, "mesh/exists (NOT CHECKED)");

  cognitive3d_ros::doctor::RobotMeshFacts no_project = base;
  no_project.project.clear();
  cognitive3d_ros::doctor::RobotMeshFacts no_key = base;
  no_key.org_key_present = false;
  cognitive3d_ros::doctor::RobotMeshFacts bad_host = base;
  bad_host.destination_permitted = false;

  for (const auto& facts : {no_project, no_key, bad_host}) {
    const Check check = CheckRobotMesh(facts);
    EXPECT_EQ(check.severity, Severity::kAdvisory);
    EXPECT_EQ(check.name, "mesh/exists (NOT CHECKED)");
    // Anything not kOk names the action -- the Check contract.
    EXPECT_FALSE(check.remedy.empty());
  }
}

TEST(RobotMesh, AnUnreadableAnswerIsAdvisoryNotFatal) {
  // "I could not tell" and "it is not there" are different answers, and only one of them should
  // stop a run. Filing the first as the second sends an operator to support over a fault that
  // is not there.
  cognitive3d_ros::doctor::RobotMeshFacts facts;
  facts.robot_mesh = "turtlebot4";
  facts.project = "90005";
  facts.org_key_present = true;
  facts.destination_permitted = true;

  facts.status = 0;
  facts.transport_error = "timeout";
  EXPECT_EQ(CheckRobotMesh(facts).severity, Severity::kAdvisory);

  facts.status = 401;
  EXPECT_EQ(CheckRobotMesh(facts).severity, Severity::kAdvisory);

  facts.status = 200;
  facts.body_is_an_object_list = false;
  EXPECT_EQ(CheckRobotMesh(facts).severity, Severity::kAdvisory);
}

// ---------------------------------------------------------------------------------------------
// The pose channel: what the transform CONTAINS, not whether it resolves.

TEST(TiltDegrees, EveryPureYawIsZeroTiltAndNothingElseIs) {
  EXPECT_NEAR(TiltDegrees(0.0, 0.0, 0.0, 1.0), 0.0, 1e-9) << "identity";
  // A yaw of 90 degrees, which is all a differential-drive base on a flat floor ever produces.
  EXPECT_NEAR(TiltDegrees(0.0, 0.0, 0.70710678, 0.70710678), 0.0, 1e-6);
  // ...and of 180, where w is 0 and a naive test on w alone would report a tilt.
  EXPECT_NEAR(TiltDegrees(0.0, 0.0, 1.0, 0.0), 0.0, 1e-9);

  // A pitch of 90 degrees about +Y lays the robot's own +Z onto the horizon.
  EXPECT_NEAR(TiltDegrees(0.0, 0.70710678, 0.0, 0.70710678), 90.0, 1e-5);
  // Upside down.
  EXPECT_NEAR(TiltDegrees(1.0, 0.0, 0.0, 0.0), 180.0, 1e-9);
  // The ramp this check exists for: a -15.22 degree pitch reads back as 15.22.
  EXPECT_NEAR(TiltDegrees(0.0, -0.1324, 0.0, 0.99119), 15.22, 0.01);
}

// The predicate the pose-channel count actually depends on. It is an exact comparison with no
// arithmetic to contract, precisely because deriving it from TiltDegrees would make it depend on
// whether the compiler fuses the multiply-adds in that expression.
TEST(CarriesTilt, EveryPureYawIsFalseAndAnythingOffAxisIsTrue) {
  EXPECT_FALSE(CarriesTilt(0.0, 0.0)) << "identity, and every pure yaw, whatever qz and qw are";
  // The smallest thing a genuine 3D source shows standing still. TiltDegrees renders this as
  // roughly 7e-5 degrees, which any sane magnitude threshold would swallow.
  EXPECT_TRUE(CarriesTilt(6.1e-07, 0.0));
  EXPECT_TRUE(CarriesTilt(0.0, 6.1e-07));
  EXPECT_TRUE(CarriesTilt(0.0, -0.1324)) << "the ramp this check exists for";
  EXPECT_TRUE(CarriesTilt(-0.0, 0.0) == false) << "negative zero compares equal to zero";
}

TEST(TiltDegrees, ANonUnitQuaternionIsNormalisedRatherThanReportingATiltItDoesNotHave) {
  // A scaled pure YAW proves nothing on its own: it reads zero whether or not the code divides by
  // the norm. The discriminating case is a scaled TILT -- without normalisation this expression
  // runs past -1, clamps, and reports 180 degrees for a 15 degree pitch.
  EXPECT_NEAR(TiltDegrees(0.0, -1.324, 0.0, 9.9119), 15.22, 0.01);
  EXPECT_NEAR(TiltDegrees(0.0, 0.0, 7.0710678, 7.0710678), 0.0, 1e-6);
  EXPECT_NEAR(TiltDegrees(0.0, 0.0, 0.0, 0.0), 0.0, 1e-9) << "a zero quaternion must not divide";
}

namespace {
cognitive3d_ros::doctor::PoseChannelFacts Flat(const std::string& frame) {
  cognitive3d_ros::doctor::PoseChannelFacts facts;
  facts.frame_in_force = frame;
  facts.samples = 1803;
  facts.height_span_meters = 0.0;
  facts.max_tilt_degrees = 0.0;
  facts.samples_with_tilt = 0;
  return facts;
}
}  // namespace

// What this check exists to explain: 1803 samples, one distinct height, every rotation a pure
// yaw, on a robot that climbs. Both frames the SDK defaults to produce it.
TEST(PoseChannel, AFlatReadingOnAFrameWhoseUsualProducerIsPlanarIsAdvisory) {
  for (const std::string& frame : {"odom", "map", "robot1/odom", "tb4/map"}) {
    const Check check = CheckPoseChannel(Flat(frame));
    EXPECT_EQ(check.severity, Severity::kAdvisory) << frame;
    EXPECT_EQ(check.name, "pose channel") << frame << " must not read as NOT CHECKED";
    EXPECT_FALSE(check.remedy.empty()) << frame;
    EXPECT_NE(check.remedy.find("wheel encoders"), std::string::npos)
        << "the remedy has to name the mechanism, or it reads as a transient";
    EXPECT_NE(check.remedy.find("nav2_amcl"), std::string::npos) << frame;
  }
}

// Observation can only ever prove the channel IS three dimensional. It never proves the negative,
// which is why the flat verdict above is structural.
TEST(PoseChannel, ObservedHeightOrTiltPromotesEvenAFrameKnownToBePlanar) {
  cognitive3d_ros::doctor::PoseChannelFacts climbed = Flat("odom");
  climbed.height_span_meters = 0.141;
  EXPECT_EQ(CheckPoseChannel(climbed).severity, Severity::kOk);

  cognitive3d_ros::doctor::PoseChannelFacts tilted = Flat("map");
  tilted.samples_with_tilt = 1803;
  tilted.max_tilt_degrees = 15.22;
  EXPECT_EQ(CheckPoseChannel(tilted).severity, Severity::kOk);
}

// The exact-zero discriminator fires at about 1e-6 degrees, which the message beside it rounds
// to `0.00`. Reported as "carries a third dimension: 0.000 m of height ... (worst 0.00 degrees)"
// it is a sentence that refutes itself, and a reader disbelieves the check -- which is worse than
// not having it. A robot parked on a ground-truth publisher produces exactly these facts.
TEST(PoseChannel, ANonPlanarProducerWithNothingToShowSaysWhichOfTheTwoItProved) {
  cognitive3d_ros::doctor::PoseChannelFacts parked = Flat("map");
  parked.samples = 22;
  parked.samples_with_tilt = 22;      // the discriminator fired
  parked.max_tilt_degrees = 1.0e-6;   // and rounds to 0.00 in any printed form
  parked.height_span_meters = 0.0;

  const Check check = CheckPoseChannel(parked);
  EXPECT_EQ(check.severity, Severity::kOk);
  EXPECT_EQ(check.name, "pose channel");
  EXPECT_TRUE(Mentions(check, "is NOT planar"));
  EXPECT_TRUE(Mentions(check, "22 of 22"));
  EXPECT_TRUE(Mentions(check, "below 0.01 degrees"));
  // A tilt count establishes the ORIENTATION is not planar. It says nothing about height, so the
  // remedy must not claim both.
  EXPECT_TRUE(Mentions(check, "nothing about HEIGHT"));
  EXPECT_FALSE(Mentions(check, "CAN carry height and tilt"));
  // The self-refuting claim must not appear when neither magnitude is visible.
  EXPECT_FALSE(Mentions(check, "carries a third dimension"))
      << "claiming a third dimension beside two printed zeros is the defect";

  // One visible magnitude and it IS the observed-3D verdict again, with the numbers to back it.
  cognitive3d_ros::doctor::PoseChannelFacts climbed = parked;
  climbed.max_tilt_degrees = 10.22;
  const Check observed = CheckPoseChannel(climbed);
  EXPECT_EQ(observed.severity, Severity::kOk);
  EXPECT_TRUE(Mentions(observed, "carries a third dimension"));
  EXPECT_TRUE(Mentions(observed, "10.22 degrees"));
}

// The wording split must not narrow WHAT PROMOTES. A split on the rendering threshold that did
// would let a sub-millimetre height span with no tilt fall through to the planar-by-construction
// advisory -- reporting a producer as planar while holding measured evidence that it is not. A
// wheel-odometry source emits EXACTLY zero, never 0.0004.
TEST(PoseChannel, SubMillimetreHeightStillRulesOutAPlanarProducer) {
  cognitive3d_ros::doctor::PoseChannelFacts crept = Flat("odom");
  crept.samples = 22;
  crept.samples_with_tilt = 0;
  crept.height_span_meters = 0.0004;  // renders as 0.000 m

  const Check check = CheckPoseChannel(crept);
  EXPECT_EQ(check.name, "pose channel");
  EXPECT_EQ(check.severity, Severity::kOk) << "a non-zero height span is the discriminator too";
  EXPECT_TRUE(Mentions(check, "is NOT planar"));
  EXPECT_TRUE(Mentions(check, "the height is not constant"));
  EXPECT_FALSE(Mentions(check, "wheel encoders"))
      << "it must not fall through to the planar-by-construction advisory";

  // The same facts on a frame with no known producer must reach the same verdict, not
  // `(NOT CHECKED)` -- the evidence is in the numbers, not in the frame's name.
  cognitive3d_ros::doctor::PoseChannelFacts custom = crept;
  custom.frame_in_force = "ground_truth";
  EXPECT_EQ(CheckPoseChannel(custom).name, "pose channel");
  EXPECT_TRUE(Mentions(CheckPoseChannel(custom), "is NOT planar"));
}

// Both thresholds pinned AT the boundary, not near it. Pinned loosely, moving 0.0005 to 0.001
// would leave the suite green while the boundary case misreported.
TEST(PoseChannel, TheVisibilityThresholdsAreTheRenderingThresholds) {
  cognitive3d_ros::doctor::PoseChannelFacts base = Flat("map");
  base.samples = 22;
  base.samples_with_tilt = 22;
  base.max_tilt_degrees = 1.0e-6;

  cognitive3d_ros::doctor::PoseChannelFacts at_height = base;
  at_height.height_span_meters = 0.0005;  // the first value that renders as 0.001
  EXPECT_TRUE(Mentions(CheckPoseChannel(at_height), "carries a third dimension"));

  cognitive3d_ros::doctor::PoseChannelFacts under_height = base;
  under_height.height_span_meters = 0.0004;
  EXPECT_FALSE(Mentions(CheckPoseChannel(under_height), "carries a third dimension"));

  cognitive3d_ros::doctor::PoseChannelFacts at_tilt = base;
  at_tilt.max_tilt_degrees = 0.005;  // the first value that renders as 0.01
  EXPECT_TRUE(Mentions(CheckPoseChannel(at_tilt), "carries a third dimension"));

  cognitive3d_ros::doctor::PoseChannelFacts under_tilt = base;
  under_tilt.max_tilt_degrees = 0.004;
  EXPECT_FALSE(Mentions(CheckPoseChannel(under_tilt), "carries a third dimension"));
}

// A custom frame that happened to read flat has established nothing -- a parked robot on a
// genuine 3D source reads identically -- so it declines rather than warning.
TEST(PoseChannel, AnUnrecognisedFrameReadingFlatDeclinesRatherThanAccusing) {
  const Check check = CheckPoseChannel(Flat("mocap_world"));
  EXPECT_EQ(check.severity, Severity::kOk);
  EXPECT_NE(check.name.find("NOT CHECKED"), std::string::npos);
}

TEST(PoseChannel, NothingSampledSaysNothing) {
  cognitive3d_ros::doctor::PoseChannelFacts nothing = Flat("odom");
  nothing.samples = 0;
  EXPECT_NE(CheckPoseChannel(nothing).name.find("NOT CHECKED"), std::string::npos);

  cognitive3d_ros::doctor::PoseChannelFacts no_frame = Flat("");
  EXPECT_NE(CheckPoseChannel(no_frame).name.find("NOT CHECKED"), std::string::npos);
}

// The discriminator is EXACT zero, not a magnitude, and that is what makes the check work on a
// flat floor: a planar producer reads 0 of N however long it runs, while a source that genuinely
// carries tilt trips the counter on its first sample even parked. A threshold on the magnitude
// would classify a parked 3D source as planar and accuse it.
TEST(PoseChannel, AnySampleCarryingAnyTiltAtAllIsEnoughToPromote) {
  cognitive3d_ros::doctor::PoseChannelFacts parked_3d = Flat("odom");
  parked_3d.samples_with_tilt = 41;
  parked_3d.max_tilt_degrees = 0.00007;  // the jitter a real 3D source shows standing still
  EXPECT_EQ(CheckPoseChannel(parked_3d).severity, Severity::kOk)
      << "a magnitude threshold here would accuse a robot_localization stack in 3D mode";

  cognitive3d_ros::doctor::PoseChannelFacts one_sample = Flat("map");
  one_sample.samples_with_tilt = 1;
  EXPECT_EQ(CheckPoseChannel(one_sample).severity, Severity::kOk);

  cognitive3d_ros::doctor::PoseChannelFacts planar = Flat("odom");
  planar.samples_with_tilt = 0;
  EXPECT_EQ(CheckPoseChannel(planar).severity, Severity::kAdvisory);
}

// The advisory must not assert an impossibility it cannot know from a frame NAME: `odom` is also
// what robot_localization publishes in 3D mode, and `map` what a 6-DOF SLAM publishes.
TEST(PoseChannel, TheAdvisoryNamesTheProducerRatherThanDeclaringTheFrameIncapable) {
  const Check check = CheckPoseChannel(Flat("odom"));
  EXPECT_EQ(check.severity, Severity::kAdvisory);
  EXPECT_EQ(check.detail.find("cannot carry"), std::string::npos)
      << "a frame name is a hint about the producer, never a proof: " << check.detail;
  EXPECT_NE(check.remedy.find("robot_localization"), std::string::npos)
      << "the remedy has to tell a 3D-stack reader that this note is not about them";
}

// The count is reported, not just the magnitude: "0 of 41 carried any tilt" and "0.00 degrees"
// are different statements, and only the first distinguishes a planar source from a still one.
TEST(PoseChannel, TheDetailReportsHowManySamplesCarriedAnyTiltAtAll) {
  const Check check = CheckPoseChannel(Flat("odom"));
  EXPECT_NE(check.detail.find("1803 samples"), std::string::npos) << check.detail;
  EXPECT_NE(check.detail.find("0 carried any tilt"), std::string::npos) << check.detail;
}


// ---------------------------------------------------------------------------------------------
// The object id: a cross-system contract, checked separately from the mesh because a mismatch
// fails differently -- and much less obviously.

namespace {
cognitive3d_ros::doctor::RobotObjectIdFacts RegisteredObject(bool listed) {
  cognitive3d_ros::doctor::RobotObjectIdFacts facts;
  facts.robot_object_id = "robot";
  facts.project = "90005";
  facts.org_key_present = true;
  facts.destination_permitted = true;
  facts.status = 200;
  facts.body_is_an_object_list = true;
  facts.id_listed = listed;
  facts.available = {"a5edce9e-0000-0000-0000-000000000000", "robot"};
  return facts;
}
}  // namespace

// The distinction from the mesh check, and the reason both exist. A missing MESH empties the
// replay, so it is fatal. A missing ID does not -- the viewer fabricates a registry entry from
// the runtime manifest and renders anyway -- so it is a warning, and the remedy has to say that,
// or the reader diagnoses a missing mesh and looks in the wrong place.
TEST(RobotObjectId, AnUnregisteredIdIsAdvisoryAndSaysTheRobotWillStillRender) {
  const Check check = CheckRobotObjectId(RegisteredObject(false));
  EXPECT_EQ(check.severity, Severity::kAdvisory)
      << "fatal would be wrong: the mesh still renders, which is exactly what makes it confusing";
  EXPECT_NE(check.remedy.find("will still show the robot"), std::string::npos);
  EXPECT_NE(check.detail.find("a5edce9e"), std::string::npos)
      << "a check that says 'not found' without saying what IS there starts a support thread";
}

TEST(RobotObjectId, ARegisteredIdPassesAndDeclaresItsScope) {
  const Check check = CheckRobotObjectId(RegisteredObject(true));
  EXPECT_EQ(check.severity, Severity::kOk);
  EXPECT_NE(check.detail.find("project scope"), std::string::npos)
      << "the project route cannot confirm the scene version, and must not imply it did";
}

TEST(RobotObjectId, EveryDeclinePathIsLabelledAndNotApplicableIsTheOnlyOkOne) {
  cognitive3d_ros::doctor::RobotObjectIdFacts unset = RegisteredObject(true);
  unset.robot_object_id = "";
  const Check nothing_to_do = CheckRobotObjectId(unset);
  EXPECT_EQ(nothing_to_do.severity, Severity::kOk);
  EXPECT_NE(nothing_to_do.name.find("NOT CHECKED"), std::string::npos);

  for (int which = 0; which < 3; ++which) {
    cognitive3d_ros::doctor::RobotObjectIdFacts facts = RegisteredObject(true);
    if (which == 0) facts.project = "";
    if (which == 1) facts.org_key_present = false;
    if (which == 2) facts.destination_permitted = false;
    const Check check = CheckRobotObjectId(facts);
    EXPECT_EQ(check.severity, Severity::kAdvisory) << which;
    EXPECT_NE(check.name.find("NOT CHECKED"), std::string::npos) << which;
  }
}

TEST(RobotObjectId, AnUnreadableAnswerIsNeverReportedAsAnAbsence) {
  cognitive3d_ros::doctor::RobotObjectIdFacts facts = RegisteredObject(false);
  facts.status = 0;
  facts.transport_error = "timeout";
  Check check = CheckRobotObjectId(facts);
  EXPECT_EQ(check.severity, Severity::kAdvisory);
  EXPECT_EQ(check.detail.find("is NOT a registered"), std::string::npos);

  facts.status = 200;
  facts.body_is_an_object_list = false;
  check = CheckRobotObjectId(facts);
  EXPECT_EQ(check.detail.find("is NOT a registered"), std::string::npos);
}

// THE SWITCH THE THREE CHECKS ABOVE HANG ON, tested against facts that are all FATAL, because a
// gate tested against facts that pass says nothing: every arrangement of it would pass too.
TEST(RobotObjectChecks, WithNoDynamicObjectNothingAboutAMeshCanFailTheRun) {
  cognitive3d_ros::doctor::RobotMeshFacts mesh;
  mesh.robot_mesh = "robot_placeholder";
  mesh.project = "90001";
  mesh.org_key_present = true;
  mesh.destination_permitted = true;
  mesh.status = 200;
  mesh.body_is_an_object_list = true;
  mesh.mesh_listed = false;            // fatal: the mesh is not there
  cognitive3d_ros::doctor::RobotObjectIdFacts id = RegisteredObject(false);
  cognitive3d_ros::doctor::RobotMeshFilesFacts files;
  files.robot_mesh = "robot_placeholder";
  files.scene_id = "S";
  files.scene_version = "3";
  files.org_key_present = true;
  files.destination_permitted = true;
  files.scene_status = 200;
  files.scene_body_is_readable = true;
  files.version_id = "70001";
  files.status = 200;
  files.body_is_an_object_list = true;
  files.mesh_listed = true;            // fatal: registered with no files
  ASSERT_EQ(Aggregate(RobotObjectChecks(true, mesh, id, files)), Severity::kFatal)
      << "the facts have to be ones that DO fail, or the off case proves nothing";

  const std::vector<Check> off = RobotObjectChecks(false, mesh, id, files);
  ASSERT_EQ(off.size(), 1u) << "one line stands in for the three, rather than three quiet passes";
  EXPECT_EQ(Aggregate(off), Severity::kOk);
  // A DECLINE, not a verdict: nothing was verified, and the name is the only place that can say
  // so -- a severity-only reader, an exit code or a CI gate, sees kOk either way.
  EXPECT_NE(off[0].name.find("NOT CHECKED"), std::string::npos);
  EXPECT_TRUE(Mentions(off[0], "pose timeline"))
      << "the reader has to be told what draws the robot instead: " << off[0].detail;
  EXPECT_TRUE(Mentions(off[0], "mesh/exists")) << "and which checks stopped running";
}

TEST(RobotObjectChecks, WithADynamicObjectTheThreeChecksAreTheOnesThatRun) {
  const std::vector<Check> on = RobotObjectChecks(true, cognitive3d_ros::doctor::RobotMeshFacts{},
                                                 RegisteredObject(true),
                                                 cognitive3d_ros::doctor::RobotMeshFilesFacts{});
  ASSERT_EQ(on.size(), 3u);
  EXPECT_NE(on[0].name.find("mesh/exists"), std::string::npos);
  EXPECT_NE(on[1].name.find("object id/registered"), std::string::npos);
  EXPECT_NE(on[2].name.find("mesh/files"), std::string::npos);
}

TEST(StringField, ReadsAnyNamedFieldNotJustMeshName) {
  std::vector<std::string> ids;
  ASSERT_TRUE(StringFieldIn(R"([{"sdkId":"robot","meshName":"turtlebot4"},{"sdkId":"forklift"}])",
                            "sdkId", &ids));
  ASSERT_EQ(ids.size(), 2u);
  EXPECT_EQ(ids[0], "robot");
  EXPECT_EQ(ids[1], "forklift");

  std::vector<std::string> absent;
  EXPECT_TRUE(StringFieldIn(R"([{"sdkId":"robot"}])", "meshName", &absent));
  EXPECT_TRUE(absent.empty()) << "a field no row carries is an empty list, not an unreadable body";
}

TEST(ActionEnrolment, NoActionServersIsNotAProblem) {
  ActionEnrolmentFacts facts;
  const Check check = CheckActionEnrolment(facts);
  EXPECT_EQ(check.severity, Severity::kOk);
  EXPECT_TRUE(Mentions(check, "no action servers"));
}

TEST(ActionEnrolment, AnUnenrolledActionIsAdvisoryAndSaysItWillEmitNothing) {
  // The whole point of the check. A session with no objectives for an action is
  // indistinguishable from a robot that never ran it, and this is the only moment
  // the difference is visible.
  ActionEnrolmentFacts facts;
  facts.actions.push_back(cognitive3d_ros::ClassifyActionTopic(
      "/navigate_to_pose/_action/status", false, false, true,
      cognitive3d_ros::DefaultActionNames()));
  facts.actions.push_back(cognitive3d_ros::ClassifyActionTopic(
      "/arm/pick_object/_action/status", false, false, true,
      cognitive3d_ros::DefaultActionNames()));
  const Check check = CheckActionEnrolment(facts);
  EXPECT_EQ(check.severity, Severity::kAdvisory);
  EXPECT_TRUE(Mentions(check, "/arm/pick_object/_action/status"));
  EXPECT_TRUE(Mentions(check, "no events"));
  EXPECT_TRUE(Mentions(check, "action_names"));
  EXPECT_FALSE(check.remedy.empty());
}

// Nav2's planner, smoother and controller actions are left out of the built-in list on purpose, so
// they read as information: their own list, a remedy that starts with "nothing to do", and no
// capitals that read as an alarm.
TEST(ActionEnrolment, ActionsLeftOutByDesignReadAsInformation) {
  ActionEnrolmentFacts facts;
  for (const char* topic : {"/navigate_to_pose/_action/status", "/follow_path/_action/status",
                            "/compute_path_to_pose/_action/status"}) {
    facts.actions.push_back(cognitive3d_ros::ClassifyActionTopic(
        topic, false, false, true, cognitive3d_ros::DefaultActionNames()));
  }
  const Check check = CheckActionEnrolment(facts);
  EXPECT_EQ(check.severity, Severity::kAdvisory);
  EXPECT_TRUE(Mentions(check, "1 of 3 action server(s) will be recorded"));
  EXPECT_TRUE(Mentions(check, "left out by design: /follow_path/_action/status, "
                              "/compute_path_to_pose/_action/status"));
  EXPECT_FALSE(Mentions(check, "not recorded:")) << check.detail;
  EXPECT_EQ(check.remedy.rfind("Nothing to do", 0), 0u) << check.remedy;
  for (const char* alarm : {"NOT ", "NO ", "REPLACES"}) {
    EXPECT_FALSE(Mentions(check, alarm)) << alarm;
  }

  // Beside an action that is simply unlisted, each keeps its own list and the remedy leads with
  // the one there is something to do about.
  facts.actions.push_back(cognitive3d_ros::ClassifyActionTopic(
      "/arm/pick_object/_action/status", false, false, true,
      cognitive3d_ros::DefaultActionNames()));
  const Check both = CheckActionEnrolment(facts);
  EXPECT_TRUE(Mentions(both, "not recorded: /arm/pick_object/_action/status"));
  EXPECT_TRUE(Mentions(both, "left out by design: /follow_path/_action/status"));
  EXPECT_EQ(both.detail.find("/follow_path/_action/status ("), std::string::npos) << both.detail;
  EXPECT_EQ(both.remedy.rfind("An action that is not recorded emits no events", 0), 0u)
      << both.remedy;
}

TEST(ActionEnrolment, EveryActionEnrolledPassesAndNamesThem) {
  ActionEnrolmentFacts facts;
  facts.actions.push_back(cognitive3d_ros::ClassifyActionTopic(
      "/robot1/dock/_action/status", false, false, true,
      cognitive3d_ros::DefaultActionNames()));
  const Check check = CheckActionEnrolment(facts);
  EXPECT_EQ(check.severity, Severity::kOk);
  EXPECT_TRUE(Mentions(check, "/robot1/dock/_action/status"));
  EXPECT_TRUE(check.remedy.empty());
}

TEST(ActionEnrolment, TheTypesTakenWithoutAnAllowlistEntryAreReportedToo) {
  // They are the other half of what a run subscribes to without being told, and
  // an operator reading the allowlist has no other way to learn about them.
  ActionEnrolmentFacts facts;
  facts.standard_topics = {"/robot1/battery_state"};
  const Check check = CheckActionEnrolment(facts);
  EXPECT_EQ(check.severity, Severity::kOk);
  EXPECT_TRUE(Mentions(check, "/robot1/battery_state"));
}

TEST(ActionEnrolment, TheStandardTypeSwitchTurnedOffIsSaidRatherThanReadingAsNothingPresent) {
  // Off, the list is empty for a reason no operator can see from the line: the
  // battery and diagnostics topics are there, and nothing but a `topics` entry
  // will now reach them. An empty list and a disabled switch must not print the
  // same thing.
  ActionEnrolmentFacts facts;
  facts.standard_types_enabled = false;
  const Check check = CheckActionEnrolment(facts);
  EXPECT_EQ(check.severity, Severity::kOk);
  EXPECT_TRUE(Mentions(check, "auto_subscribe_standard_types"));

  ActionEnrolmentFacts on;
  on.standard_types_enabled = true;
  EXPECT_FALSE(Mentions(CheckActionEnrolment(on), "auto_subscribe_standard_types"));
}

TEST(ActionEnrolment, DiscoveryTurnedOffIsSaidRatherThanLookingLikeAnEmptyGraph) {
  ActionEnrolmentFacts facts;
  facts.discovery_enabled = false;
  const Check check = CheckActionEnrolment(facts);
  EXPECT_TRUE(Mentions(check, "action_discovery"));
}

// --- which build of the node the report is about ------------------------------------------------

TEST(Build, TheReportNamesTheBuildAndWhatItTranslates) {
  BuildFacts nav2;
  nav2.package = "cognitive3d_ros_nav2";
  nav2.constructed = true;
  nav2.translated_types = {"nav2_msgs/msg/BehaviorTreeLog", "nav_msgs/msg/Odometry"};
  const Check check = CheckBuild(nav2);
  EXPECT_EQ(check.name, "build");
  EXPECT_EQ(check.severity, Severity::kOk);
  EXPECT_EQ(check.detail,
            "cognitive3d_ros_nav2's node, the one the launch file runs while that package is "
            "installed; translating 2 message type(s): nav2_msgs/msg/BehaviorTreeLog, "
            "nav_msgs/msg/Odometry");

  BuildFacts plain;
  plain.package = "cognitive3d_ros";
  const Check unconstructed = CheckBuild(plain);
  EXPECT_EQ(unconstructed.severity, Severity::kOk);
  EXPECT_EQ(unconstructed.detail,
            "cognitive3d_ros's node, the one the launch file runs while cognitive3d_ros_nav2 is "
            "not installed")
      << "a node that was not constructed has no types to report";
}

// The fallback is reported beside the build it fell back to, because that build is not the one
// the launch file runs.
TEST(Build, AHandoffThatFailedIsANoteNamingWhy) {
  BuildFacts facts;
  facts.package = "cognitive3d_ros";
  facts.constructed = true;
  facts.translated_types = {"nav_msgs/msg/Odometry"};
  facts.handoff_error = "/opt/ros/jazzy/lib/cognitive3d_ros_nav2/doctor: No such file or directory";
  const Check check = CheckBuild(facts);
  EXPECT_EQ(check.severity, Severity::kAdvisory);
  EXPECT_TRUE(Mentions(check, "cognitive3d_ros's node; translating 1 message type(s)"));
  EXPECT_TRUE(Mentions(check, "that package's doctor did not start: /opt/ros/jazzy/lib/"
                              "cognitive3d_ros_nav2/doctor: No such file or directory"));
  EXPECT_FALSE(Mentions(check, "while cognitive3d_ros_nav2 is not installed"));
  EXPECT_TRUE(Mentions(check, "nav2_msgs/msg/BehaviorTreeLog"));
}

TEST(Build, AnIndexThatCannotBeReadIsANoteNamingWhy) {
  BuildFacts facts;
  facts.package = "cognitive3d_ros";
  facts.constructed = true;
  facts.translated_types = {"nav_msgs/msg/Odometry"};
  facts.index_error = "Environment variable 'AMENT_PREFIX_PATH' is not set or empty";
  const Check check = CheckBuild(facts);
  EXPECT_EQ(check.severity, Severity::kAdvisory);
  EXPECT_TRUE(Mentions(check, "cognitive3d_ros's node; translating 1 message type(s)"));
  EXPECT_TRUE(Mentions(check, "which build the launch file runs is unknown: Environment variable "
                              "'AMENT_PREFIX_PATH' is not set or empty"));
  EXPECT_FALSE(Mentions(check, "while cognitive3d_ros_nav2 is not installed"));
  EXPECT_TRUE(Mentions(check, "setup.bash"));
}

TEST(Build, TheLaunchedBuildIsNav2sExactlyWhenThatPackageIsInstalled) {
  std::vector<std::string> asked;
  const auto installed = [&asked](const std::string& package) {
    asked.push_back(package);
    return std::string("/opt/ros/jazzy");
  };
  EXPECT_EQ(cognitive3d_ros::LaunchedBuildDoctor(installed),
            "/opt/ros/jazzy/lib/cognitive3d_ros_nav2/doctor");
  EXPECT_EQ(asked, std::vector<std::string>{"cognitive3d_ros_nav2"});
  EXPECT_EQ(cognitive3d_ros::LaunchedBuildDoctor([](const std::string&) { return std::string(); }),
            "");
}

// --- the scene's project and version, which nothing at run time checks --------------------------

namespace {

// The shape of `GET /v0/scenes/<id>`, with synthetic ids: the version NUMBER and the numeric
// version id side by side, which is what makes one easy to configure in place of the other.
const char* const kTwoVersionScene = R"({
  "id":"a1b2c3d4-0000-4000-8000-000000000001","projectId":90003,"sceneName":"World",
  "versions":[{"id":70001,"versionNumber":1},{"id":70002,"versionNumber":2}]
})";

SceneProjectFacts AnsweredScene(const std::string& version) {
  SceneProjectFacts facts;
  facts.scene_id = "a1b2c3d4-0000-4000-8000-000000000001";
  facts.scene_version = version;
  facts.expected_project = "90003";
  facts.org_key_present = true;
  facts.destination_permitted = true;
  facts.status = 200;
  facts.body_is_a_scene = SceneDocumentIn(kTwoVersionScene, &facts.scene_project, &facts.versions);
  return facts;
}

}  // namespace

TEST(SceneDocument, ReadsTheProjectAndEveryVersionInEitherSpelling) {
  std::string project;
  std::vector<SceneVersion> versions;
  ASSERT_TRUE(SceneDocumentIn(kTwoVersionScene, &project, &versions));
  EXPECT_EQ(project, "90003");
  ASSERT_EQ(versions.size(), 2u);
  EXPECT_EQ(versions[1].number, "2");
  EXPECT_EQ(versions[1].id, "70002");

  ASSERT_TRUE(SceneDocumentIn(R"({"projectId":"90004","versions":[{"versionNumber":"1"}]})",
                              &project, &versions));
  EXPECT_EQ(project, "90004");
  ASSERT_EQ(versions.size(), 1u);
  EXPECT_EQ(versions[0].id, "") << "a version with no id is still a version";

  for (const char* body : {"not json", "[]", R"({"versions":[]})", R"({"projectId":90004})",
                           R"({"projectId":90004,"versions":"soon"})",
                           R"({"projectId":90004,"versions":[{"id":1}]})",
                           R"({"projectId":null,"versions":[]})"}) {
    EXPECT_FALSE(SceneDocumentIn(body, &project, &versions)) << body;
  }
}

TEST(SceneProject, TheScenesProjectAndVersionPass) {
  const Check check = CheckSceneProject(AnsweredScene("2"));
  EXPECT_EQ(check.name, "scene/project");
  EXPECT_EQ(check.severity, Severity::kOk);
  EXPECT_EQ(check.detail,
            "scene a1b2c3d4-0000-4000-8000-000000000001 has version 2 (version id 70002) and is "
            "in project 90003");
}

// scene_version is a string of digits, and configure reads it as a number.
TEST(SceneProject, TheVersionIsComparedAsANumber) {
  const Check check = CheckSceneProject(AnsweredScene("02"));
  EXPECT_EQ(check.severity, Severity::kOk) << check.detail;
  EXPECT_TRUE(Mentions(check, "(version id 70002)"));
  EXPECT_EQ(CheckSceneProject(AnsweredScene("070002")).severity, Severity::kFatal);
  EXPECT_TRUE(Mentions(CheckSceneProject(AnsweredScene("070002")), "is the version id of version 2"));
  EXPECT_EQ(CheckSceneProject(AnsweredScene("20")).severity, Severity::kFatal);
}

TEST(SceneProject, ASceneInAnotherProjectIsFatalAndNamesBothAndTheShadowScene) {
  SceneProjectFacts facts = AnsweredScene("1");
  facts.expected_project = "90004";
  const Check check = CheckSceneProject(facts);
  EXPECT_EQ(check.name, "scene/project");
  EXPECT_EQ(check.severity, Severity::kFatal);
  EXPECT_TRUE(Mentions(check, "belongs to project 90003, not to project 90004"));
  EXPECT_TRUE(Mentions(check, "empty shadow scene"));
  EXPECT_TRUE(Mentions(check, "the session records against it"));
  EXPECT_TRUE(Mentions(check, "or correct C3D_PROJECT_ID if it names the wrong project"));
}

TEST(SceneProject, TheVersionIdInPlaceOfTheNumberIsFatalAndNamesTheNumber) {
  const Check check = CheckSceneProject(AnsweredScene("70002"));
  EXPECT_EQ(check.severity, Severity::kFatal);
  EXPECT_TRUE(Mentions(check, "scene_version \"70002\" is the version id of version 2"));
  EXPECT_TRUE(Mentions(check, "Set scene_version: \"2\""));
}

TEST(SceneProject, AVersionTheSceneDoesNotHaveIsFatalAndListsTheOnesItHas) {
  const Check check = CheckSceneProject(AnsweredScene("3"));
  EXPECT_EQ(check.severity, Severity::kFatal);
  EXPECT_TRUE(Mentions(check, "has no version numbered 3; its versions are 1, 2"));
  EXPECT_FALSE(Mentions(check, "version id of"));
}

TEST(SceneProject, BothWrongAtOnceAreBothReported) {
  SceneProjectFacts facts = AnsweredScene("3");
  facts.expected_project = "90004";
  const Check check = CheckSceneProject(facts);
  EXPECT_EQ(check.severity, Severity::kFatal);
  EXPECT_TRUE(Mentions(check, "not to project 90004"));
  EXPECT_TRUE(Mentions(check, "has no version numbered 3"));
}

// The org key asks for the check; the project to compare with is a second, separate setting.
TEST(SceneProject, WithoutAnExpectedProjectTheVersionIsCheckedAndTheProjectIsANote) {
  SceneProjectFacts facts = AnsweredScene("1");
  facts.expected_project.clear();
  const Check check = CheckSceneProject(facts);
  EXPECT_EQ(check.name, "scene/project");
  EXPECT_EQ(check.severity, Severity::kAdvisory);
  EXPECT_TRUE(Mentions(check, "is in project 90003"));
  EXPECT_TRUE(Mentions(check, "Set C3D_PROJECT_ID"));

  facts.scene_version = "3";
  EXPECT_EQ(CheckSceneProject(facts).severity, Severity::kFatal);
}

TEST(SceneProject, WithoutAnOrganisationKeyItIsNotCheckedAndSaysHowToEnableIt) {
  SceneProjectFacts facts = AnsweredScene("3");
  facts.org_key_present = false;
  const Check check = CheckSceneProject(facts);
  EXPECT_EQ(check.name, "scene/project (NOT CHECKED)");
  EXPECT_EQ(check.severity, Severity::kOk);
  EXPECT_TRUE(Mentions(check, "set C3D_ORG_API_KEY"));
}

// Every way the lookup can fail to answer is a note, never a verdict about the scene, whether or
// not key/project proved the organisation key.
TEST(SceneProject, EveryUnansweredLookupIsANoteAndNamesWhatToDo) {
  SceneProjectFacts refused = AnsweredScene("3");
  refused.status = 401;
  refused.body_is_a_scene = false;
  SceneProjectFacts unreachable = AnsweredScene("3");
  unreachable.status = 0;
  unreachable.transport_error = "Could not resolve host";
  SceneProjectFacts unreadable = AnsweredScene("3");
  unreadable.body_is_a_scene = false;
  SceneProjectFacts not_https = AnsweredScene("3");
  not_https.destination_permitted = false;
  // A failed status beside a body that happens to parse is still not an answer.
  SceneProjectFacts failed = AnsweredScene("3");
  failed.status = 503;

  for (const bool verified : {false, true}) {
    for (SceneProjectFacts facts : {refused, unreachable, unreadable, not_https, failed}) {
      facts.org_key_verified = verified;
      const Check check = CheckSceneProject(facts);
      EXPECT_EQ(check.severity, Severity::kAdvisory) << check.detail;
      EXPECT_FALSE(check.remedy.empty()) << check.detail;
    }
  }
  EXPECT_TRUE(Mentions(CheckSceneProject(refused), "C3D_ORG_API_KEY"));
  EXPECT_TRUE(Mentions(CheckSceneProject(unreachable), "Could not resolve host"));
  EXPECT_EQ(CheckSceneProject(not_https).name, "scene/project (NOT CHECKED)");
}

// The API answers 403 for a scene id it does not show the organisation key. Once key/project has
// proved that key, the scene is not in the organisation: the id is wrong.
TEST(SceneProject, ASceneTheProvenOrganisationKeyCannotSeeIsFatal) {
  for (const long status : {403L, 404L}) {
    SceneProjectFacts facts = AnsweredScene("1");
    facts.status = status;
    facts.body_is_a_scene = false;
    facts.org_key_verified = true;
    const Check check = CheckSceneProject(facts);
    EXPECT_EQ(check.name, "scene/project");
    EXPECT_EQ(check.severity, Severity::kFatal) << status;
    EXPECT_EQ(check.detail, "no scene a1b2c3d4-0000-4000-8000-000000000001 is visible to the "
                            "organisation that owns project 90003 (HTTP " +
                                std::to_string(status) + ")");
    EXPECT_TRUE(Mentions(check, "does not exist, or belongs to another organisation"));
    EXPECT_TRUE(Mentions(check, "not a credential fault"));
    EXPECT_FALSE(Mentions(check, "developer key")) << "the key is the one thing proven fine";
  }
}

// Without that proof the same answer could be the key's, so it stays a note, led by the fact.
TEST(SceneProject, ASceneAnUnprovenOrganisationKeyCannotSeeIsANoteThatSaysHowToProveIt) {
  for (const long status : {403L, 404L}) {
    SceneProjectFacts facts = AnsweredScene("1");
    facts.status = status;
    facts.body_is_a_scene = false;
    facts.org_key_verified = false;
    const Check check = CheckSceneProject(facts);
    EXPECT_EQ(check.name, "scene/project");
    EXPECT_EQ(check.severity, Severity::kAdvisory) << status;
    EXPECT_EQ(check.detail.rfind("no scene a1b2c3d4-0000-4000-8000-000000000001 is visible to "
                                 "this organisation key",
                                 0),
              0u)
        << check.detail;
    EXPECT_TRUE(Mentions(check, "does not exist or belongs to another organisation"));
    EXPECT_TRUE(Mentions(check, "Set C3D_PROJECT_ID"));
  }
}

namespace {

std::string HttpAnswer(const std::string& status_line, const std::string& body) {
  return "HTTP/1.1 " + status_line + "\r\nContent-Type: application/json\r\nContent-Length: " +
         std::to_string(body.size()) + "\r\n\r\n" + body;
}

SceneProjectFacts Asked(const std::string& version) {
  SceneProjectFacts facts;
  facts.scene_id = "a1b2c3d4-0000-4000-8000-000000000001";
  facts.scene_version = version;
  facts.expected_project = "90003";
  facts.org_key_present = true;
  facts.destination_permitted = true;
  return facts;
}

// A loopback port with nothing listening on it.
int ClosedPort() {
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  ::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address));
  socklen_t length = sizeof(address);
  ::getsockname(fd, reinterpret_cast<sockaddr*>(&address), &length);
  ::close(fd);
  return ntohs(address.sin_port);
}

}  // namespace

// The probe against a stub of the API: the request it makes, and each answer through to a verdict.
TEST(SceneProjectProbe, ReadsTheSceneWithTheOrganisationKeyAndReachesEachVerdict) {
  using c3d::testing_support::ThrottledHttpServer;
  struct Case {
    std::string answer;
    std::string version;
    std::string expected_project;
    Severity severity;
    std::string says;
    bool org_key_verified = true;
  };
  const std::vector<Case> cases = {
      {HttpAnswer("200 OK", kTwoVersionScene), "2", "90003", Severity::kOk, "version id 70002"},
      {HttpAnswer("200 OK", kTwoVersionScene), "1", "90004", Severity::kFatal,
       "belongs to project 90003, not to project 90004"},
      {HttpAnswer("200 OK", kTwoVersionScene), "70001", "90003", Severity::kFatal,
       "is the version id of version 1"},
      {HttpAnswer("200 OK", kTwoVersionScene), "5", "90003", Severity::kFatal,
       "has no version numbered 5"},
      {HttpAnswer("401 Unauthorized", R"({"error":"unauthorized"})"), "1", "90003",
       Severity::kAdvisory, "answered 401 to the scene lookup"},
      {HttpAnswer("403 Forbidden", "Forbidden"), "1", "90003", Severity::kFatal,
       "is visible to the organisation that owns project 90003 (HTTP 403)", true},
      {HttpAnswer("403 Forbidden", "Forbidden"), "1", "90003", Severity::kAdvisory,
       "is visible to this organisation key (HTTP 403)", false},
  };
  for (const Case& c : cases) {
    ThrottledHttpServer::Options options;
    options.response = c.answer;
    ThrottledHttpServer server(options);
    SceneProjectFacts asked = Asked(c.version);
    asked.expected_project = c.expected_project;
    asked.org_key_verified = c.org_key_verified;
    std::string body;
    const SceneProjectFacts facts = cognitive3d_ros::ProbeSceneProject(
        asked, "http://" + server.host(), "orgkey-abcdefghijklmnop", 5, &body);
    const Check check = CheckSceneProject(facts);
    EXPECT_EQ(check.severity, c.severity) << c.says << ": " << check.detail;
    EXPECT_TRUE(Mentions(check, c.says)) << check.detail;

    const auto received = server.received();
    ASSERT_EQ(received.size(), 1u) << "one read of the scene";
    EXPECT_EQ(received[0].path, "/v0/scenes/a1b2c3d4-0000-4000-8000-000000000001");
    EXPECT_NE(received[0].headers.find("Authorization: orgkey-abcdefghijklmnop\r\n"),
              std::string::npos)
        << "an orgkey- key goes as it is: " << received[0].headers;
    EXPECT_EQ(body, c.answer.substr(c.answer.find("\r\n\r\n") + 4)) << "mesh/files reads it too";
  }

  // A key without the prefix takes the legacy scheme.
  ThrottledHttpServer::Options options;
  options.response = HttpAnswer("200 OK", kTwoVersionScene);
  ThrottledHttpServer server(options);
  cognitive3d_ros::ProbeSceneProject(Asked("1"), "http://" + server.host(), "legacykey0123456",
                                     5, nullptr);
  ASSERT_EQ(server.received().size(), 1u);
  EXPECT_NE(server.received()[0].headers.find(
                "Authorization: APIKEY:ORGANIZATION legacykey0123456\r\n"),
            std::string::npos)
      << server.received()[0].headers;
}

TEST(SceneProjectProbe, AnApiThatDoesNotAnswerIsANote) {
  const SceneProjectFacts facts = cognitive3d_ros::ProbeSceneProject(
      Asked("1"), "http://127.0.0.1:" + std::to_string(ClosedPort()), "orgkey-abcdefghijklmnop",
      5, nullptr);
  EXPECT_EQ(facts.status, 0);
  const Check check = CheckSceneProject(facts);
  EXPECT_EQ(check.severity, Severity::kAdvisory);
  EXPECT_TRUE(Mentions(check, "could not reach the API"));
}

// --- every publisher of an allowlisted topic, against each other and against the subscription ---

namespace {

TopicQosFacts JointStates(std::vector<PublisherQos> publishers) {
  TopicQosFacts facts;
  facts.topic = "/joint_states";
  facts.override_key = "topic_config.joint_states";
  facts.publishers = std::move(publishers);
  return facts;
}

}  // namespace

TEST(TopicQos, PublishersThatDisagreeOnDurabilityAreNamedWithTheNodesOwnFix) {
  TopicQosFacts facts = JointStates({{"/joint_state_publisher", true, false},
                                     {"/joint_state_broadcaster", true, true}});
  facts.requests_reliable = true;
  facts.requested = "reliable/volatile/keep_last(10)";
  const Check check = CheckTopicQos({facts});
  EXPECT_EQ(check.name, "topics/qos");
  EXPECT_EQ(check.severity, Severity::kAdvisory);
  EXPECT_TRUE(Mentions(check, "/joint_states: its publishers disagree on durability "
                              "(/joint_state_broadcaster reliable/transient_local, "
                              "/joint_state_publisher reliable/volatile)"));
  // Word for word what the node logs when the race goes the wrong way.
  const std::string fix = cognitive3d_ros::QosRelaxation("topic_config.joint_states",
                                                         RMW_QOS_POLICY_DURABILITY);
  EXPECT_EQ(fix, "Set topic_config.joint_states.qos.durability: volatile to match a volatile "
                 "publisher.");
  EXPECT_EQ(check.remedy.rfind(fix, 0), 0u) << check.remedy;
  EXPECT_NE(cognitive3d_ros::DescribeQosIncompatibility(
                "/joint_states", 1, RMW_QOS_POLICY_DURABILITY, "topic_config.joint_states")
                .find(fix),
            std::string::npos);
  EXPECT_FALSE(Mentions(check, "reliability"));
}

TEST(TopicQos, PublishersThatDisagreeOnReliabilityAreNamedWithTheNodesOwnFix) {
  const Check check = CheckTopicQos({JointStates({{"/a", true, false}, {"/b", false, false}})});
  EXPECT_EQ(check.severity, Severity::kAdvisory);
  EXPECT_TRUE(Mentions(check, "disagree on reliability (/a reliable/volatile, /b "
                              "best_effort/volatile)"));
  EXPECT_EQ(check.remedy.rfind("Set topic_config.joint_states.qos.reliability: best_effort to "
                               "match a best-effort publisher.", 0),
            0u)
      << check.remedy;
  EXPECT_FALSE(Mentions(check, "durability"));
}

TEST(TopicQos, PublishersThatAgreePassAndAnOverrideThatPinsTheWeakerValueSettlesIt) {
  const Check agreeing = CheckTopicQos(
      {JointStates({{"/a", true, true}, {"/b", true, true}}), JointStates({{"/c", false, false}})});
  EXPECT_EQ(agreeing.name, "topics/qos");
  EXPECT_EQ(agreeing.severity, Severity::kOk);
  EXPECT_TRUE(Mentions(agreeing, "2 allowlisted topic(s)"));

  TopicQosFacts pinned = JointStates({{"/a", true, true}, {"/b", false, false}});
  pinned.override_durability = "volatile";
  pinned.override_reliability = "best_effort";
  EXPECT_EQ(CheckTopicQos({pinned}).severity, Severity::kOk)
      << "the override makes the request the same whichever publisher is discovered first";

  EXPECT_EQ(CheckTopicQos({pinned}).detail,
            "/joint_states: its publishers disagree on durability and reliability (/a "
            "reliable/transient_local, /b best_effort/volatile), and your overrides "
            "(topic_config.joint_states.qos.durability: volatile, "
            "topic_config.joint_states.qos.reliability: best_effort) reach every one");

  pinned.override_reliability.clear();
  EXPECT_EQ(CheckTopicQos({pinned}).severity, Severity::kAdvisory) << "half the fix";
}

// ros2_control's /joint_states with the override doctor recommends. The check passes, and must not
// pass by saying the publishers agree: they do not, and the override is why the run works.
TEST(TopicQos, PublishersThatDisagreeUnderAnOverrideThatReachesEveryOneAreNotCalledAgreeing) {
  TopicQosFacts joint_states = JointStates({{"/joint_state_publisher", true, false},
                                            {"/joint_state_broadcaster", true, true}});
  joint_states.override_durability = "volatile";
  joint_states.requests_reliable = true;
  joint_states.requested = "reliable/volatile/keep_last(10)";
  TopicQosFacts odom = JointStates({{"/odometry", true, false}});
  odom.topic = "/odom";
  odom.override_key = "topic_config.odom";

  const Check alone = CheckTopicQos({joint_states});
  EXPECT_EQ(alone.severity, Severity::kOk);
  EXPECT_EQ(alone.detail,
            "/joint_states: its publishers disagree on durability (/joint_state_broadcaster "
            "reliable/transient_local, /joint_state_publisher reliable/volatile), and your "
            "override (topic_config.joint_states.qos.durability: volatile) reaches every one");

  const Check beside = CheckTopicQos({joint_states, odom, odom});
  EXPECT_EQ(beside.severity, Severity::kOk);
  EXPECT_TRUE(Mentions(beside, "/joint_states: its publishers disagree on durability"));
  EXPECT_TRUE(Mentions(beside, "; the publishers of the other 2 allowlisted topic(s) agree on "
                               "reliability and durability"))
      << beside.detail;
  EXPECT_FALSE(Mentions(beside, "the publishers of 3 allowlisted topic(s) agree")) << beside.detail;
}

// An override can ask for more than a publisher offers. That is an incompatibility on every start.
TEST(TopicQos, ASubscriptionAPublisherCannotServeIsNamed) {
  TopicQosFacts facts = JointStates({{"/odom_publisher", false, false}});
  facts.override_key = "topic_config.odom";
  facts.override_durability = "transient_local";
  facts.requests_reliable = true;
  facts.requests_transient_local = true;
  facts.requested = "reliable/transient_local/keep_last(10)";
  const Check check = CheckTopicQos({facts});
  EXPECT_EQ(check.severity, Severity::kAdvisory);
  EXPECT_TRUE(Mentions(check, "nothing would arrive from /odom_publisher, which is volatile"));
  EXPECT_TRUE(Mentions(check, "nothing would arrive from /odom_publisher, which is best_effort"));
  EXPECT_TRUE(Mentions(check, "Set topic_config.odom.qos.durability: volatile"));
  EXPECT_TRUE(Mentions(check, "Set topic_config.odom.qos.reliability: best_effort"));
}

TEST(TopicQos, NoAllowlistedTopicOnTheGraphIsNotChecked) {
  const Check check = CheckTopicQos({});
  EXPECT_EQ(check.name, "topics/qos (NOT CHECKED)");
  EXPECT_EQ(check.severity, Severity::kOk);
}
