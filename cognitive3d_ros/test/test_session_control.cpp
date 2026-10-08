// NODE-level tests: each builds a real Cognitive3DNode and drives it the way an
// orchestrator does.
//
// A class tested in isolation cannot see a defect in how the node wires classes
// together -- an operator warning that goes permanently silent after a session
// restart is one -- because no such test ever builds a Cognitive3DNode.
// Session control is driven entirely through services, so testing it any other
// way would mean testing something other than what an orchestrator calls.
//
// No network: `protocol: http` against a port nothing listens on. The session is
// created, named and ended locally, which is all of the behaviour under test;
// uploads fail and the payloads stay in the spool, which is what they are for.

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <string>
#include <filesystem>
#include <functional>
#include <fstream>
#include <iterator>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <thread>
#include <tuple>
#include <utility>
#include <memory>
#include <mutex>
#include <vector>

#include <action_msgs/msg/goal_status_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "c3d/environment.hpp"
#include "c3d/properties.hpp"
#include "cognitive3d_ros/doctor.hpp"
#include "cognitive3d_ros/node.hpp"
#include "throttled_http_server.hpp"
#include "cognitive3d_ros_msgs/msg/event.hpp"
#include "cognitive3d_ros_msgs/srv/end_session.hpp"
#include "cognitive3d_ros_msgs/srv/start_session.hpp"

namespace fs = std::filesystem;
using cognitive3d_ros_msgs::msg::Event;
using cognitive3d_ros_msgs::srv::EndSession;
using cognitive3d_ros_msgs::srv::StartSession;

namespace {

// Configure refuses a scene_id that is not a UUID.
constexpr const char kSceneId[] = "a1b2c3d4-0000-4000-8000-000000000001";

// What configure() refused with, for a working offline configuration with `extra` applied on top,
// or empty when the node configured. An entry in `extra` replaces the base entry of that name.
std::string ConfigureError(const fs::path& dir, const std::vector<rclcpp::Parameter>& extra) {
  std::vector<rclcpp::Parameter> overrides = {
      {"scene_id", std::string(kSceneId)},
      {"scene_version", std::string("1")},
      {"device_id", std::string("robot")},
      {"spool_dir", dir.string()},
      {"protocol", std::string("http")},
      {"gateway", std::string("127.0.0.1:1")},
      {"pose_convention", std::string("unity")},
      {"auto_start_session", false},
  };
  for (const auto& parameter : extra) {
    overrides.erase(std::remove_if(overrides.begin(), overrides.end(),
                                   [&parameter](const rclcpp::Parameter& existing) {
                                     return existing.get_name() == parameter.get_name();
                                   }),
                    overrides.end());
    overrides.push_back(parameter);
  }
  rclcpp::NodeOptions options;
  options.parameter_overrides(overrides);
  auto node = std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
  const bool configured =
      node->configure().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
  return configured ? std::string() : node->configure_error();
}

class SessionControlTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() /
           ("c3d_sc_" + std::to_string(::getpid()) + "_" + std::to_string(counter_++));
    fs::create_directories(dir_);

    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> overrides = {
        {"scene_id", std::string(kSceneId)},
        {"pose_convention", std::string("unity")},
        {"scene_version", std::string("1")},
        {"device_id", std::string("robot")},
        {"spool_dir", dir_.string()},
        // Nothing listens here. Uploads fail and spool, which is the point.
        {"protocol", std::string("http")},
        {"gateway", std::string("127.0.0.1:1")},
        // The arrangement under test: activation brings up ingest and opens NO
        // session, so the orchestrator owns every session boundary.
        {"auto_start_session", false},
        {"session_name", std::string("")},
        {"topics", std::vector<std::string>{}},
    };
    for (const auto& extra : ExtraOverrides()) overrides.push_back(extra);
    options.parameter_overrides(overrides);
    node_ = std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
    client_node_ = std::make_shared<rclcpp::Node>("session_control_test_client");
    start_ = client_node_->create_client<StartSession>("/cognitive3d/start_session");
    events_ = client_node_->create_publisher<Event>("/cognitive3d/events",
                                                    rclcpp::QoS(100).reliable());
    end_ = client_node_->create_client<EndSession>("/cognitive3d/end_session");

    executor_.add_node(node_->get_node_base_interface());
    executor_.add_node(client_node_);

    ASSERT_EQ(node_->configure().id(),
              lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    ASSERT_TRUE(start_->wait_for_service(std::chrono::seconds(5)));
    ASSERT_TRUE(end_->wait_for_service(std::chrono::seconds(5)));
  }

  void TearDown() override {
    executor_.remove_node(client_node_);
    executor_.remove_node(node_->get_node_base_interface());
    node_.reset();
    client_node_.reset();
    std::error_code ec;
    fs::remove_all(dir_, ec);
  }

  // A derived fixture adds or replaces parameters without copying the list above.
  virtual std::vector<rclcpp::Parameter> ExtraOverrides() { return {}; }

  StartSession::Response::SharedPtr Start(const std::string& name,
                                          const std::string& properties_json = "",
                                          const std::string& participant_id = "") {
    auto request = std::make_shared<StartSession::Request>();
    request->session_name = name;
    request->properties_json = properties_json;
    request->participant_id = participant_id;
    auto future = start_->async_send_request(request);
    if (executor_.spin_until_future_complete(future, std::chrono::seconds(10)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      return nullptr;
    }
    return future.get();
  }

  EndSession::Response::SharedPtr End(const std::string& reason) {
    auto request = std::make_shared<EndSession::Request>();
    request->reason = reason;
    auto future = end_->async_send_request(request);
    if (executor_.spin_until_future_complete(future, std::chrono::seconds(10)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      return nullptr;
    }
    return future.get();
  }

  void PublishEvent(const std::string& name, const std::string& properties_json = "",
                    const std::string& dynamic_object_id = "") {
    Event event;
    event.name = name;
    event.properties_json = properties_json;
    event.dynamic_object_id = dynamic_object_id;
    events_->publish(event);
    // Spin both nodes for a bounded stretch so the subscription actually runs.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some(std::chrono::milliseconds(20));
    }
  }

  // Reads the SPOOL. Uploads fail (nothing listens on that port), so every
  // serialized payload is sitting on disk as a self-describing JSON chunk --
  // which makes this an end-to-end assertion rather than a smoke test: the event
  // was accepted, given properties, serialized in the wire format, and written.
  int SpoolOccurrences(const std::string& needle) {
    int found = 0;
    std::error_code ec;
    // RECURSIVE: DiskSpool keeps chunks in a subdirectory beside the markers.
    for (const auto& entry : fs::recursive_directory_iterator(dir_, ec)) {
      if (!entry.is_regular_file()) continue;
      std::ifstream in(entry.path(), std::ios::binary);
      const std::string text((std::istreambuf_iterator<char>(in)),
                             std::istreambuf_iterator<char>());
      // The chunk stores its payload as an OPAQUE STRING inside the envelope, so
      // the payload's quotes are escaped on disk and a raw search for
      // `"key":value` finds nothing. Parse the envelope and let nlohmann
      // unescape, which also means these assertions read as the wire format
      // rather than as its file encoding.
      std::string body;
      try {
        const auto envelope = nlohmann::json::parse(text);
        if (!envelope.contains("body") || !envelope["body"].is_string()) continue;
        body = envelope["body"].get<std::string>();
      } catch (const nlohmann::json::exception&) {
        continue;  // a marker file, not a chunk
      }
      std::size_t pos = 0;
      while ((pos = body.find(needle, pos)) != std::string::npos) {
        ++found;
        pos += needle.size();
      }
    }
    return found;
  }

  fs::path dir_;
  std::shared_ptr<cognitive3d_ros::Cognitive3DNode> node_;
  std::shared_ptr<rclcpp::Node> client_node_;
  rclcpp::Client<StartSession>::SharedPtr start_;
  rclcpp::Client<EndSession>::SharedPtr end_;
  rclcpp::Publisher<Event>::SharedPtr events_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  static int counter_;
};
int SessionControlTest::counter_ = 0;

}  // namespace

TEST_F(SessionControlTest, ActivationOpensNoSessionWhenTheOrchestratorOwnsThem) {
  // The whole reason auto_start_session exists. A session opened by activation is
  // a session the orchestrator did not name, and it would make the
  // orchestrator's own first call fail as "already running".
  auto ended = End("nothing should be running");
  ASSERT_NE(ended, nullptr);
  EXPECT_FALSE(ended->ok);
  EXPECT_EQ(ended->session_id, "");
}

TEST_F(SessionControlTest, AStartedSessionReportsItsIdAndCarriesTheRequestedName) {
  auto started = Start("maxvel/run-007");
  ASSERT_NE(started, nullptr);
  EXPECT_TRUE(started->ok) << started->message;
  // The id is the whole point of the response: without it a caller cannot find
  // the session it just created.
  EXPECT_FALSE(started->session_id.empty());
  EXPECT_TRUE(started->message.empty());
}

TEST_F(SessionControlTest, ASecondStartIsRefusedAND_NamesTheSessionThatBlockedIt) {
  auto first = Start("first");
  ASSERT_NE(first, nullptr);
  ASSERT_TRUE(first->ok) << first->message;

  auto second = Start("second");
  ASSERT_NE(second, nullptr);
  EXPECT_FALSE(second->ok);
  // Returning the BLOCKING session's id is what lets a caller tell "already
  // running as X" from a genuine failure, and name X in its own recovery path.
  // An empty string here would make the two indistinguishable.
  EXPECT_EQ(second->session_id, first->session_id);
  EXPECT_FALSE(second->message.empty());
}

TEST_F(SessionControlTest, EndReportsTheSessionItClosedAndThenAnotherCanStart) {
  auto first = Start("first");
  ASSERT_NE(first, nullptr);
  ASSERT_TRUE(first->ok) << first->message;

  auto ended = End("completed");
  ASSERT_NE(ended, nullptr);
  EXPECT_TRUE(ended->ok) << ended->message;
  EXPECT_EQ(ended->session_id, first->session_id);

  auto second = Start("second");
  ASSERT_NE(second, nullptr);
  EXPECT_TRUE(second->ok) << second->message;
  EXPECT_NE(second->session_id, first->session_id) << "a new session, not the old one resumed";
}

TEST_F(SessionControlTest, TypedPropertiesAreAccepted) {
  auto started = Start("typed",
                       R"({"site":"warehouse-3","robots":4,"speed":1.25,"charging":false})");
  ASSERT_NE(started, nullptr);
  EXPECT_TRUE(started->ok) << started->message;
  EXPECT_FALSE(started->session_id.empty());
}

TEST_F(SessionControlTest, AMalformedPropertiesBlobOpensNoSessionAndLeavesNoneBehind) {
  // `ok=false` with a non-empty session_id has exactly ONE meaning -- "already
  // running as X" -- so a properties failure must report an EMPTY id. The
  // alternative leaks: a caller told "X is already active" correctly declines to
  // end X, and the session the node opened on its behalf is never closed.
  auto started = Start("malformed", "{not json");
  ASSERT_NE(started, nullptr);
  EXPECT_FALSE(started->ok);
  EXPECT_TRUE(started->session_id.empty())
      << "a non-empty id here means 'already running as X', which is not what happened";
  EXPECT_NE(started->message.find("properties_json"), std::string::npos);

  // Nothing was opened, so the next start succeeds rather than being refused by
  // a session the caller never asked to keep.
  auto second = Start("second");
  ASSERT_NE(second, nullptr);
  EXPECT_TRUE(second->ok) << second->message;
  EXPECT_FALSE(second->session_id.empty());
}

TEST_F(SessionControlTest, ATopLevelArrayIsRejectedAndOpensNothing) {
  auto started = Start("array", R"(["site","warehouse-3"])");
  ASSERT_NE(started, nullptr);
  EXPECT_FALSE(started->ok);
  EXPECT_TRUE(started->session_id.empty());

  auto second = Start("second");
  ASSERT_NE(second, nullptr);
  EXPECT_TRUE(second->ok) << second->message;
}

TEST_F(SessionControlTest, ARefusalCarryingASessionIdAlwaysMeansAlreadyRunning) {
  // The invariant stated positively, so the two branches that produce
  // `ok=false` cannot drift back together: only a genuine already-active
  // refusal carries an id, and it is the ACTIVE session's.
  auto first = Start("the_incumbent");
  ASSERT_NE(first, nullptr);
  ASSERT_TRUE(first->ok) << first->message;

  auto blocked = Start("the_challenger");
  ASSERT_NE(blocked, nullptr);
  EXPECT_FALSE(blocked->ok);
  EXPECT_EQ(blocked->session_id, first->session_id);

  // Same refusal, malformed properties: still blocked by the incumbent, so the
  // id is still the incumbent's and not an empty string. The active check runs
  // first precisely so this reads as "already running", the more useful fact.
  auto blocked_and_malformed = Start("the_challenger", "{not json");
  ASSERT_NE(blocked_and_malformed, nullptr);
  EXPECT_FALSE(blocked_and_malformed->ok);
  EXPECT_EQ(blocked_and_malformed->session_id, first->session_id);
}

// ---------------------------------------------------------------------------
// Event ingress. This is the surface a mission node uses for the facts no topic
// carries -- its own state machine, a task outcome -- and a mission runner may
// publish a dozen distinct names through it.
// ---------------------------------------------------------------------------

TEST_F(SessionControlTest, AnEventPublishedWithNoSessionIsDroppedLoudlyNotSilently) {
  // A mission node that publishes before it calls ~/start_session loses every
  // event. Silently, the session it eventually opens looks merely uneventful.
  PublishEvent("mission.start");
  auto started = Start("after_the_drop");
  ASSERT_NE(started, nullptr);
  ASSERT_TRUE(started->ok) << started->message;
  auto ended = End("done");
  ASSERT_NE(ended, nullptr);
  ASSERT_TRUE(ended->ok) << ended->message;

  // The dropped event must not appear in the session that came after it. An
  // event queued before a session and delivered into it would attribute one
  // task's fact to the next task's measurement.
  EXPECT_EQ(SpoolOccurrences("mission.start"), 0);
}

TEST_F(SessionControlTest, EventsWithTypedPropertiesReachARunningSession) {
  auto started = Start("with_events");
  ASSERT_NE(started, nullptr);
  ASSERT_TRUE(started->ok) << started->message;

  // The shapes a mission runner publishes.
  PublishEvent("mission.start", R"({"waypoint_count":5,"experiment":"maxvel"})");
  PublishEvent("mission.waypoint_reached", R"({"waypoint_index":1,"distance_meters":2.5})");
  PublishEvent("robot.collision", R"({"frame_id":"base_link"})", "robot");
  PublishEvent("mission.completed", "{}");

  auto ended = End("completed");
  ASSERT_NE(ended, nullptr);
  EXPECT_TRUE(ended->ok) << ended->message;

  // End flushes, so by here the events are serialized on disk.
  EXPECT_EQ(SpoolOccurrences("mission.start"), 1);
  EXPECT_EQ(SpoolOccurrences("mission.waypoint_reached"), 1);
  EXPECT_EQ(SpoolOccurrences("robot.collision"), 1);
  EXPECT_EQ(SpoolOccurrences("mission.completed"), 1);
  // Typed, not stringified: the platform indexes numbers separately, so a
  // quoted 5 lands where a numeric filter cannot see it.
  EXPECT_EQ(SpoolOccurrences("\"waypoint_count\":5"), 1);
  EXPECT_EQ(SpoolOccurrences("\"waypoint_count\":\"5\""), 0) << "sent as a string";
  EXPECT_EQ(SpoolOccurrences("\"distance_meters\":2.5"), 1);
  EXPECT_EQ(SpoolOccurrences("\"experiment\":\"maxvel\""), 1) << "a string stays a string";
}

TEST_F(SessionControlTest, AnUnreachableRemoteVariablesFetchIsRecordedNotSwallowed) {
  // The fixture's gateway refuses connections. The session must still start, and it must SAY
  // it has no arms -- the alternative is a run in no arm with nothing to explain why.
  auto started = Start("ab/unreachable");
  ASSERT_NE(started, nullptr);
  ASSERT_TRUE(started->ok) << started->message;
  EXPECT_EQ(started->remote_variables_json, "")
      << "no answer must read as no answer, never as {}";

  auto ended = End("completed");
  ASSERT_NE(ended, nullptr);
  ASSERT_TRUE(ended->ok) << ended->message;

  EXPECT_EQ(SpoolOccurrences("c3d.remote_variables.unavailable"), 1);
  // Through the same door as every other event: the last sampled body position,
  // or the origin CARRYING the property that says so. This one fires inside
  // StartSession, before any pose has landed, so an event that claimed the origin
  // silently would put it on the map at the scene's zero.
  EXPECT_EQ(SpoolOccurrences("\"c3d.position_unknown\":true"), 1);
  EXPECT_EQ(SpoolOccurrences("\"identifier\":\"robot\""), 1)
      << "with no participant_id the draw is keyed on device_id";
  EXPECT_EQ(SpoolOccurrences("\"status\":0"), 1) << "connection refused is status 0, typed";
  EXPECT_EQ(SpoolOccurrences("c3d.remote_variable."), 0) << "nothing was applied";
}

TEST_F(SessionControlTest, TheRequestedParticipantIdKeysTheDrawAndNamesTheParticipant) {
  const std::string uuid = "3f2a9c1e-7b4d-4e1a-9c2b-0d5e6f7a8b9c";
  auto started = Start("ab/run", "", uuid);
  ASSERT_NE(started, nullptr);
  ASSERT_TRUE(started->ok) << started->message;
  EXPECT_EQ(started->remote_variables_json, "");
  auto ended = End("completed");
  ASSERT_NE(ended, nullptr);
  ASSERT_TRUE(ended->ok) << ended->message;

  EXPECT_EQ(SpoolOccurrences("\"identifier\":\"" + uuid + "\""), 1)
      << "the unavailable event must name the identifier the fetch used";
  EXPECT_GE(SpoolOccurrences("\"c3d.participant.id\":\"" + uuid + "\""), 1)
      << "the same string is the session's participant";
  EXPECT_EQ(SpoolOccurrences("\"c3d.participant.id\":\"robot\""), 0)
      << "device_id must not overwrite the requested participant";
}

class SessionControlNoRemoteVariablesTest : public SessionControlTest {
 protected:
  std::vector<rclcpp::Parameter> ExtraOverrides() override {
    return {{"remote_variables", false}};
  }
};

TEST_F(SessionControlNoRemoteVariablesTest, TheSwitchOffSkipsTheFetchEntirely) {
  auto started = Start("ab/disabled");
  ASSERT_NE(started, nullptr);
  ASSERT_TRUE(started->ok) << started->message;
  EXPECT_EQ(started->remote_variables_json, "");
  auto ended = End("completed");
  ASSERT_NE(ended, nullptr);
  EXPECT_EQ(SpoolOccurrences("c3d.remote_variables.unavailable"), 0)
      << "disabled is a choice, not a failure";
}

TEST_F(SessionControlTest, AParticipantIdOver64BytesIsRefusedBeforeAnySessionOpens) {
  // The platform draws the arm for the WHOLE string; core records only the first 64 bytes. A
  // session started on one and recorded under the other cannot be joined back to its arm, so the
  // request is refused instead -- and refused before anything is opened, which is what makes the
  // empty session_id readable as "nothing of yours is running".
  auto started = Start("too_long", "", std::string(65, 'a'));
  ASSERT_NE(started, nullptr);
  EXPECT_FALSE(started->ok);
  EXPECT_TRUE(started->session_id.empty())
      << "a non-empty id here means 'already running as X', which is not what happened";
  EXPECT_NE(started->message.find("64"), std::string::npos) << started->message;

  auto second = Start("second");
  ASSERT_NE(second, nullptr);
  EXPECT_TRUE(second->ok) << second->message;
}

TEST_F(SessionControlTest, APropertiesJsonNamingARemoteVariableIsRefusedWhileFetchIsEnabled) {
  // While the node draws arms itself, a consumer-supplied `c3d.remote_variable.*` is
  // indistinguishable on the dashboard from a drawn one -- and would be overwritten or would
  // overwrite, depending on ordering nobody can see.
  auto started = Start("own_arm", R"({"c3d.remote_variable.x":15})");
  ASSERT_NE(started, nullptr);
  EXPECT_FALSE(started->ok);
  EXPECT_TRUE(started->session_id.empty());
  EXPECT_NE(started->message.find("c3d.remote_variable.x"), std::string::npos)
      << started->message;

  auto second = Start("second");
  ASSERT_NE(second, nullptr);
  ASSERT_TRUE(second->ok) << second->message;
  auto ended = End("completed");
  ASSERT_NE(ended, nullptr);
  EXPECT_EQ(SpoolOccurrences("c3d.remote_variable.x"), 0) << "nothing was recorded";
}

TEST_F(SessionControlNoRemoteVariablesTest, AConsumerMaySupplyArmsWhenTheFetchIsDisabled) {
  // The other half of the rule: with the fetch off, the namespace belongs to whoever drew the
  // arm -- which is how a consumer's own pre-launch draw is carried.
  auto started = Start("own_arm", R"({"c3d.remote_variable.x":15})");
  ASSERT_NE(started, nullptr);
  ASSERT_TRUE(started->ok) << started->message;
  auto ended = End("completed");
  ASSERT_NE(ended, nullptr);
  ASSERT_TRUE(ended->ok) << ended->message;

  EXPECT_GE(SpoolOccurrences("\"c3d.remote_variable.x\":15"), 1)
      << "the consumer's own arm must reach the wire, typed";
}

// Every other `c3d.` key is the SDK's whatever remote_variables says: one supplied here would
// replace the SDK's own, and without `c3d.app.version` the platform discards the session.
TEST_F(SessionControlTest, APropertiesJsonNamingAnSdkPropertyOpensNoSession) {
  auto refused = Start("spoofed", R"({"site":"w3","c3d.app.version":"9.9"})");
  ASSERT_NE(refused, nullptr);
  EXPECT_FALSE(refused->ok);
  EXPECT_TRUE(refused->session_id.empty()) << "nothing was opened, so no id";
  EXPECT_NE(refused->message.find("c3d.app.version"), std::string::npos) << refused->message;

  auto started = Start("honest", R"({"site":"w3"})");
  ASSERT_NE(started, nullptr);
  ASSERT_TRUE(started->ok) << started->message;
  auto ended = End("completed");
  ASSERT_NE(ended, nullptr);
  EXPECT_EQ(SpoolOccurrences("\"c3d.app.version\":\"9.9\""), 0);
  EXPECT_GE(SpoolOccurrences("\"c3d.app.version\""), 1) << "the SDK's own value is still sent";
}

TEST_F(SessionControlNoRemoteVariablesTest, AnSdkPropertyIsRefusedWithTheFetchOffToo) {
  auto refused = Start("spoofed", R"({"c3d.participant.id":"someone-else"})");
  ASSERT_NE(refused, nullptr);
  EXPECT_FALSE(refused->ok);
  EXPECT_NE(refused->message.find("c3d.participant.id"), std::string::npos) << refused->message;
}

// The clamp is a pure function so it can be asserted without standing up a node. A timeout in
// the thousands is the failure it exists for: a milliseconds value written into a seconds
// parameter, which would otherwise block ~/start_session for the rest of the run.
TEST(ReaderTimeoutSeconds, ClampsIntoWholeSecondsAndTreatsNonPositiveAsDisabled) {
  EXPECT_EQ(cognitive3d_ros::ReaderTimeoutSeconds(0.0), 0);
  EXPECT_EQ(cognitive3d_ros::ReaderTimeoutSeconds(-1.0), 0);
  EXPECT_EQ(cognitive3d_ros::ReaderTimeoutSeconds(0.5), 1);
  EXPECT_EQ(cognitive3d_ros::ReaderTimeoutSeconds(3.0), 3);
  EXPECT_EQ(cognitive3d_ros::ReaderTimeoutSeconds(3000.0), 60);
  EXPECT_EQ(cognitive3d_ros::ReaderTimeoutSeconds(1e12), 60);
  // A NaN compares false against every bound, so it has to be answered before the arithmetic.
  EXPECT_EQ(cognitive3d_ros::ReaderTimeoutSeconds(std::numeric_limits<double>::quiet_NaN()), 0);
  EXPECT_EQ(cognitive3d_ros::ReaderTimeoutSeconds(std::numeric_limits<double>::infinity()), 60);
}

// device_id is the draw identifier for every session started without a participant_id -- which is
// every auto-started one. The service refuses an over-long participant_id; nothing downstream of
// configure can refuse a parameter, so the same bound has to be applied here.
TEST(NodeParameterWiring, AnOverLongDeviceIdIsRefusedAtConfigure) {
  const fs::path dir = fs::temp_directory_path() /
                       ("c3d_devid_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  auto make = [&dir](const std::string& device_id) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"scene_id", std::string(kSceneId)},
        {"pose_convention", std::string("unity")},
        {"scene_version", std::string("1")},
        {"device_id", device_id},
        {"spool_dir", dir.string()},
        {"protocol", std::string("http")},
        {"gateway", std::string("127.0.0.1:1")},
        {"auto_start_session", false},
        {"topics", std::vector<std::string>{}},
    });
    return std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
  };

  auto over = make(std::string(65, 'a'));
  EXPECT_NE(over->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      << "a device_id the platform draws 65 bytes for and the session records 64 of is not a "
         "usable configuration";
  EXPECT_NE(over->configure_error().find("device_id"), std::string::npos)
      << "the refusal has to name the parameter: " << over->configure_error();

  // Exactly at the cap is legal, and must stay legal: the bound is on what gets TRUNCATED.
  EXPECT_EQ(make(std::string(64, 'a'))->configure().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  std::error_code ec;
  fs::remove_all(dir, ec);
}

TEST_F(SessionControlTest, AMalformedPropertiesBlobDoesNotCostTheEvent) {
  // The event is the fact that something happened, and nothing else can
  // reconstruct it. A bad properties blob is a publisher defect; dropping the
  // event too would delete the more valuable half.
  auto started = Start("malformed_event_props");
  ASSERT_NE(started, nullptr);
  ASSERT_TRUE(started->ok) << started->message;

  PublishEvent("mission.failed", "{not json");
  PublishEvent("mission.failed", R"(["an","array"])");
  PublishEvent("", "{}");  // no name: this one IS dropped, there is nothing to record

  auto ended = End("done");
  ASSERT_NE(ended, nullptr);
  EXPECT_TRUE(ended->ok) << ended->message;

  EXPECT_EQ(SpoolOccurrences("mission.failed"), 2)
      << "both events must survive their bad properties";
}

// The key has one source, the process environment, and the refusals say so by the variable's
// name. A key in a params file refuses configure rather than being ignored, because ignoring it
// would turn a deployment that thinks it has a key into "no key" with nothing to say why.
TEST(ApplicationKeySource, TheKeyComesFromTheEnvironmentAndAKeyParameterIsRefusedByName) {
  const fs::path dir = fs::temp_directory_path() /
                       ("c3d_keyenv_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  auto make = [&dir](std::vector<rclcpp::Parameter> extra) {
    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> overrides = {
        {"scene_id", std::string(kSceneId)},
        {"scene_version", std::string("1")},
        {"device_id", std::string("robot")},
        {"spool_dir", dir.string()},
        {"protocol", std::string("http")},
        {"gateway", std::string("127.0.0.1:1")},
        {"pose_convention", std::string("unity")},
        {"auto_start_session", false},
        {"topics", std::vector<std::string>{}},
    };
    for (auto& parameter : extra) overrides.push_back(std::move(parameter));
    options.parameter_overrides(overrides);
    return std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
  };

  ::unsetenv("C3D_APPLICATION_API_KEY");
  {
    auto node = make({});
    EXPECT_NE(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        << "no key in the environment is not a usable configuration";
    EXPECT_NE(node->configure_error().find("C3D_APPLICATION_API_KEY"), std::string::npos)
        << "the refusal must name the variable, not a parameter: " << node->configure_error();
    // doctor maps the node's own refusal to its key check, so the two must agree on the text.
    const auto checks =
        cognitive3d_ros::doctor::ChecksForConfigurationRefusal(node->configure_error());
    ASSERT_FALSE(checks.empty());
    EXPECT_EQ(checks.front().name, "application key") << node->configure_error();
  }

  ::setenv("C3D_APPLICATION_API_KEY", "from-the-environment", 1);
  EXPECT_EQ(make({})->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      << "the environment is where the key comes from";

  // Any value, the empty one included: the name is never a parameter.
  for (const std::string value : {"from-a-params-file", ""}) {
    auto node = make({{"application_key", value}});
    EXPECT_NE(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        << "'" << value << "': a key in the parameters must refuse configure, not be ignored";
    EXPECT_EQ(node->configure_error().rfind("application_key is not a parameter", 0), 0u)
        << node->configure_error();
    EXPECT_NE(node->configure_error().find("C3D_APPLICATION_API_KEY"), std::string::npos)
        << node->configure_error();
    const auto checks =
        cognitive3d_ros::doctor::ChecksForConfigurationRefusal(node->configure_error());
    ASSERT_FALSE(checks.empty());
    EXPECT_NE(checks.front().name, "application key")
        << "a key in the parameters is not a missing key, and its remedy is different";
  }

  for (const std::string value : {"late", ""}) {
    {
      auto node = make({});
      // Rejected either way rclcpp has of saying so: the parameter is not declared.
      bool rejected = false;
      try {
        rejected = !node->set_parameter(rclcpp::Parameter("application_key", value)).successful;
      } catch (const rclcpp::exceptions::ParameterNotDeclaredException&) {
        rejected = true;
      }
      EXPECT_TRUE(rejected) << "'" << value << "': a runtime set of the key must be rejected";
      EXPECT_FALSE(node->has_parameter("application_key"));
    }
    {
      // A container may start the node taking undeclared parameters, and rclcpp then stores any
      // set.
      rclcpp::NodeOptions options;
      options.allow_undeclared_parameters(true);
      auto node = std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
      const auto result = node->set_parameter(rclcpp::Parameter("application_key", value));
      EXPECT_FALSE(result.successful)
          << "'" << value << "': a node that takes undeclared parameters must still reject the key";
      EXPECT_EQ(result.reason.rfind("application_key is not a parameter", 0), 0u) << result.reason;
      EXPECT_FALSE(node->has_parameter("application_key"));
    }
  }

  std::error_code ec;
  fs::remove_all(dir, ec);
}

// The key is never a parameter because anything on the graph can read one. A key in a params file
// is refused at configure, and in the meantime the node holds it nowhere a `ros2 param get`,
// `list` or `dump` reaches, and a `ros2 param set` cannot put one there.
TEST(ApplicationKeySource, AKeyInTheParametersIsNeverReadableAsAParameter) {
  const fs::path dir = fs::temp_directory_path() /
                       ("c3d_keyhidden_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  const std::string secret = "SECRET-NEVER-ON-THE-GRAPH";
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"scene_id", std::string(kSceneId)},
      {"scene_version", std::string("1")},
      {"spool_dir", dir.string()},
      {"protocol", std::string("http")},
      {"gateway", std::string("127.0.0.1:1")},
      {"pose_convention", std::string("unity")},
      {"application_key", secret},
  });
  auto node = std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
  EXPECT_NE(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_NE(node->configure_error().find("application_key is not a parameter"), std::string::npos)
      << node->configure_error();
  EXPECT_EQ(node->configure_error().find(secret), std::string::npos)
      << "the refusal must not repeat the key";

  EXPECT_FALSE(node->has_parameter("application_key"));
  const auto local = node->list_parameters({}, 0).names;
  EXPECT_EQ(std::find(local.begin(), local.end(), "application_key"), local.end());

  // Over the parameter services, which is what `ros2 param` uses.
  auto client_node = std::make_shared<rclcpp::Node>("parameter_reader");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(client_node);
  auto parameters = std::make_shared<rclcpp::AsyncParametersClient>(client_node, "/cognitive3d");
  ASSERT_TRUE(parameters->wait_for_service(std::chrono::seconds(5)));
  auto listed = parameters->list_parameters({}, 0);
  ASSERT_EQ(executor.spin_until_future_complete(listed, std::chrono::seconds(5)),
            rclcpp::FutureReturnCode::SUCCESS);
  const auto names = listed.get().names;
  EXPECT_EQ(std::find(names.begin(), names.end(), "application_key"), names.end())
      << "`ros2 param list` and `dump` would show it";
  auto got = parameters->get_parameters({"application_key"});
  ASSERT_EQ(executor.spin_until_future_complete(got, std::chrono::seconds(5)),
            rclcpp::FutureReturnCode::SUCCESS);
  for (const auto& parameter : got.get()) {
    EXPECT_EQ(parameter.get_type(), rclcpp::ParameterType::PARAMETER_NOT_SET)
        << "`ros2 param get` would read it";
  }
  auto set = parameters->set_parameters({rclcpp::Parameter("application_key", "late")});
  ASSERT_EQ(executor.spin_until_future_complete(set, std::chrono::seconds(5)),
            rclcpp::FutureReturnCode::SUCCESS);
  for (const auto& result : set.get()) EXPECT_FALSE(result.successful);
  EXPECT_FALSE(node->has_parameter("application_key"));

  executor.remove_node(client_node);
  executor.remove_node(node->get_node_base_interface());
  std::error_code ec;
  fs::remove_all(dir, ec);
}

// http sends the key unencrypted. It exists for a capture stub, so it is accepted only beside a
// gateway override, and never when that override names a Cognitive3D host.
TEST(NodeParameterWiring, HttpIsRefusedUnlessTheGatewayNamesACaptureStub) {
  const fs::path dir = fs::temp_directory_path() / ("c3d_http_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  EXPECT_EQ(ConfigureError(dir, {}), "") << "http beside a loopback stub is the supported use";

  const std::string no_gateway = ConfigureError(dir, {{"gateway", std::string("")}});
  EXPECT_NE(no_gateway.find("protocol 'http'"), std::string::npos) << no_gateway;

  const std::string production =
      ConfigureError(dir, {{"gateway", std::string(c3d::DefaultHosts().data)}});
  EXPECT_NE(production.find("protocol 'http'"), std::string::npos)
      << "http to the production data host must be refused: " << production;
  for (const std::string spelling : {"data.cognitive3d.com:80", "DATA.COGNITIVE3D.COM",
                                     "data.cognitive3d.com.", "http://data.cognitive3d.com"}) {
    const std::string refused = ConfigureError(dir, {{"gateway", spelling}});
    EXPECT_NE(refused.find("protocol 'http'"), std::string::npos)
        << "http to '" << spelling << "' must be refused: " << refused;
  }

  const std::string typo = ConfigureError(dir, {{"protocol", std::string("htps")}});
  EXPECT_NE(typo.find("protocol must be"), std::string::npos) << typo;

  EXPECT_EQ(ConfigureError(dir, {{"protocol", std::string("https")}, {"gateway", std::string("")}}),
            "");
  std::error_code ec;
  fs::remove_all(dir, ec);
}

// `remote_variables` is the switch and `remote_variables_timeout_s` only the timeout, so a
// timeout that is not a positive number is refused whatever the switch says.
TEST(NodeParameterWiring, TheRemoteVariablesTimeoutIsOnlyATimeout) {
  const fs::path dir = fs::temp_directory_path() / ("c3d_rvt_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  for (const bool enabled : {true, false}) {
    const std::string error = ConfigureError(
        dir, {{"remote_variables", enabled}, {"remote_variables_timeout_s", 0.0}});
    EXPECT_NE(error.find("remote_variables_timeout_s"), std::string::npos) << enabled;
    EXPECT_NE(error.find("remote_variables: false"), std::string::npos)
        << "the refusal must name the switch: " << error;
  }
  EXPECT_NE(ConfigureError(dir, {{"remote_variables_timeout_s", -1.0}}), "");
  EXPECT_EQ(ConfigureError(dir, {{"remote_variables", false}}), "");
  std::error_code ec;
  fs::remove_all(dir, ec);
}

// A node whose lifecycle each test drives itself, with a client for the two session services.
class LifecycleContractTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() /
           ("c3d_contract_" + std::to_string(::getpid()) + "_" + std::to_string(counter_++));
    fs::create_directories(dir_);
    client_ = std::make_shared<rclcpp::Node>("lifecycle_contract_test_client");
    start_ = client_->create_client<StartSession>("/cognitive3d/start_session");
    end_ = client_->create_client<EndSession>("/cognitive3d/end_session");
    executor_.add_node(client_);
  }
  void TearDown() override {
    if (node_) executor_.remove_node(node_->get_node_base_interface());
    executor_.remove_node(client_);
    node_.reset();
    client_.reset();
    std::error_code ec;
    fs::remove_all(dir_, ec);
  }
  void MakeNode(bool auto_start_session, std::vector<rclcpp::Parameter> extra = {}) {
    MakeNodeOf<cognitive3d_ros::Cognitive3DNode>(auto_start_session, std::move(extra));
  }
  // The same node as a subclass a test stands in for it.
  template <typename NodeT>
  std::shared_ptr<NodeT> MakeNodeOf(bool auto_start_session,
                                    std::vector<rclcpp::Parameter> extra = {}) {
    std::vector<rclcpp::Parameter> overrides = {
        {"scene_id", std::string(kSceneId)},
        {"scene_version", std::string("1")},
        {"device_id", std::string("robot")},
        {"spool_dir", dir_.string()},
        {"protocol", std::string("http")},
        {"gateway", std::string("127.0.0.1:1")},
        {"pose_convention", std::string("unity")},
        {"remote_variables", false},
        {"auto_start_session", auto_start_session},
    };
    for (auto& parameter : extra) overrides.push_back(std::move(parameter));
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    auto node = std::make_shared<NodeT>(options);
    node_ = node;
    executor_.add_node(node_->get_node_base_interface());
    return node;
  }
  StartSession::Response::SharedPtr Start(const std::string& name) {
    if (!start_->wait_for_service(std::chrono::seconds(5))) return nullptr;
    auto request = std::make_shared<StartSession::Request>();
    request->session_name = name;
    auto future = start_->async_send_request(request);
    if (executor_.spin_until_future_complete(future, std::chrono::seconds(10)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      return nullptr;
    }
    return future.get();
  }
  EndSession::Response::SharedPtr End() {
    if (!end_->wait_for_service(std::chrono::seconds(5))) return nullptr;
    auto request = std::make_shared<EndSession::Request>();
    request->reason = "test";
    auto future = end_->async_send_request(request);
    if (executor_.spin_until_future_complete(future, std::chrono::seconds(10)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      return nullptr;
    }
    return future.get();
  }
  // Spins until `condition` holds or `timeout` passes; graph changes reach a client
  // asynchronously, so a count read once can be stale.
  bool SpinUntil(const std::function<bool()>& condition,
                 std::chrono::milliseconds timeout = std::chrono::seconds(5)) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (condition()) return true;
      executor_.spin_some(std::chrono::milliseconds(20));
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return condition();
  }

  fs::path dir_;
  std::shared_ptr<cognitive3d_ros::Cognitive3DNode> node_;
  std::shared_ptr<rclcpp::Node> client_;
  rclcpp::Client<StartSession>::SharedPtr start_;
  rclcpp::Client<EndSession>::SharedPtr end_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  static int counter_;
};
int LifecycleContractTest::counter_ = 0;

// Sessions exist only while the node is active: a session opened while inactive has no pose or
// sensor ingest behind it. The refusal opens nothing, so its session id is empty.
TEST_F(LifecycleContractTest, StartSessionIsRefusedWhileInactiveAndOpensNothing) {
  MakeNode(/*auto_start_session=*/false);
  ASSERT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  auto refused = Start("too-early");
  ASSERT_NE(refused, nullptr);
  EXPECT_FALSE(refused->ok);
  EXPECT_TRUE(refused->session_id.empty())
      << "a non-empty id with ok=false means 'already running and not yours'";
  EXPECT_NE(refused->message.find("activate"), std::string::npos) << refused->message;
  auto nothing = End();
  ASSERT_NE(nothing, nullptr);
  EXPECT_FALSE(nothing->ok) << "the refused start must not have opened a session";

  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  auto started = Start("in-time");
  ASSERT_NE(started, nullptr);
  EXPECT_TRUE(started->ok) << started->message;
  EXPECT_FALSE(started->session_id.empty());
}

// Deactivate takes the node out of the contract as well as ending its session: a start after it is
// refused and opens nothing, exactly as before the first activation.
TEST_F(LifecycleContractTest, StartSessionIsRefusedAfterDeactivateAndOpensNothing) {
  MakeNode(/*auto_start_session=*/false);
  ASSERT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  auto started = Start("while-active");
  ASSERT_NE(started, nullptr);
  ASSERT_TRUE(started->ok) << started->message;
  ASSERT_EQ(node_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  auto refused = Start("after-deactivate");
  ASSERT_NE(refused, nullptr);
  EXPECT_FALSE(refused->ok) << "a session opened while inactive has no ingest behind it";
  EXPECT_TRUE(refused->session_id.empty()) << refused->session_id;
  EXPECT_NE(refused->message.find("activate"), std::string::npos) << refused->message;
  auto nothing = End();
  ASSERT_NE(nothing, nullptr);
  EXPECT_FALSE(nothing->ok) << "the refused start must not have opened a session";
}

// With auto_start_session, activation opens the session: a start before it is refused and
// leaves activation working, and a start after it is refused naming the parameter.
TEST_F(LifecycleContractTest, TheActivationSessionIsNamedInTheRefusalAndActivationStillWorks) {
  MakeNode(/*auto_start_session=*/true);
  ASSERT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  auto early = Start("too-early");
  ASSERT_NE(early, nullptr);
  EXPECT_FALSE(early->ok);
  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      << "a start refused while inactive must leave activation able to open its session";

  auto blocked = Start("the-name-i-wanted");
  ASSERT_NE(blocked, nullptr);
  EXPECT_FALSE(blocked->ok);
  EXPECT_FALSE(blocked->session_id.empty()) << "the refusal names the session that blocks it";
  EXPECT_NE(blocked->message.find("auto_start_session"), std::string::npos) << blocked->message;

  auto ended = End();
  ASSERT_NE(ended, nullptr);
  ASSERT_TRUE(ended->ok);
  EXPECT_EQ(ended->session_id, blocked->session_id);
  auto mine = Start("the-name-i-wanted");
  ASSERT_NE(mine, nullptr);
  ASSERT_TRUE(mine->ok) << mine->message;
  auto again = Start("another");
  ASSERT_NE(again, nullptr);
  EXPECT_FALSE(again->ok);
  EXPECT_EQ(again->session_id, mine->session_id);
  EXPECT_EQ(again->message.find("auto_start_session"), std::string::npos)
      << "a session the service opened is not the activation's: " << again->message;
}

// A translator for a type no core translator takes, counting how many of its calls overlap. Both
// halves are reached from different callbacks -- Translate from a subscription, Reset from the
// ~/start_session service -- so an overlap is two of the node's callbacks running at once.
std::atomic<int> g_probe_in_flight{0};
std::atomic<int> g_probe_max_in_flight{0};
std::atomic<int> g_probe_translations{0};
std::atomic<int> g_probe_resets{0};

void ProbeEnter() {
  const int now = ++g_probe_in_flight;
  int seen = g_probe_max_in_flight.load();
  while (now > seen && !g_probe_max_in_flight.compare_exchange_weak(seen, now)) {
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  --g_probe_in_flight;
}

class OverlapProbeTranslator : public cognitive3d_ros::ITranslator {
 public:
  const char* type_name() const override { return "sensor_msgs/msg/Temperature"; }
  void Translate(const rclcpp::SerializedMessage&, const cognitive3d_ros::TranslationContext&,
                 cognitive3d_ros::SessionRunner&) override {
    ProbeEnter();
    ++g_probe_translations;
  }
  void Reset() override {
    ProbeEnter();
    ++g_probe_resets;
  }
};

void RegisterOverlapProbe(cognitive3d_ros::TranslatorRegistry& registry) {
  registry.Register(std::make_unique<OverlapProbeTranslator>());
}

// The node serialises its own callbacks, so component_container_mt cannot race them: under a
// four-thread executor, ingest and session control never run at the same time.
TEST_F(LifecycleContractTest, UnderAMultiThreadedExecutorNoTwoCallbacksOverlap) {
  MakeNode(/*auto_start_session=*/false,
           {{"topics", std::vector<std::string>{"/probe/temperature"}},
            {"discovery_period_s", 0.1},
            {"report_period_s", 1.0},
            {"sample_rate_hz", 50.0}});
  executor_.remove_node(node_->get_node_base_interface());
  node_->AddTranslators(&RegisterOverlapProbe);
  ASSERT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  rclcpp::executors::MultiThreadedExecutor parallel(rclcpp::ExecutorOptions(), 4);
  parallel.add_node(node_->get_node_base_interface());
  std::thread spinner([&parallel]() { parallel.spin(); });

  auto temperature = client_->create_publisher<sensor_msgs::msg::Temperature>(
      "/probe/temperature", rclcpp::QoS(200).reliable());
  ASSERT_TRUE(SpinUntil([&] { return temperature->get_subscription_count() > 0; }));
  g_probe_max_in_flight = 0;
  g_probe_translations = 0;
  g_probe_resets = 0;

  // About half the group's time goes to ingest: enough for the two halves to meet often, and not
  // so much that the services wait behind a subscription that is always ready.
  std::atomic<bool> publishing{true};
  std::thread publisher([&]() {
    while (publishing) {
      temperature->publish(sensor_msgs::msg::Temperature());
      std::this_thread::sleep_for(std::chrono::milliseconds(4));
    }
  });
  // Failures are collected rather than asserted here, so the threads are always joined.
  std::string failure;
  for (int i = 0; i < 12 && failure.empty(); ++i) {
    auto started = Start("overlap-" + std::to_string(i));
    if (started == nullptr || !started->ok) {
      failure = "start " + std::to_string(i) + ": " + (started ? started->message : "no answer");
      break;
    }
    SpinUntil([] { return false; }, std::chrono::milliseconds(120));
    if (End() == nullptr) failure = "end " + std::to_string(i) + ": no answer";
  }
  publishing = false;
  publisher.join();
  parallel.cancel();
  spinner.join();
  parallel.remove_node(node_->get_node_base_interface());
  executor_.add_node(node_->get_node_base_interface());
  node_->deactivate();

  EXPECT_EQ(failure, "") << "session control did not answer under the multi-threaded executor";

  EXPECT_GT(g_probe_translations.load(), 100) << "the probe was barely driven";
  EXPECT_GE(g_probe_resets.load(), 12);
  EXPECT_EQ(g_probe_max_in_flight.load(), 1)
      << "two of the node's callbacks ran at the same time under a multi-threaded executor";
}

// Deactivate is the kill switch: every subscription goes, the tf listener and ~/events with it,
// and activation brings them back with a buffer that holds nothing from before.
TEST_F(LifecycleContractTest, DeactivateDestroysEverySubscriptionAndActivateRestoresThem) {
  MakeNode(/*auto_start_session=*/false);
  auto broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(client_);
  const auto publish_pose = [&]() {
    geometry_msgs::msg::TransformStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = client_->now();
    pose.child_frame_id = "base_link";
    pose.transform.rotation.w = 1.0;
    broadcaster->sendTransform(pose);
  };
  const auto resolves = [this]() {
    return node_->tf_buffer()->canTransform("map", "base_link", tf2::TimePointZero);
  };
  const auto subscribers = [this]() {
    return client_->count_subscribers("/tf") + client_->count_subscribers("/tf_static") +
           client_->count_subscribers("/cognitive3d/events");
  };
  ASSERT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_TRUE(SpinUntil([&] { return subscribers() == 0; }))
      << "a configured but inactive node subscribes to nothing";

  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_TRUE(SpinUntil([&] { return subscribers() == 3; })) << subscribers();
  EXPECT_TRUE(SpinUntil([&] {
    publish_pose();
    return resolves();
  })) << "the listener must fill the node's buffer while active";

  ASSERT_EQ(node_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_TRUE(SpinUntil([&] { return subscribers() == 0; }))
      << "/tf, /tf_static and ~/events must lose their subscriber: " << subscribers();
  publish_pose();
  SpinUntil([] { return false; }, std::chrono::milliseconds(300));
  EXPECT_FALSE(resolves())
      << "an inactive node holds no pose from before, and receives none while inactive";

  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_TRUE(SpinUntil([&] { return subscribers() == 3; })) << subscribers();
  EXPECT_TRUE(SpinUntil([&] {
    publish_pose();
    return resolves();
  })) << "the new listener must fill the buffer again";
}

// Shutdown can arrive straight from ACTIVE. Nothing left running may subscribe again afterwards.
TEST_F(LifecycleContractTest, ShutdownFromActiveStopsTheTimersThatWouldResubscribe) {
  MakeNode(/*auto_start_session=*/false, {{"topics", std::vector<std::string>{"/probe/battery"}},
                                          {"discovery_period_s", 0.2}});
  auto battery = client_->create_publisher<sensor_msgs::msg::BatteryState>("/probe/battery", 10);
  ASSERT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_TRUE(SpinUntil([&] { return battery->get_subscription_count() > 0; }))
      << "the allowlisted topic was never subscribed";

  ASSERT_EQ(node_->shutdown().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED);
  EXPECT_TRUE(SpinUntil([&] { return battery->get_subscription_count() == 0; }));
  // Several discovery periods, spun: a timer that survived would have resubscribed by now.
  SpinUntil([] { return false; }, std::chrono::milliseconds(1500));
  EXPECT_EQ(battery->get_subscription_count(), 0u) << "a timer survived shutdown";
}

// A transform that stops advancing is counted in the session's own c3d.stale_poses series.
TEST_F(LifecycleContractTest, AStaleRobotTransformIsCountedInTheSession) {
  MakeNode(/*auto_start_session=*/true, {{"fallback_reference_frame", std::string("")}});
  auto broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(client_);
  geometry_msgs::msg::TransformStamped pose;
  pose.header.frame_id = "map";
  pose.child_frame_id = "base_link";
  pose.header.stamp = client_->now();
  pose.transform.rotation.w = 1.0;
  ASSERT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  // Published at one stamp and never advanced, which is what a localiser that died leaves behind.
  SpinUntil([&] {
    broadcaster->sendTransform(pose);
    return node_->tf_buffer()->canTransform("map", "base_link", tf2::TimePointZero);
  });
  // Past the five-second bound, and past the next five-second telemetry record.
  SpinUntil([] { return false; }, std::chrono::milliseconds(11500));
  ASSERT_EQ(node_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  bool counted = false;
  std::error_code ec;
  for (const auto& entry : fs::recursive_directory_iterator(dir_, ec)) {
    if (!entry.is_regular_file()) continue;
    std::ifstream in(entry.path(), std::ios::binary);
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    try {
      const auto envelope = nlohmann::json::parse(text);
      if (!envelope.contains("body") || !envelope["body"].is_string()) continue;
      const std::string body = envelope["body"].get<std::string>();
      // The sensor stream records each series under its name with [time, value] pairs.
      const std::size_t at = body.find("\"c3d.stale_poses\"");
      if (at == std::string::npos) continue;
      const auto parsed = nlohmann::json::parse(body);
      for (const auto& series : parsed.value("data", nlohmann::json::array())) {
        if (series.value("name", "") != "c3d.stale_poses") continue;
        for (const auto& point : series.value("data", nlohmann::json::array())) {
          if (point.is_array() && point.size() >= 2 && point[1].get<double>() > 0.0) counted = true;
        }
      }
    } catch (const nlohmann::json::exception&) {
    }
  }
  EXPECT_TRUE(counted) << "no c3d.stale_poses sample above zero reached the spool";
}

// An unset spool_dir is a per-user state directory, never a shared world-writable one.
TEST(SpoolDirectory, TheDefaultIsAPerUserStateDirectory) {
  using Env = std::map<std::string, std::string>;
  const auto resolve = [](const Env& env) {
    return cognitive3d_ros::DefaultSpoolDirectory([&env](const char* name) -> const char* {
      const auto found = env.find(name);
      return found == env.end() ? nullptr : found->second.c_str();
    });
  };
  EXPECT_EQ(resolve({{"XDG_STATE_HOME", "/state"}, {"HOME", "/home/robot"}}), "/state/cognitive3d");
  EXPECT_EQ(resolve({{"HOME", "/home/robot"}}), "/home/robot/.local/state/cognitive3d");
  EXPECT_EQ(resolve({{"XDG_STATE_HOME", ""}, {"HOME", "/home/robot"}}),
            "/home/robot/.local/state/cognitive3d");
  EXPECT_EQ(resolve({{"XDG_STATE_HOME", "relative/state"}, {"HOME", "/home/robot"}}),
            "/home/robot/.local/state/cognitive3d")
      << "a relative XDG_STATE_HOME is invalid and ignored";
  EXPECT_EQ(resolve({}), "");
}

// The node resolves it at configure, from its own environment, and reports the result.
TEST(SpoolDirectory, AnUnsetSpoolDirResolvesAtConfigure) {
  const fs::path state = fs::temp_directory_path() / ("c3d_state_" + std::to_string(::getpid()));
  const char* saved = std::getenv("XDG_STATE_HOME");
  const std::string restore = saved ? saved : "";
  ::setenv("XDG_STATE_HOME", state.c_str(), 1);
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"scene_id", std::string(kSceneId)},
      {"scene_version", std::string("1")},
      {"pose_convention", std::string("unity")},
      {"protocol", std::string("http")},
      {"gateway", std::string("127.0.0.1:1")},
  });
  auto node = std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
  EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      << node->configure_error();
  EXPECT_EQ(node->resolved_config().spool_dir, (state / "cognitive3d").string());
  node->cleanup();
  if (saved) ::setenv("XDG_STATE_HOME", restore.c_str(), 1);
  else ::unsetenv("XDG_STATE_HOME");
  std::error_code ec;
  fs::remove_all(state, ec);
}

// Every value that would otherwise be accepted and then misbehave is refused by name.
TEST(ParameterValidation, OutOfRangeValuesAreRefusedByName) {
  const fs::path dir = fs::temp_directory_path() / ("c3d_validate_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  const std::vector<std::pair<std::string, rclcpp::Parameter>> refused = {
      {"sample_rate_hz", {"sample_rate_hz", 0.0}},
      {"sample_rate_hz", {"sample_rate_hz", -5.0}},
      {"discovery_period_s", {"discovery_period_s", 0.0}},
      {"report_period_s", {"report_period_s", -1.0}},
      {"event_rate_limit_hz", {"event_rate_limit_hz", -1.0}},
      {"spool_max_mb", {"spool_max_mb", -1}},
      {"spool_max_mb", {"spool_max_mb", 0}},
      {"camera_flush_bytes", {"camera_flush_bytes", -1}},
      {"lidar_flush_bytes", {"lidar_flush_bytes", -4096}},
  };
  for (const auto& [name, parameter] : refused) {
    const std::string error = ConfigureError(dir, {parameter});
    EXPECT_NE(error.find(name), std::string::npos)
        << parameter.get_name() << "=" << parameter.value_to_string() << " gave: " << error;
  }
  EXPECT_EQ(ConfigureError(dir, {{"event_rate_limit_hz", 0.0}}), "") << "0 is no cap";

  // Each timer's period is held between a millisecond and a day, because rclcpp refuses one that
  // does not fit in nanoseconds, and only at activation, once the session has opened.
  for (const rclcpp::Parameter& period :
       std::vector<rclcpp::Parameter>{{"report_period_s", 1.0e12},
                                      {"report_period_s", 86400.5},
                                      {"discovery_period_s", 1.0e12},
                                      {"discovery_period_s", 0.0005},
                                      {"sample_rate_hz", 1000.5},
                                      {"sample_rate_hz", 1.0e-6},
                                      {"sample_rate_hz", std::numeric_limits<double>::infinity()}}) {
    const std::string error = ConfigureError(dir, {period});
    EXPECT_EQ(error.rfind(period.get_name() + " must be ", 0), 0u)
        << period.get_name() << "=" << period.value_to_string() << " gave: " << error;
  }
  for (const rclcpp::Parameter& period :
       std::vector<rclcpp::Parameter>{{"report_period_s", 86400.0},
                                      {"discovery_period_s", 0.001},
                                      {"sample_rate_hz", 1000.0},
                                      {"sample_rate_hz", 1.0 / 86400.0}}) {
    EXPECT_EQ(ConfigureError(dir, {period}), "")
        << period.get_name() << "=" << period.value_to_string() << " is inside the bound";
  }

  // Per sensor, including values that would wrap back into range when narrowed to an int.
  const std::vector<std::pair<std::string, rclcpp::Parameter>> sensor = {
      {"camera.front.jpeg_quality", {"camera.front.jpeg_quality", std::int64_t{4294967376}}},
      {"camera.front.max_width_px", {"camera.front.max_width_px", -1}},
      {"lidar.scan.pointcloud.max_points", {"lidar.scan.pointcloud.max_points", -1}},
      {"lidar.scan.pointcloud.zstd_level",
       {"lidar.scan.pointcloud.zstd_level", std::int64_t{4294967297}}},
  };
  for (const auto& [name, parameter] : sensor) {
    const std::string error = ConfigureError(
        dir, {{"cameras", std::vector<std::string>{"front"}},
              {"camera.front.topic", std::string("/image")},
              {"lidars", std::vector<std::string>{"scan"}},
              {"lidar.scan.topic", std::string("/points")},
              parameter});
    EXPECT_NE(error.find(name), std::string::npos) << name << " gave: " << error;
  }
  std::error_code ec;
  fs::remove_all(dir, ec);
}

// A relative name in `topics` or a sensor's topic resolves against the node's namespace, as
// `ros2 topic` would; compared with the graph as written, it would match nothing.
TEST(ParameterValidation, RelativeTopicNamesResolveAgainstTheNodesNamespace) {
  EXPECT_EQ(cognitive3d_ros::ExpandTopicName("odom", "/", "/cognitive3d"), "/odom");
  EXPECT_EQ(cognitive3d_ros::ExpandTopicName("odom", "/robot1", "/robot1/cognitive3d"),
            "/robot1/odom");
  EXPECT_EQ(cognitive3d_ros::ExpandTopicName("~/status", "/robot1", "/robot1/cognitive3d"),
            "/robot1/cognitive3d/status");
  EXPECT_EQ(cognitive3d_ros::ExpandTopicName("/odom", "/robot1", "/robot1/cognitive3d"), "/odom");
  EXPECT_EQ(cognitive3d_ros::ExpandTopicName("sensors/*", "/robot1", "/robot1/cognitive3d"),
            "/robot1/sensors/*");
  EXPECT_EQ(cognitive3d_ros::ExpandTopicName("~", "/robot1", "/robot1/cognitive3d"),
            "/robot1/cognitive3d");
  // ROS 2 refuses `~` followed by anything but `/`; it does not name `/robot1/cognitive3dstatus`.
  EXPECT_EQ(cognitive3d_ros::ExpandTopicName("~status", "/robot1", "/robot1/cognitive3d"), "");
  const fs::path tilde = fs::temp_directory_path() / ("c3d_tilde_" + std::to_string(::getpid()));
  const std::vector<rclcpp::Parameter> sensors = {
      {"cameras", std::vector<std::string>{"front"}},
      {"camera.front.topic", std::string("/image")},
      {"lidars", std::vector<std::string>{"scan"}},
      {"lidar.scan.topic", std::string("/scan")},
  };
  EXPECT_EQ(ConfigureError(tilde, sensors), "");
  for (const rclcpp::Parameter& unresolvable :
       {rclcpp::Parameter("topics", std::vector<std::string>{"~status"}),
        rclcpp::Parameter("camera.front.topic", std::string("~image")),
        rclcpp::Parameter("lidar.scan.topic", std::string("~scan"))}) {
    std::vector<rclcpp::Parameter> extra = sensors;
    extra.push_back(unresolvable);  // a later entry replaces an earlier one of its name
    const std::string refused = ConfigureError(tilde, extra);
    EXPECT_NE(refused.find("'~' must be followed by '/'"), std::string::npos)
        << unresolvable.get_name() << ": " << refused;
  }
  {
    std::error_code ec;
    fs::remove_all(tilde, ec);
  }

  const fs::path dir = fs::temp_directory_path() / ("c3d_relative_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__ns:=/robot1"});
  options.parameter_overrides({
      {"scene_id", std::string(kSceneId)},
      {"scene_version", std::string("1")},
      {"spool_dir", dir.string()},
      {"pose_convention", std::string("unity")},
      {"protocol", std::string("http")},
      {"gateway", std::string("127.0.0.1:1")},
      {"topics", std::vector<std::string>{"odom", "/absolute"}},
      // Keyed by the entry as written.
      {"topic_config.odom.rate_hz", 7.0},
      {"cameras", std::vector<std::string>{"front"}},
      {"camera.front.topic", std::string("camera/image")},
  });
  auto node = std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      << node->configure_error();
  ASSERT_EQ(node->resolved_allowlist().size(), 2u);
  EXPECT_EQ(node->resolved_allowlist()[0], "/robot1/odom");
  EXPECT_EQ(node->resolved_allowlist()[1], "/absolute");
  const auto* rule = node->resolved_plan().Match("/robot1/odom");
  ASSERT_NE(rule, nullptr);
  EXPECT_DOUBLE_EQ(rule->rate_hz, 7.0);
  ASSERT_EQ(node->resolved_cameras().size(), 1u);
  EXPECT_EQ(node->resolved_cameras()[0].topic, "/robot1/camera/image");
  node->cleanup();
  std::error_code ec;
  fs::remove_all(dir, ec);
}

// `rate_hz: 2` in YAML is an integer to rclcpp, which aborts construction for a double parameter.
// Every double the node declares takes a whole number as the double it means, the per-topic and
// per-sensor ones included.
TEST(ParameterValidation, AWholeNumberIsAcceptedWhereADoubleIsDeclared) {
  const fs::path dir = fs::temp_directory_path() / ("c3d_integer_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"scene_id", std::string(kSceneId)},
      {"scene_version", std::string("1")},
      {"spool_dir", dir.string()},
      {"pose_convention", std::string("unity")},
      {"protocol", std::string("http")},
      {"gateway", std::string("127.0.0.1:1")},
      {"sample_rate_hz", 5},
      {"remote_variables_timeout_s", 2},
      {"topics", std::vector<std::string>{"/odom", "/sonar"}},
      {"topic_config.odom.rate_hz", 2},
      {"cameras", std::vector<std::string>{"front"}},
      {"camera.front.topic", std::string("/image")},
      {"camera.front.rate_hz", 3},
      {"lidars", std::vector<std::string>{"depth"}},
      {"lidar.depth.topic", std::string("/points")},
      {"lidar.depth.pointcloud.voxel_leaf_m", 1},
      {"sensors", std::vector<std::string>{"sonar"}},
      {"sensor.sonar.kind", std::string("cone_rangefinder")},
      {"sensor.sonar.topic", std::string("/sonar")},
      {"sensor.sonar.units", std::string("m")},
      {"sensor.sonar.cone_half_angle_deg", 15},
      {"sensor.sonar.range_max_m", 4},
      {"sensor.sonar.mount_xyz_m", std::vector<std::int64_t>{1, 0, 2}},
      {"sensor.sonar.mount_yaw_deg", 90},
  });
  std::shared_ptr<cognitive3d_ros::Cognitive3DNode> node;
  ASSERT_NO_THROW(node = std::make_shared<cognitive3d_ros::Cognitive3DNode>(options));
  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      << node->configure_error();
  EXPECT_DOUBLE_EQ(node->resolved_sampler_config().rate_hz, 5.0);
  EXPECT_EQ(node->get_parameter("sample_rate_hz").get_type(),
            rclcpp::ParameterType::PARAMETER_DOUBLE);
  const auto* rule = node->resolved_plan().Match("/odom");
  ASSERT_NE(rule, nullptr);
  EXPECT_DOUBLE_EQ(rule->rate_hz, 2.0);
  ASSERT_EQ(node->resolved_cameras().size(), 1u);
  EXPECT_DOUBLE_EQ(node->resolved_cameras()[0].rate_hz, 3.0);
  ASSERT_EQ(node->resolved_lidars().size(), 1u);
  EXPECT_DOUBLE_EQ(node->resolved_lidars()[0].pointcloud.voxel_leaf_m, 1.0);
  ASSERT_EQ(node->resolved_sensors().size(), 1u);
  const auto& sonar = node->resolved_sensors()[0];
  EXPECT_DOUBLE_EQ(sonar.cone_half_angle_deg, 15.0);
  EXPECT_DOUBLE_EQ(sonar.range_max_m, 4.0);
  EXPECT_EQ(sonar.mount_xyz_m, (std::vector<double>{1.0, 0.0, 2.0}));
  EXPECT_DOUBLE_EQ(sonar.mount_yaw_deg, 90.0);
  node->cleanup();
  std::error_code ec;
  fs::remove_all(dir, ec);
}

// The gaze envelope's `interval` is the time one sample stands for, so it follows the rate.
TEST_F(LifecycleContractTest, TheGazeIntervalFollowsTheSampleRate) {
  MakeNode(/*auto_start_session=*/true, {{"sample_rate_hz", 4.0}});
  ASSERT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_DOUBLE_EQ(node_->resolved_config().gaze_interval_s, 0.25);
  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_EQ(node_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  int gaze = 0;
  int at_the_rate = 0;
  std::error_code ec;
  for (const auto& entry : fs::recursive_directory_iterator(dir_, ec)) {
    if (!entry.is_regular_file()) continue;
    std::ifstream in(entry.path(), std::ios::binary);
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    try {
      const auto envelope = nlohmann::json::parse(text);
      if (!envelope.contains("body") || !envelope["body"].is_string()) continue;
      const auto body = nlohmann::json::parse(envelope["body"].get<std::string>());
      if (!body.contains("interval")) continue;
      ++gaze;
      if (body["interval"].get<double>() == 0.25) ++at_the_rate;
    } catch (const nlohmann::json::exception&) {
    }
  }
  ASSERT_GT(gaze, 0) << "no gaze envelope reached the spool";
  EXPECT_EQ(at_the_rate, gaze);
}

// Ten periods at the default rate are a third of a second, well inside the gap between two
// updates of a localiser that does not future-date its transform. The node's stale window at the
// default rate is the five-second floor, so such a localiser is not counted as stopped.
TEST_F(LifecycleContractTest, AtTheDefaultSampleRateTheStaleWindowIsFiveSeconds) {
  MakeNode(/*auto_start_session=*/false);
  ASSERT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_DOUBLE_EQ(node_->resolved_sampler_config().rate_hz, 30.0);
  EXPECT_DOUBLE_EQ(node_->resolved_config().gaze_interval_s, 1.0 / 30.0);
  EXPECT_DOUBLE_EQ(node_->resolved_sampler_config().stale_transform_after_s, 5.0);
  node_->cleanup();
}

// A namespaced node takes battery and diagnostics by type only under its own namespace.
TEST(DiscoveryScope, ANamespacedNodeTakesStandardTypesOnlyUnderItsNamespace) {
  const fs::path dir = fs::temp_directory_path() / ("c3d_scope_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "__ns:=/robot1"});
  options.parameter_overrides({
      {"scene_id", std::string(kSceneId)},
      {"scene_version", std::string("1")},
      {"spool_dir", dir.string()},
      {"pose_convention", std::string("unity")},
      {"protocol", std::string("http")},
      {"gateway", std::string("127.0.0.1:1")},
      {"auto_start_session", false},
      {"discovery_period_s", 0.2},
      {"topics", std::vector<std::string>{"/robot2/named_battery"}},
  });
  auto node = std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
  auto client = std::make_shared<rclcpp::Node>("discovery_scope_test_client");
  auto mine = client->create_publisher<sensor_msgs::msg::BatteryState>("/robot1/battery_state", 10);
  auto theirs =
      client->create_publisher<sensor_msgs::msg::BatteryState>("/robot2/battery_state", 10);
  auto named =
      client->create_publisher<sensor_msgs::msg::BatteryState>("/robot2/named_battery", 10);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(client);
  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(std::chrono::milliseconds(50));
  }
  EXPECT_EQ(node->topic_stats().count("/robot1/battery_state"), 1u) << "its own robot's battery";
  EXPECT_EQ(node->topic_stats().count("/robot2/battery_state"), 0u)
      << "another robot's battery, taken by type alone";
  EXPECT_EQ(node->topic_stats().count("/robot2/named_battery"), 1u)
      << "the allowlist reaches outside the namespace";
  node->deactivate();
  executor.remove_node(client);
  executor.remove_node(node->get_node_base_interface());
  std::error_code ec;
  fs::remove_all(dir, ec);
}

// The node leaves an allowlist entry's rate to its message type unless topic_config sets one.
TEST(ParameterValidation, AnAllowlistedTopicWithNoRuleTakesItsTypesDefaultRate) {
  const fs::path dir = fs::temp_directory_path() / ("c3d_typerate_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"scene_id", std::string(kSceneId)},
      {"scene_version", std::string("1")},
      {"spool_dir", dir.string()},
      {"pose_convention", std::string("unity")},
      {"protocol", std::string("http")},
      {"gateway", std::string("127.0.0.1:1")},
      {"topics", std::vector<std::string>{"/odom", "/joint_states"}},
      {"topic_config.joint_states.rate_hz", 2.0},
  });
  auto node = std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      << node->configure_error();
  EXPECT_DOUBLE_EQ(node->resolved_plan().Resolve("/odom", "nav_msgs/msg/Odometry").rate_hz, 30.0);
  EXPECT_DOUBLE_EQ(
      node->resolved_plan().Resolve("/joint_states", "sensor_msgs/msg/JointState").rate_hz, 2.0);
  node->cleanup();
  std::error_code ec;
  fs::remove_all(dir, ec);
}

// The two parameter files the package ships, loaded into the real node through --params-file,
// as a launch would load them. An untyped empty YAML list aborts the node at construction, so
// the files must carry none.
std::shared_ptr<cognitive3d_ros::Cognitive3DNode> NodeFromParamsFile(
    const std::string& file, const std::vector<rclcpp::Parameter>& overrides) {
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", file});
  // Constructor overrides win over a params file, which is what lets the test give the reference
  // file the values it deliberately leaves empty.
  options.parameter_overrides(overrides);
  return std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
}

// The collector answers 200 for a scene id that names no scene and discards every session, so the
// file's placeholder refuses configure, naming the parameter and where the id comes from. Once it
// is replaced, every other value the file ships configures.
TEST(ShippedParameterFiles, TheMinimalFileIsRefusedUntilItsSceneIdIsSet) {
  const fs::path dir = fs::temp_directory_path() / ("c3d_minimal_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  const std::string file = std::string(C3D_CONFIG_DIR) + "/params.minimal.yaml";
  {
    std::shared_ptr<cognitive3d_ros::Cognitive3DNode> node;
    ASSERT_NO_THROW(node = NodeFromParamsFile(file, {{"spool_dir", dir.string()}}));
    EXPECT_NE(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        << "the placeholder scene id configured";
    const std::string& error = node->configure_error();
    EXPECT_EQ(error.rfind("scene_id 'REPLACE-WITH-YOUR-SCENE-UUID' is not a scene id", 0), 0u)
        << "the placeholder is the only thing refused: " << error;
    EXPECT_NE(error.find("UUID"), std::string::npos) << error;
    EXPECT_NE(error.find("dashboard"), std::string::npos) << "where to find the id: " << error;
  }
  std::shared_ptr<cognitive3d_ros::Cognitive3DNode> node;
  ASSERT_NO_THROW(node = NodeFromParamsFile(
                      file, {{"spool_dir", dir.string()}, {"scene_id", std::string(kSceneId)}}));
  EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      << node->configure_error();
  EXPECT_EQ(node->get_parameter("pose_convention").as_string(), "gltf_authored")
      << "the file did not reach the node";
  node->cleanup();
  std::error_code ec;
  fs::remove_all(dir, ec);
}

// A scene id is a UUID in either case and nothing else: a placeholder, a version id, a name or a
// UUID with a character missing or out of place.
TEST(NodeParameterWiring, ASceneIdThatIsNotAUuidIsRefusedNamingTheParameter) {
  const fs::path dir = fs::temp_directory_path() / ("c3d_sceneid_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  for (const std::string accepted :
       {"a1b2c3d4-0000-4000-8000-000000000001", "9C1E5A7B-3D2F-4E8A-B6C4-1F0A2E7D5B93",
        "9c1e5a7b-3d2f-4E8A-b6c4-1F0A2E7d5b93"}) {
    EXPECT_EQ(ConfigureError(dir, {{"scene_id", accepted}}), "") << accepted;
  }
  for (const std::string refused :
       {"S", "REPLACE-WITH-YOUR-SCENE-UUID", "<your-scene-uuid>", "4711",
        "9c1e5a7b3d2f4 e8ab6c41f0a2e7d5b93", "9c1e5a7b-3d2f-4e8a-b6c4-1f0a2e7d5b9",
        "9c1e5a7b-3d2f-4e8a-b6c4-1f0a2e7d5b930", "9c1e5a7b-3d2f-4e8a-b6c4_1f0a2e7d5b93",
        "9c1e5a7b3-d2f-4e8a-b6c4-1f0a2e7d5b93", "9c1e5a7g-3d2f-4e8a-b6c4-1f0a2e7d5b93",
        " c1e5a7b-3d2f-4e8a-b6c4-1f0a2e7d5b93", "{c1e5a7b-3d2f-4e8a-b6c4-1f0a2e7d5b9}"}) {
    const std::string error = ConfigureError(dir, {{"scene_id", refused}});
    EXPECT_NE(error.find("scene_id '" + refused + "' is not a scene id"), std::string::npos)
        << "'" << refused << "': " << error;
  }
  // Empty is a missing value, said once as such.
  const std::string empty = ConfigureError(dir, {{"scene_id", std::string("")}});
  EXPECT_NE(empty.find("scene_id is empty"), std::string::npos) << empty;
  EXPECT_EQ(empty.find("is not a scene id"), std::string::npos) << empty;
  std::error_code ec;
  fs::remove_all(dir, ec);
}

TEST(ShippedParameterFiles, TheReferenceFileConstructsAndRefusesOnlyItsMandatoryBlanks) {
  const fs::path dir = fs::temp_directory_path() / ("c3d_reference_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  const std::string file = std::string(C3D_CONFIG_DIR) + "/cognitive3d.yaml";
  {
    std::shared_ptr<cognitive3d_ros::Cognitive3DNode> node;
    ASSERT_NO_THROW(node = NodeFromParamsFile(file, {{"spool_dir", dir.string()}}))
        << "the reference file must not abort the node at construction";
    EXPECT_NE(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    const std::string& error = node->configure_error();
    EXPECT_EQ(error.rfind("3 configuration problems: ", 0), 0u)
        << "exactly the three values with no default, and nothing else: " << error;
    for (const char* value : {"pose_convention", "scene_id", "scene_version"}) {
      EXPECT_NE(error.find(value), std::string::npos) << value << " in " << error;
    }
  }
  std::shared_ptr<cognitive3d_ros::Cognitive3DNode> node =
      NodeFromParamsFile(file, {{"spool_dir", dir.string()},
                                {"scene_id", std::string(kSceneId)},
                                {"scene_version", std::string("1")},
                                {"pose_convention", std::string("unity")}});
  EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      << "every other value in the reference file must be one the node accepts: "
      << node->configure_error();
  node->cleanup();
  std::error_code ec;
  fs::remove_all(dir, ec);
}

// What rclcpp throws at construction is turned by doctor into a check that names the parameter
// and the fix. Asserted on the exceptions THIS distro's rclcpp really throws, because their text
// differs between distros.
TEST(ConstructionFailures, DoctorNamesTheParameterAndTheFixForBothShapes) {
  const fs::path file = fs::temp_directory_path() /
                        ("c3d_empty_list_" + std::to_string(::getpid()) + ".yaml");
  {
    std::ofstream out(file);
    out << "/**:\n  ros__parameters:\n    cameras: []\n";
  }
  std::string what;
  try {
    NodeFromParamsFile(file.string(), {});
  } catch (const std::exception& thrown) {
    what = thrown.what();
  }
  ASSERT_FALSE(what.empty()) << "an empty YAML list no longer aborts construction";
  const auto empty_list = cognitive3d_ros::doctor::CheckConstruction(what);
  EXPECT_NE(empty_list.detail.find("cameras: []"), std::string::npos) << empty_list.detail;
  EXPECT_NE(empty_list.remedy.find("Delete"), std::string::npos) << empty_list.remedy;
  std::error_code ec;
  fs::remove(file, ec);

  what.clear();
  try {
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"scene_version", 1}});
    std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
  } catch (const std::exception& thrown) {
    what = thrown.what();
  }
  ASSERT_FALSE(what.empty()) << "an integer scene_version is accepted at construction";
  const auto mistyped = cognitive3d_ros::doctor::CheckConstruction(what);
  EXPECT_NE(mistyped.detail.find("scene_version must be a string"), std::string::npos)
      << mistyped.detail << " from: " << what;
  EXPECT_NE(mistyped.remedy.find("Quote"), std::string::npos) << mistyped.remedy;
}

// pose_convention has no default, because the right value depends on how the scene was authored
// and a wrong one is invisible in the payload. Its refusal is reported together with the other
// missing mandatory values, so a fresh params file learns everything in one configure.
TEST(NodeParameterWiring, PoseConventionHasNoDefaultAndIsRefusedWithTheOtherMissingValues) {
  const fs::path dir = fs::temp_directory_path() /
                       ("c3d_convention_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  auto make = [&dir](std::vector<rclcpp::Parameter> extra) {
    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> overrides = {
        {"scene_version", std::string("1")},
        {"device_id", std::string("robot")},
        {"spool_dir", dir.string()},
        {"protocol", std::string("http")},
        {"gateway", std::string("127.0.0.1:1")},
        {"auto_start_session", false},
    };
    for (auto& parameter : extra) overrides.push_back(std::move(parameter));
    options.parameter_overrides(overrides);
    return std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
  };

  {
    auto node = make({{"scene_id", std::string("")}});
    EXPECT_NE(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    const std::string& error = node->configure_error();
    EXPECT_NE(error.find("pose_convention is not set"), std::string::npos) << error;
    EXPECT_NE(error.find("scene_id is empty"), std::string::npos)
        << "the other missing value must be reported in the same refusal: " << error;
    const auto checks = cognitive3d_ros::doctor::ChecksForConfigurationRefusal(error);
    ASSERT_FALSE(checks.empty());
    EXPECT_EQ(checks.front().name, "pose/convention") << error;
  }
  {
    auto node = make({{"scene_id", std::string(kSceneId)}});
    EXPECT_NE(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        << "a node with no pose_convention must not configure";
  }
  EXPECT_EQ(make({{"scene_id", std::string(kSceneId)},
                  {"pose_convention", std::string("gltf_authored")}})
                ->configure()
                .id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  std::error_code ec;
  fs::remove_all(dir, ec);
}

// A PARAMETER THAT REACHES THE RUNNER, asserted through the node rather than
// through the runner.
//
// A parameter like `dynamic_up_offset_meters` can be declared, given a setter,
// documented and set in a consumer's config while the line calling the setter is
// missing -- every unit test of the setter passes and the value reaches nothing.
// A test on SessionRunner cannot see that; only one that builds the node and reads
// what it stored can. The same is true of any future parameter, which is why this
// test asserts the wiring and not the arithmetic.
TEST(NodeParameterWiring, TheUpOffsetParameterReachesTheDynamicChannel) {
  const auto dir = fs::temp_directory_path() /
                   ("c3d_offset_" + std::to_string(::getpid()));
  fs::create_directories(dir);

  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"scene_id", std::string(kSceneId)},
      {"scene_version", std::string("1")},
      {"device_id", std::string("robot")},
      {"spool_dir", dir.string()},
      {"protocol", std::string("http")},
      {"gateway", std::string("127.0.0.1:1")},
      {"auto_start_session", true},
      {"topics", std::vector<std::string>{}},
      // rep103 sends raw ROS, so the stored coordinate IS the assertion: no
      // convention arithmetic sits between the parameter and the number on disk.
      {"pose_convention", std::string("rep103")},
      {"dynamic_up_offset_meters", 0.75},
      // The dynamic object is off by default, and this test is about the channel it feeds.
      {"robot_dynamic_object", true},
  });
  auto node = std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
  auto tf_node = std::make_shared<rclcpp::Node>("offset_wiring_tf");
  tf2_ros::StaticTransformBroadcaster tf(tf_node);

  auto stamped = [&](const std::string& parent, const std::string& child, double z) {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = parent;
    t.child_frame_id = child;
    t.header.stamp = tf_node->now();
    t.transform.translation.x = 1.0;
    t.transform.translation.y = 2.0;
    t.transform.translation.z = z;
    t.transform.rotation.w = 1.0;
    return t;
  };
  // map -> odom at the origin, odom -> base_link at ROS (1, 2, 3). The x and y
  // above ride on both, so base_link lands at (2, 4, 3); only the up coordinate
  // matters here and it comes from this leg alone.
  auto identity = stamped("map", "odom", 0.0);
  identity.transform.translation.x = 0.0;
  identity.transform.translation.y = 0.0;
  tf.sendTransform({identity, stamped("odom", "base_link", 3.0)});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(tf_node);
  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // Spin long enough for the sample timer to fire at least once.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(std::chrono::milliseconds(20));
  }
  node->deactivate();
  node->cleanup();

  // Read the spool: uploads fail against a dead port, so the serialized payload
  // is on disk. Search the dynamics chunks for the lifted up coordinate.
  bool lifted = false, unlifted = false;
  std::error_code ec;
  for (const auto& entry : fs::recursive_directory_iterator(dir, ec)) {
    if (!entry.is_regular_file()) continue;
    std::ifstream in(entry.path(), std::ios::binary);
    const std::string text((std::istreambuf_iterator<char>(in)),
                           std::istreambuf_iterator<char>());
    std::string url, body;
    try {
      const auto envelope = nlohmann::json::parse(text);
      if (!envelope.contains("body") || !envelope["body"].is_string()) continue;
      url = envelope.value("url", std::string());
      body = envelope["body"].get<std::string>();
    } catch (const nlohmann::json::exception&) {
      continue;
    }
    if (url.find("/dynamics/") == std::string::npos) continue;
    if (body.find(",3.75]") != std::string::npos) lifted = true;
    if (body.find(",3.0]") != std::string::npos) unlifted = true;
  }
  executor.remove_node(tf_node);
  executor.remove_node(node->get_node_base_interface());
  node.reset();

  EXPECT_TRUE(lifted)
      << "dynamic_up_offset_meters: 0.75 must reach the runner; the stored up "
         "coordinate should read 3.75, not the robot's own 3.0";
  EXPECT_FALSE(unlifted) << "an unlifted sample means the offset reached only some samples";

  fs::remove_all(dir, ec);
}

// THE SAME WIRING QUESTION FOR THE OBJECT ID, and it earns its own test for the same reason the
// one above does: `robot_object_id` is declared, has a setter and is documented, and every unit
// test of the setter would pass with the call site missing. It is worth more here than the offset
// is, because this id is a CROSS-SYSTEM contract -- the uploaded dashboard object has to be
// registered under whatever this run emits, and a silent fallback to `robot` breaks that join
// while the replay still renders a robot.
TEST(NodeParameterWiring, TheRobotObjectIdParameterReachesEveryDynamicsSample) {
  const auto dir = fs::temp_directory_path() / ("c3d_objid_" + std::to_string(::getpid()));
  fs::create_directories(dir);

  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"scene_id", std::string(kSceneId)},
      {"pose_convention", std::string("unity")},
      {"scene_version", std::string("1")},
      // Deliberately NOT "robot": the device id also reaches this payload, and a bare substring
      // search for the default id would match it and pass for the wrong reason.
      {"device_id", std::string("unit-under-test")},
      {"spool_dir", dir.string()},
      {"protocol", std::string("http")},
      {"gateway", std::string("127.0.0.1:1")},
      {"auto_start_session", true},
      {"topics", std::vector<std::string>{}},
      {"robot_object_id", std::string("tb4")},
      {"robot_dynamic_object", true},
  });
  auto node = std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
  auto tf_node = std::make_shared<rclcpp::Node>("objid_wiring_tf");
  tf2_ros::StaticTransformBroadcaster tf(tf_node);
  auto stamped = [&](const std::string& parent, const std::string& child) {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = parent;
    t.child_frame_id = child;
    t.header.stamp = tf_node->now();
    t.transform.rotation.w = 1.0;
    return t;
  };
  tf.sendTransform({stamped("map", "odom"), stamped("odom", "base_link")});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(tf_node);
  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(std::chrono::milliseconds(20));
  }
  node->deactivate();
  node->cleanup();

  bool configured_id = false;
  bool default_id = false;
  std::error_code ec;
  for (const auto& entry : fs::recursive_directory_iterator(dir, ec)) {
    if (!entry.is_regular_file()) continue;
    std::ifstream in(entry.path(), std::ios::binary);
    const std::string text((std::istreambuf_iterator<char>(in)),
                           std::istreambuf_iterator<char>());
    std::string url, body;
    try {
      const auto envelope = nlohmann::json::parse(text);
      if (!envelope.contains("body") || !envelope["body"].is_string()) continue;
      url = envelope.value("url", std::string());
      body = envelope["body"].get<std::string>();
    } catch (const nlohmann::json::exception&) {
      continue;
    }
    if (url.find("/dynamics/") == std::string::npos) continue;
    // Both halves of the contract live in this one payload: the MANIFEST is keyed by the id and
    // every sample carries it, so a fix that reached only one of them would still be broken.
    if (body.find("\"id\":\"tb4\"") != std::string::npos &&
        body.find("\"manifest\":{\"tb4\"") != std::string::npos) {
      configured_id = true;
    }
    if (body.find("\"id\":\"robot\"") != std::string::npos ||
        body.find("\"manifest\":{\"robot\"") != std::string::npos) {
      default_id = true;
    }
  }
  executor.remove_node(tf_node);
  executor.remove_node(node->get_node_base_interface());
  node.reset();

  EXPECT_TRUE(configured_id)
      << "robot_object_id: tb4 must reach BOTH the manifest key and the dynamics samples";
  EXPECT_FALSE(default_id)
      << "the hardcoded default reached the wire despite the parameter being set";

  fs::remove_all(dir, ec);
}

// THE DEFAULT, asserted through the node for the same reason the two above are: the parameter is
// the only thing standing between a params file and a channel, and a default is a decision. The
// platform draws the robot from the pose timeline, so this run registers nothing and sends no
// dynamics -- while the timeline itself, which everything else is indexed by, still has to be
// there. A run that quietly stopped sampling would satisfy half of this on its own.
TEST(NodeParameterWiring, TheRobotDynamicObjectIsOffByDefaultAndTheTimelineIsNot) {
  const auto dir = fs::temp_directory_path() / ("c3d_dynoff_" + std::to_string(::getpid()));
  fs::create_directories(dir);

  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"scene_id", std::string(kSceneId)},
      {"pose_convention", std::string("unity")},
      {"scene_version", std::string("1")},
      {"device_id", std::string("unit-under-test")},
      {"spool_dir", dir.string()},
      {"protocol", std::string("http")},
      {"gateway", std::string("127.0.0.1:1")},
      {"auto_start_session", true},
      {"topics", std::vector<std::string>{}},
      // robot_dynamic_object deliberately unset: the default IS the assertion.
  });
  auto node = std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
  auto tf_node = std::make_shared<rclcpp::Node>("dynoff_wiring_tf");
  tf2_ros::StaticTransformBroadcaster tf(tf_node);
  auto stamped = [&](const std::string& parent, const std::string& child) {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = parent;
    t.child_frame_id = child;
    t.header.stamp = tf_node->now();
    t.transform.rotation.w = 1.0;
    return t;
  };
  tf.sendTransform({stamped("map", "odom"), stamped("odom", "base_link")});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(tf_node);
  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(std::chrono::milliseconds(20));
  }
  node->deactivate();
  node->cleanup();

  bool gaze = false, dynamics = false;
  std::error_code ec;
  for (const auto& entry : fs::recursive_directory_iterator(dir, ec)) {
    if (!entry.is_regular_file()) continue;
    std::ifstream in(entry.path(), std::ios::binary);
    const std::string text((std::istreambuf_iterator<char>(in)),
                           std::istreambuf_iterator<char>());
    std::string url;
    try {
      const auto envelope = nlohmann::json::parse(text);
      if (!envelope.contains("body") || !envelope["body"].is_string()) continue;
      url = envelope.value("url", std::string());
    } catch (const nlohmann::json::exception&) {
      continue;
    }
    if (url.find("/gaze/") != std::string::npos) gaze = true;
    if (url.find("/dynamics/") != std::string::npos) dynamics = true;
  }
  executor.remove_node(tf_node);
  executor.remove_node(node->get_node_base_interface());
  node.reset();

  EXPECT_TRUE(gaze) << "the pose timeline is what the robot is drawn from and must still be sent";
  EXPECT_FALSE(dynamics)
      << "the default registers no dynamic object, so no dynamics payload may reach the wire";

  fs::remove_all(dir, ec);
}

// THE SWITCH IS ON THE WIRE, in both positions. A session recorded with the object off and one
// whose dynamics were dropped by a defect are indistinguishable in the payload -- the stream is
// simply absent -- so the property is the only thing that lets a stored session be read at all.
// Asserted both ways round because a value hardcoded to either one satisfies half the question.
TEST(NodeParameterWiring, TheRobotDynamicObjectSwitchIsRecordedAsASessionProperty) {
  // Properties travel on gaze and are applied after Begin, so the union of every block on disk is
  // the honest question -- the same reason NodeIdentityProperties reads them that way.
  const auto properties_of_a_run = [](bool switch_on, const std::string& tag) {
    const auto dir = fs::temp_directory_path() /
                     ("c3d_dynprop_" + tag + "_" + std::to_string(::getpid()));
    fs::create_directories(dir);

    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> overrides = {
        {"scene_id", std::string(kSceneId)},
        {"pose_convention", std::string("unity")},
        {"scene_version", std::string("1")},
        {"device_id", std::string("robot")},
        {"spool_dir", dir.string()},
        {"protocol", std::string("http")},
        {"gateway", std::string("127.0.0.1:1")},
        {"auto_start_session", true},
        {"topics", std::vector<std::string>{}},
    };
    // Left unset for the off run: the default is half of what is under test.
    if (switch_on) overrides.push_back({"robot_dynamic_object", true});
    options.parameter_overrides(overrides);

    auto node = std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some(std::chrono::milliseconds(20));
    }
    node->deactivate();
    node->cleanup();

    nlohmann::json sent = nlohmann::json::object();
    std::error_code ec;
    for (const auto& entry : fs::recursive_directory_iterator(dir, ec)) {
      if (!entry.is_regular_file()) continue;
      std::ifstream in(entry.path(), std::ios::binary);
      const std::string text((std::istreambuf_iterator<char>(in)),
                             std::istreambuf_iterator<char>());
      try {
        const auto envelope = nlohmann::json::parse(text);
        if (!envelope.contains("body") || !envelope["body"].is_string()) continue;
        const auto body = nlohmann::json::parse(envelope["body"].get<std::string>());
        if (!body.contains("properties")) continue;
        for (const auto& [key, value] : body["properties"].items()) { sent[key] = value; }
      } catch (const nlohmann::json::exception&) {
        continue;
      }
    }
    executor.remove_node(node->get_node_base_interface());
    node.reset();
    fs::remove_all(dir, ec);
    return sent;
  };

  const auto off = properties_of_a_run(false, "off");
  ASSERT_TRUE(off.contains("c3d.ros.robot_dynamic_object")) << off.dump();
  EXPECT_EQ(off["c3d.ros.robot_dynamic_object"], false)
      << "the default run must say so: nothing else in the payload distinguishes it";

  const auto on = properties_of_a_run(true, "on");
  ASSERT_TRUE(on.contains("c3d.ros.robot_dynamic_object")) << on.dump();
  EXPECT_EQ(on["c3d.ros.robot_dynamic_object"], true)
      << "the parameter reached the runner but not the session";
}

// An empty id is REFUSED rather than silently replaced, and the refusal is UNCONDITIONAL -- this
// run has `robot_dynamic_object` at its default, so nothing here would read the value, and the
// configure still fails. The id is a cross-system contract with an uploaded object; a parameter
// that only became invalid once another parameter was switched on would pass this configure and
// fail the one that matters, which is the run already recording.
TEST(NodeParameterWiring, AnEmptyRobotObjectIdIsRefusedAtConfigure) {
  const auto dir = fs::temp_directory_path() / ("c3d_objid_empty_" + std::to_string(::getpid()));
  fs::create_directories(dir);

  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"scene_id", std::string(kSceneId)},
      {"pose_convention", std::string("unity")},
      {"scene_version", std::string("1")},
      {"spool_dir", dir.string()},
      {"topics", std::vector<std::string>{}},
      {"robot_object_id", std::string("")},
  });
  auto node = std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
  EXPECT_NE(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_NE(node->configure_error().find("robot_object_id"), std::string::npos)
      << "the refusal has to name the parameter: " << node->configure_error();

  std::error_code ec;
  fs::remove_all(dir, ec);
}

// THE CROSS-SDK IDENTITY PROPERTIES, asserted through the node because that is where two of the
// three are set. `c3d.app.sdktype` comes from core and is pinned in test_session.cpp; the other two
// are the node's. A dashboard view spanning several SDKs reads these three and nothing in the
// `c3d.ros.*` namespace, so the distro being in c3d.ros.distro does not make
// `c3d.app.engine.version` redundant.
TEST(NodeIdentityProperties, SendsTheCrossSdkEngineVersionAndOsAlongsideTheRosNamespacedOnes) {
  const auto dir = fs::temp_directory_path() / ("c3d_identity_" + std::to_string(::getpid()));
  fs::create_directories(dir);

  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"scene_id", std::string(kSceneId)},
      {"pose_convention", std::string("unity")},
      {"scene_version", std::string("1")},
      {"device_id", std::string("robot")},
      {"spool_dir", dir.string()},
      {"protocol", std::string("http")},
      {"gateway", std::string("127.0.0.1:1")},
      {"auto_start_session", true},
      {"topics", std::vector<std::string>{}},
  });
  auto node = std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(std::chrono::milliseconds(20));
  }
  node->deactivate();   // ends the session, which flushes
  node->cleanup();

  // Read the spool: uploads fail against a dead port, so every property block is on disk.
  //
  // THE UNION OF EVERY BLOCK, not the last one found. `Session::Begin` snapshots the properties it
  // knows at that moment into the OPENING payload, and the node applies its own -- `c3d.ros.*` and
  // the two added here -- immediately afterwards. So the first payload legitimately carries only
  // core's identity set, and a test that read one block would assert against a pre-session
  // snapshot.
  // Asserting the union is the honest question: did these reach the wire at all.
  nlohmann::json sent = nlohmann::json::object();
  std::error_code ec;
  for (const auto& entry : fs::recursive_directory_iterator(dir, ec)) {
    if (!entry.is_regular_file()) continue;
    std::ifstream in(entry.path(), std::ios::binary);
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    try {
      const auto envelope = nlohmann::json::parse(text);
      if (!envelope.contains("body") || !envelope["body"].is_string()) continue;
      const auto body = nlohmann::json::parse(envelope["body"].get<std::string>());
      if (!body.contains("properties")) continue;
      for (const auto& [key, value] : body["properties"].items()) { sent[key] = value; }
    } catch (const nlohmann::json::exception&) {
      continue;
    }
  }
  executor.remove_node(node->get_node_base_interface());
  node.reset();

  ASSERT_FALSE(sent.empty()) << "no property block reached the spool";
  const std::string properties = sent.dump();
  // The ROS-namespaced one.
  EXPECT_TRUE(sent.contains("c3d.ros.distro"));
  // The cross-SDK pair. engine.version carries the SAME value as c3d.ros.distro on purpose: ROS 2's
  // version IS its distro, so this is the engine's version rather than a second fact.
  ASSERT_TRUE(sent.contains("c3d.app.engine.version")) << properties;
  EXPECT_EQ(sent["c3d.app.engine.version"], sent["c3d.ros.distro"]);
  ASSERT_TRUE(sent.contains("c3d.device.os")) << properties;
  // Never empty. "unknown" is the honest answer where /etc/os-release is absent; an empty string
  // would read as "reported nothing" and a missing key as "never asked".
  EXPECT_FALSE(sent["c3d.device.os"].get<std::string>().empty());
  EXPECT_TRUE(sent.contains("c3d.ros.domain_id")) << properties;
  // Humble has no ROS_AUTOMATIC_DISCOVERY_RANGE, so a value recorded there would describe a
  // variable that decides nothing.
  EXPECT_EQ(sent.contains("c3d.ros.discovery_range"), std::string(C3D_TEST_ROS_DISTRO) != "humble")
      << "built for " << C3D_TEST_ROS_DISTRO << ": " << properties;

  fs::remove_all(dir, ec);
}

// A SESSION THE NODE REOPENS BY ITSELF must be the same session the operator configured.
//
// A bare SessionRunner::Restart -- End() then Begin() -- is not enough for the clock-jump branch.
// `Session::End` restores the properties to the snapshot taken inside `Begin`, and every property
// the node sets is applied AFTER Begin, so such a replacement session carries only core's six
// identity properties: no `c3d.ros.*`, no `c3d.app.engine.version`, no `c3d.device.os`, no
// participant id, no session name, and none of the operator's own `session_properties`. Only a
// test that drives a backward clock jump can see it.
TEST(NodeIdentityProperties, ASessionReopenedAfterABackwardClockJumpCarriesTheWholeIdentity) {
  const auto dir = fs::temp_directory_path() / ("c3d_jump_" + std::to_string(::getpid()));
  fs::create_directories(dir);

  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"scene_id", std::string(kSceneId)},
      {"pose_convention", std::string("unity")},
      {"scene_version", std::string("1")},
      {"device_id", std::string("robot")},
      {"spool_dir", dir.string()},
      {"protocol", std::string("http")},
      {"gateway", std::string("127.0.0.1:1")},
      // NOT auto-started, and `session_name` deliberately empty: the session is opened through
      // the SERVICE, the way a real orchestrator does it, so the name and the properties exist
      // ONLY in the request. Driven by parameters instead, `StartSession`'s own
      // `ApplySessionIdentity("")` re-applies them on the restart and the replay this test exists
      // to pin is never exercised -- verified by mutation: with parameters, deleting the replay
      // leaves the whole suite passing.
      {"auto_start_session", false},
      {"topics", std::vector<std::string>{}},
      {"use_sim_time", true},
      {"session_name", std::string("")},
  });
  auto node = std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
  auto client = std::make_shared<rclcpp::Node>("c3d_test_jump_client");
  auto start = client->create_client<cognitive3d_ros_msgs::srv::StartSession>(
      "/cognitive3d/start_session");
  // A separate node for /clock: the node under test consumes it through rclcpp's own time source,
  // which is exactly how a sim drives it.
  auto clock_source = std::make_shared<rclcpp::Node>("c3d_test_clock_source");
  auto clock_publisher =
      clock_source->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());

  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  // Added AFTER activate on purpose: the node's callback groups are created at configure and its
  // timers at activate, and an executor that collected the node before either does not necessarily
  // pick them up -- which shows as "the code never restarted the session" rather than as a test
  // harness problem.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(clock_source);
  executor.add_node(client);
  ASSERT_TRUE(start->wait_for_service(std::chrono::seconds(5)));
  auto request = std::make_shared<cognitive3d_ros_msgs::srv::StartSession::Request>();
  request->session_name = "patrol/run-007";
  request->properties_json = R"({"site": "warehouse-3"})";
  auto pending = start->async_send_request(request);
  ASSERT_EQ(executor.spin_until_future_complete(pending, std::chrono::seconds(10)),
            rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(pending.get()->ok) << pending.get()->message;

  const auto drive_clock_to = [&](double seconds) {
    rosgraph_msgs::msg::Clock message;
    message.clock = rclcpp::Time(static_cast<int64_t>(seconds * 1e9));
    // The sample timer is a 30 Hz WALL timer and the jump is only observed on a tick, so this has
    // to let real time pass. `spin_some` does NOT wait out its argument -- it returns as soon as no
    // more work is ready -- so a loop of forty spin_somes can finish in microseconds and the timer
    // never fires. Sleeping is what makes the tick happen; without it the test reports a correct
    // node as broken.
    for (int i = 0; i < 30; ++i) {
      clock_publisher->publish(message);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      executor.spin_some(std::chrono::milliseconds(10));
    }
  };
  // Assert the PRECONDITIONS explicitly rather than inferring them from the outcome. A test whose
  // environmental setup silently fails reports the code as broken, with nothing in its output to
  // say the setup was at fault.
  ASSERT_TRUE(node->get_clock()->ros_time_is_active())
      << "the node is not on sim time, so no /clock value can reach it";
  drive_clock_to(100.0);   // the sim running
  ASSERT_NEAR(node->get_clock()->now().seconds(), 100.0, 1.0)
      << "/clock never reached the node under test";
  drive_clock_to(1.0);     // a world reset, or a bag looping
  ASSERT_NEAR(node->get_clock()->now().seconds(), 1.0, 1.0)
      << "the backward /clock value never reached the node under test";

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    executor.spin_some(std::chrono::milliseconds(10));
  }
  node->deactivate();
  node->cleanup();

  // Properties, grouped by the session they were sent for. Grouping is the whole point: the first
  // session is correct either way, so a check over the union of every block cannot see a reopened
  // session that lost its properties.
  std::map<std::string, nlohmann::json> per_session;
  // Raw substring count over every spooled body, the same technique
  // SessionControlTest::SpoolOccurrences uses -- the unavailable event does not carry
  // "properties"+"sessionid" and would not be seen by the per-session parse below.
  int unavailable_events = 0;
  std::error_code ec;
  for (const auto& entry : fs::recursive_directory_iterator(dir, ec)) {
    if (!entry.is_regular_file()) continue;
    std::ifstream in(entry.path(), std::ios::binary);
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    try {
      const auto envelope = nlohmann::json::parse(text);
      if (!envelope.contains("body") || !envelope["body"].is_string()) continue;
      const std::string raw_body = envelope["body"].get<std::string>();
      std::size_t pos = 0;
      while ((pos = raw_body.find("c3d.remote_variables.unavailable", pos)) != std::string::npos) {
        ++unavailable_events;
        pos += 1;
      }
      const auto body = nlohmann::json::parse(raw_body);
      if (!body.contains("properties") || !body.contains("sessionid")) continue;
      auto& into = per_session[body["sessionid"].get<std::string>()];
      for (const auto& [key, value] : body["properties"].items()) into[key] = value;
    } catch (const nlohmann::json::exception&) {
      continue;
    }
  }
  executor.remove_node(node->get_node_base_interface());
  executor.remove_node(clock_source);
  executor.remove_node(client);
  node.reset();

  ASSERT_GE(per_session.size(), 2u)
      << "the backward clock jump did not close one session and open another";
  // The first fetch fails against this dead gateway; the reopen must re-record the same
  // unavailable event WITHOUT fetching again -- one per session opened, never zero, never one.
  EXPECT_EQ(unavailable_events, static_cast<int>(per_session.size()))
      << "the reopened session did not reapply (or silently skipped) the remote-variables outcome";
  for (const auto& [session_id, properties] : per_session) {
    const std::string dumped = properties.dump();
    EXPECT_TRUE(properties.contains("c3d.app.engine.version")) << session_id << ": " << dumped;
    EXPECT_TRUE(properties.contains("c3d.device.os")) << session_id << ": " << dumped;
    EXPECT_TRUE(properties.contains("c3d.ros.distro")) << session_id << ": " << dumped;
    EXPECT_TRUE(properties.contains("c3d.ros.rmw")) << session_id << ": " << dumped;
    EXPECT_TRUE(properties.contains("c3d.participant.id")) << session_id << ": " << dumped;
    // The operator's own metadata, which is the part with a customer on the other end.
    ASSERT_TRUE(properties.contains("site")) << session_id << ": " << dumped;
    EXPECT_EQ(properties["site"], "warehouse-3") << session_id;
    ASSERT_TRUE(properties.contains("c3d.sessionname")) << session_id << ": " << dumped;
    EXPECT_EQ(properties["c3d.sessionname"], "patrol/run-007") << session_id;
  }

  fs::remove_all(dir, ec);
}

// A JUMP THAT HAPPENED BETWEEN SESSIONS must not close the session just opened.
//
// `OnSampleTimer` returns before polling while no session is active, so between two sessions only
// the slow discovery tick advances the edge detector. A world reset in that gap is therefore first
// OBSERVED after the next session is already open -- and without `StartSession` polling and
// consuming, that session is closed `ros_time_jumped_backwards` about one sample period after the
// caller was handed its id, with the data landing in a replacement id the caller never saw.
//
// This test is what constrains both of those lines: without it, deleting either leaves the suite
// green.
TEST(NodeIdentityProperties, AJumpThatHappenedBetweenSessionsDoesNotCloseTheNextOne) {
  const auto dir = fs::temp_directory_path() / ("c3d_gap_" + std::to_string(::getpid()));
  fs::create_directories(dir);

  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"scene_id", std::string(kSceneId)},
      {"pose_convention", std::string("unity")},
      {"scene_version", std::string("1")},
      {"device_id", std::string("robot")},
      {"spool_dir", dir.string()},
      {"protocol", std::string("http")},
      {"gateway", std::string("127.0.0.1:1")},
      {"auto_start_session", false},
      {"topics", std::vector<std::string>{}},
      {"use_sim_time", true},
  });
  auto node = std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
  auto clock_source = std::make_shared<rclcpp::Node>("c3d_test_gap_clock");
  auto clock_publisher =
      clock_source->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
  auto client = std::make_shared<rclcpp::Node>("c3d_test_gap_client");
  auto start = client->create_client<cognitive3d_ros_msgs::srv::StartSession>(
      "/cognitive3d/start_session");
  auto stop =
      client->create_client<cognitive3d_ros_msgs::srv::EndSession>("/cognitive3d/end_session");

  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(clock_source);
  executor.add_node(client);
  ASSERT_TRUE(start->wait_for_service(std::chrono::seconds(5)));
  ASSERT_TRUE(stop->wait_for_service(std::chrono::seconds(5)));

  const auto drive_clock_to = [&](double seconds) {
    rosgraph_msgs::msg::Clock message;
    message.clock = rclcpp::Time(static_cast<int64_t>(seconds * 1e9));
    for (int i = 0; i < 25; ++i) {
      clock_publisher->publish(message);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      executor.spin_some(std::chrono::milliseconds(10));
    }
  };
  const auto call_start = [&]() {
    auto request = std::make_shared<cognitive3d_ros_msgs::srv::StartSession::Request>();
    auto pending = start->async_send_request(request);
    EXPECT_EQ(executor.spin_until_future_complete(pending, std::chrono::seconds(10)),
              rclcpp::FutureReturnCode::SUCCESS);
    return pending.get()->session_id;
  };

  // One session, so the detector's memory is advanced by a tick that had a session to sample for.
  drive_clock_to(100.0);
  const std::string first = call_start();
  ASSERT_FALSE(first.empty());
  drive_clock_to(100.0);
  auto ending = stop->async_send_request(
      std::make_shared<cognitive3d_ros_msgs::srv::EndSession::Request>());
  ASSERT_EQ(executor.spin_until_future_complete(ending, std::chrono::seconds(10)),
            rclcpp::FutureReturnCode::SUCCESS);

  // The world resets while NOTHING is open. Nobody samples, so nobody observes it.
  drive_clock_to(1.0);

  const std::string second = call_start();
  ASSERT_FALSE(second.empty());
  ASSERT_NE(second, first);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    executor.spin_some(std::chrono::milliseconds(10));
  }

  std::set<std::string> sessions;
  std::error_code ec;
  for (const auto& entry : fs::recursive_directory_iterator(dir, ec)) {
    if (!entry.is_regular_file()) continue;
    std::ifstream in(entry.path(), std::ios::binary);
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    try {
      const auto envelope = nlohmann::json::parse(text);
      if (!envelope.contains("body") || !envelope["body"].is_string()) continue;
      const auto body = nlohmann::json::parse(envelope["body"].get<std::string>());
      if (body.contains("sessionid")) sessions.insert(body["sessionid"].get<std::string>());
    } catch (const nlohmann::json::exception&) {
      continue;
    }
  }
  node->deactivate();
  node->cleanup();
  executor.remove_node(node->get_node_base_interface());
  executor.remove_node(clock_source);
  executor.remove_node(client);
  node.reset();

  // Two, not three. A third means the gap's jump was carried into the second session and closed it.
  EXPECT_EQ(sessions.size(), 2u) << "sessions seen: " << sessions.size()
                                 << "; the jump from the gap closed the session just opened";
  EXPECT_TRUE(sessions.count(second) == 1) << "the session the caller was handed is not in the spool";

  fs::remove_all(dir, ec);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  // Every node here reads its key from the environment; a test that changes it restores it.
  ::setenv("C3D_APPLICATION_API_KEY", "K", 1);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}

// An allowlist entry whose '*' is not the last character is refused at configure.
//
// This test pins that LoadConfig actually refuses: without it, compiling the
// refusal out of LoadConfig leaves the whole suite green.
// TopicPlan.AnInteriorStarIsNotAWildcard pins only the matching SEMANTICS the
// refusal relies on -- and a helper that is tested but never called is exactly
// the shape of the defect being prevented here: an entry that looks configured
// and can never match anything.
TEST(AllowlistPatterns, AnInteriorStarIsRefusedAtConfigure) {
  const fs::path dir = fs::temp_directory_path() /
                       ("c3d_star_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  auto make = [&dir](const std::vector<std::string>& topics) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"scene_id", std::string(kSceneId)},
        {"pose_convention", std::string("unity")},
        {"scene_version", std::string("1")},
        {"device_id", std::string("robot")},
        {"spool_dir", dir.string()},
        {"protocol", std::string("http")},
        {"gateway", std::string("127.0.0.1:1")},
        {"auto_start_session", false},
        {"topics", topics},
    });
    return std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
  };

  EXPECT_NE(make({"/rosbot*/odom"})->configure().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      << "an interior '*' can never match a topic name and is skipped by the "
         "missing-topic check, so configure must refuse it rather than leave it "
         "silently dead while printing in the plan dump as configured";

  // The documented, supported form. If the refusal ever widens to catch this,
  // every namespaced multi-robot config in the field stops configuring.
  EXPECT_EQ(make({"/robot1/*"})->configure().id(),
            lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      << "a trailing wildcard is supported and must not be caught by the refusal";

  fs::remove_all(dir);
}

// The camera and lidar groups are PLUMBED: their nested per-sensor keys are read,
// a refusal reaches configure naming the parameter, and a good configuration reads
// back resolved.
//
// Four nodes, not one per bad value. Every refusal this could enumerate is already
// covered against the pure validators in test_media.cpp, and a node costs a DDS
// participant: fifteen constructed and destroyed in one process push this whole
// binary past its timeout about half the time, taking unrelated tests with it. The
// question a NODE test can answer that a pure one cannot is whether the parameters
// are wired at all.
TEST(NodeParameterWiring, TheCameraAndLidarGroupsAreReadAndRefuseByName) {
  const fs::path dir = fs::temp_directory_path() /
                       ("c3d_media_cfg_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  auto make = [&dir](std::vector<rclcpp::Parameter> extra) {
    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> overrides = {
        {"scene_id", std::string(kSceneId)},
        {"pose_convention", std::string("unity")},
        {"scene_version", std::string("1")},
        {"device_id", std::string("robot")},
        {"spool_dir", dir.string()},
        {"protocol", std::string("http")},
        {"gateway", std::string("127.0.0.1:1")},
        {"auto_start_session", false},
        {"topics", std::vector<std::string>{}},
    };
    for (auto& parameter : extra) overrides.push_back(parameter);
    options.parameter_overrides(overrides);
    return std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
  };

  // Empty lists are the default and must stay a clean configure: a deployment with
  // no cameras or lidars must not have to mention either.
  EXPECT_EQ(make({})->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  // A name in `cameras` with no topic. The refusal must name the parameter path,
  // because the operator's next move is to open the params file at that key.
  auto no_topic = make({{"cameras", std::vector<std::string>{"front"}}});
  EXPECT_NE(no_topic->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_NE(no_topic->configure_error().find("camera.front.topic"), std::string::npos)
      << no_topic->configure_error();

  // A name listed twice. The second would silently overwrite the first's parameters
  // and the run would record one sensor where the file names two -- and it is the
  // one refusal with no pure counterpart, because uniqueness is a property of the
  // LIST rather than of a config.
  auto duplicate = make({{"cameras", std::vector<std::string>{"front", "front"}},
                         {"camera.front.topic", std::string("/image")}});
  EXPECT_NE(duplicate->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_NE(duplicate->configure_error().find("twice"), std::string::npos)
      << duplicate->configure_error();

  // One enum, through the nested `pointcloud.` key: proof the deepest parameter
  // path in the group is read at all, and that an unknown value is refused here
  // rather than collecting a 400 per part for the life of the run.
  auto bad_colour = make({{"lidars", std::vector<std::string>{"depth"}},
                          {"lidar.depth.topic", std::string("/points")},
                          {"lidar.depth.pointcloud.color", std::string("RGB565")}});
  EXPECT_NE(bad_colour->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_NE(bad_colour->configure_error().find("lidar.depth.pointcloud.color"),
            std::string::npos)
      << bad_colour->configure_error();

  // And a whole valid configuration reads back RESOLVED, which is what `doctor`
  // reports from and the only way to see that a nested value survived the trip.
  auto good = make({{"cameras", std::vector<std::string>{"front"}},
                    {"camera.front.topic", std::string("/oakd/rgb/preview/image_raw")},
                    {"camera.front.rate_hz", 1.0},
                    {"camera.front.max_width_px", 640},
                    {"lidars", std::vector<std::string>{"scan", "depth"}},
                    {"lidar.scan.topic", std::string("/scan")},
                    {"lidar.scan.intensities", true},
                    {"lidar.depth.topic", std::string("/oakd/rgb/preview/depth/points")},
                    {"lidar.depth.pointcloud.color", std::string("rgb565")},
                    {"lidar.depth.pointcloud.voxel_leaf_m", 0.1}});
  EXPECT_EQ(good->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      << good->configure_error();
  ASSERT_EQ(good->resolved_cameras().size(), 1u);
  EXPECT_EQ(good->resolved_cameras()[0].topic, "/oakd/rgb/preview/image_raw");
  EXPECT_EQ(good->resolved_cameras()[0].max_width_px, 640);
  ASSERT_EQ(good->resolved_lidars().size(), 2u);
  EXPECT_TRUE(good->resolved_lidars()[0].intensities);
  EXPECT_EQ(good->resolved_lidars()[1].pointcloud.color, c3d::PointColor::kRgb565);
  EXPECT_DOUBLE_EQ(good->resolved_lidars()[1].pointcloud.voxel_leaf_m, 0.1);

  fs::remove_all(dir);
}

// A fixture whose discovery tick is fast enough to observe, with an allowlist that
// stays empty: everything these tests assert on is subscribed WITHOUT an entry.
class DiscoveryByTypeTest : public SessionControlTest {
 protected:
  std::vector<rclcpp::Parameter> ExtraOverrides() override {
    return {{"discovery_period_s", 0.2}};
  }
  void SpinFor(std::chrono::milliseconds total) {
    const auto deadline = std::chrono::steady_clock::now() + total;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some(std::chrono::milliseconds(50));
    }
  }
  bool Subscribed(const std::string& topic) const {
    return node_->topic_stats().count(topic) != 0;
  }
  bool OnGraph(const std::string& topic) {
    for (const auto& entry : node_->get_topic_names_and_types()) {
      if (entry.first == topic) return true;
    }
    return false;
  }
};

TEST_F(DiscoveryByTypeTest, BatteryAndDiagnosticsAreTakenOnTheirTypeAlone) {
  // Their topic NAMES are a deployment detail -- /battery_state, /battery_status
  // and a namespaced /robot1/battery_state are all in use -- while their types
  // are fixed. `topics` is empty here, so an entry is not what reaches them.
  auto diagnostics = client_node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(10));
  auto battery = client_node_->create_publisher<sensor_msgs::msg::BatteryState>(
      "/robot1/battery_state", rclcpp::QoS(10));
  SpinFor(std::chrono::seconds(2));

  ASSERT_TRUE(OnGraph("/diagnostics"));
  ASSERT_TRUE(OnGraph("/robot1/battery_state"));
  EXPECT_TRUE(Subscribed("/diagnostics"));
  EXPECT_TRUE(Subscribed("/robot1/battery_state"));
}

TEST_F(DiscoveryByTypeTest, TheNodeNeverTakesItsOwnHealthReport) {
  // `~/diagnostics` is a DiagnosticArray like any other, and it is the node's own
  // output. Subscribed, every line it says about itself becomes an event about
  // itself, once per report period, for the life of the run.
  SpinFor(std::chrono::seconds(2));
  ASSERT_TRUE(OnGraph("/cognitive3d/diagnostics"))
      << "the topic under test never appeared on the graph, so the assertion below proves nothing";
  EXPECT_FALSE(Subscribed("/cognitive3d/diagnostics"));

  // The same answer read straight off the predicate, because `doctor` reports
  // from THIS rather than from a copy of it -- and `doctor` configures the node,
  // so its own health topic is on the graph at the moment it looks.
  EXPECT_FALSE(node_->AutoSubscribesByType("/cognitive3d/diagnostics",
                                           "diagnostic_msgs/msg/DiagnosticArray"));
  EXPECT_TRUE(node_->AutoSubscribesByType("/robot1/diagnostics",
                                          "diagnostic_msgs/msg/DiagnosticArray"));
}

// `topics` names an action status topic AND refuses it, which is the one way to
// reach a refusal on a topic the name rules would otherwise enrol.
class RefusedActionTopicTest : public DiscoveryByTypeTest {
 protected:
  std::vector<rclcpp::Parameter> ExtraOverrides() override {
    auto overrides = DiscoveryByTypeTest::ExtraOverrides();
    overrides.push_back(
        {"topics", std::vector<std::string>{"/robot1/navigate_to_pose/_action/status"}});
    overrides.push_back(
        {"topic_config.robot1_navigate_to_pose__action_status.ingest", std::string("refuse")});
    return overrides;
  }
};

TEST_F(RefusedActionTopicTest, ARefusedActionTopicIsReportedAsNotRecordedRatherThanEnrolled) {
  // `TopicPlan::Allowed` reports a refused topic as NOT allowlisted, so it falls
  // through to the name rules -- and `navigate_to_pose` is one of the baked-in
  // names. Enrolled by the verdict, subscribed by nothing: the startup line and
  // `doctor` both promise an action that emits not one event.
  auto status = client_node_->create_publisher<action_msgs::msg::GoalStatusArray>(
      "/robot1/navigate_to_pose/_action/status", rclcpp::QoS(10));
  SpinFor(std::chrono::seconds(2));

  ASSERT_TRUE(OnGraph("/robot1/navigate_to_pose/_action/status"));
  const auto& enrolment = node_->action_enrolment();
  const auto it = enrolment.find("/robot1/navigate_to_pose/_action/status");
  ASSERT_NE(it, enrolment.end()) << "every action on the graph is classified, recorded or not";
  EXPECT_FALSE(it->second.enrolled);
  EXPECT_NE(it->second.reason.find("refuse"), std::string::npos) << it->second.reason;
  EXPECT_FALSE(Subscribed("/robot1/navigate_to_pose/_action/status"));
}

TEST_F(DiscoveryByTypeTest, AnActionInTheListIsSubscribedAndOneOutsideItIsNot) {
  auto enrolled = client_node_->create_publisher<action_msgs::msg::GoalStatusArray>(
      "/robot1/navigate_to_pose/_action/status", rclcpp::QoS(10));
  auto absent = client_node_->create_publisher<action_msgs::msg::GoalStatusArray>(
      "/arm/pick_object/_action/status", rclcpp::QoS(10));
  SpinFor(std::chrono::seconds(2));

  ASSERT_TRUE(OnGraph("/robot1/navigate_to_pose/_action/status"));
  ASSERT_TRUE(OnGraph("/arm/pick_object/_action/status"));
  // Matched by NAME regardless of namespace, which is what lets one entry cover a
  // fleet.
  EXPECT_TRUE(Subscribed("/robot1/navigate_to_pose/_action/status"));
  // Not in `action_names`, so it emits nothing at all -- the case `doctor` and the
  // startup line exist to make visible.
  EXPECT_FALSE(Subscribed("/arm/pick_object/_action/status"));
}

// ---------------------------------------------------------------------------
// Uploads: the parameters that decide whether and how the spool is sent.
// ---------------------------------------------------------------------------

namespace {

std::vector<rclcpp::Parameter> UploadTestOverrides(const fs::path& dir, const std::string& gateway) {
  return {
      {"scene_id", std::string(kSceneId)},
      {"pose_convention", std::string("unity")},
      {"scene_version", std::string("1")},
      {"device_id", std::string("robot")},
      {"spool_dir", dir.string()},
      {"protocol", std::string("http")},
      {"gateway", gateway},
      {"topics", std::vector<std::string>{}},
      {"remote_variables", false},
  };
}

std::shared_ptr<cognitive3d_ros::Cognitive3DNode> MakeUploadNode(
    const fs::path& dir, const std::string& gateway, std::vector<rclcpp::Parameter> extra) {
  rclcpp::NodeOptions options;
  auto overrides = UploadTestOverrides(dir, gateway);
  for (auto& parameter : extra) overrides.push_back(parameter);
  options.parameter_overrides(overrides);
  return std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
}

std::size_t SpooledChunks(const fs::path& dir) {
  std::size_t n = 0;
  std::error_code ec;
  for (const auto& entry : fs::directory_iterator(dir / "chunks", ec)) {
    if (entry.path().extension() == ".c3dchunk") ++n;
  }
  return n;
}

bool WaitUntil(const std::function<bool()>& done, double seconds) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (std::chrono::steady_clock::now() < deadline) {
    if (done()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return done();
}

}  // namespace

TEST(NodeParameterWiring, TheUploadParametersReachTheConfigAndARefusalNamesTheParameter) {
  const fs::path dir = fs::temp_directory_path() / ("c3d_upload_cfg_" + std::to_string(::getpid()));
  fs::create_directories(dir);

  // The defaults are c3d::Config's, so a params file that sets none of these
  // changes nothing: uploads are judged by stall, as core judges them, not by a fixed timeout.
  auto defaults = MakeUploadNode(dir, "127.0.0.1:1", {{"auto_start_session", false}});
  ASSERT_EQ(defaults->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      << defaults->configure_error();
  const c3d::Config shipped;
  EXPECT_FALSE(defaults->resolved_config().uploads_paused);
  EXPECT_EQ(defaults->resolved_config().http_connect_timeout_s, shipped.http_connect_timeout_s);
  EXPECT_EQ(defaults->resolved_config().http_timeout_s, shipped.http_timeout_s);
  EXPECT_EQ(defaults->resolved_config().http_low_speed_time_s, shipped.http_low_speed_time_s);
  EXPECT_FALSE(defaults->uploads_paused());
  defaults.reset();

  auto set = MakeUploadNode(dir, "127.0.0.1:1",
                            {{"auto_start_session", false},
                             {"uploads_paused", true},
                             {"http_connect_timeout_s", 7},
                             {"http_low_speed_limit_bytes_per_s", 2048},
                             {"http_low_speed_time_s", 45},
                             {"http_timeout_s", 600},
                             {"retry_min_s", 5.0},
                             {"retry_max_s", 20.0}});
  ASSERT_EQ(set->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      << set->configure_error();
  const c3d::Config& c = set->resolved_config();
  EXPECT_TRUE(c.uploads_paused);
  EXPECT_TRUE(set->uploads_paused()) << "the parameter did not reach the session";
  EXPECT_EQ(c.http_connect_timeout_s, 7);
  EXPECT_EQ(c.http_low_speed_limit_bytes_per_s, 2048);
  EXPECT_EQ(c.http_low_speed_time_s, 45);
  EXPECT_EQ(c.http_timeout_s, 600);
  EXPECT_DOUBLE_EQ(c.retry_min_s, 5.0);
  EXPECT_DOUBLE_EQ(c.retry_max_s, 20.0);
  set.reset();

  const std::vector<std::pair<rclcpp::Parameter, std::string>> refused = {
      {{"http_connect_timeout_s", 0}, "http_connect_timeout_s"},
      {{"http_low_speed_limit_bytes_per_s", -5}, "http_low_speed_limit_bytes_per_s"},
      {{"http_low_speed_time_s", 0}, "http_low_speed_time_s"},
      // Past INT_MAX: must be refused, not wrapped into something plausible.
      {{"http_timeout_s", static_cast<int64_t>(1) << 32}, "http_timeout_s"},
      {{"retry_min_s", 0.0}, "retry window"},
  };
  for (const auto& [parameter, name] : refused) {
    auto node = MakeUploadNode(dir, "127.0.0.1:1", {{"auto_start_session", false}, parameter});
    EXPECT_NE(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        << parameter.get_name() << " was accepted";
    EXPECT_NE(node->configure_error().find(name), std::string::npos)
        << "the refusal must name " << name << ": " << node->configure_error();
  }
  std::error_code ec;
  fs::remove_all(dir, ec);
}

// The long-session procedure end to end, through the node: record with uploads
// paused, then a node that is configured and NEVER activated drains the spool once
// uploads are unpaused -- here by `ros2 param set`, the runtime path.
TEST(NodeUploadsPaused, APausedRunSendsNothingAndAConfiguredNodeDrainsItWhenUnpaused) {
  const fs::path dir = fs::temp_directory_path() / ("c3d_paused_node_" + std::to_string(::getpid()));
  fs::remove_all(dir);
  fs::create_directories(dir);
  c3d::testing_support::ThrottledHttpServer server({});

  {
    auto recorder = MakeUploadNode(dir, server.host(), {{"uploads_paused", true}});
    ASSERT_EQ(recorder->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        << recorder->configure_error();
    ASSERT_EQ(recorder->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    recorder->deactivate();
    recorder->cleanup();
  }
  const std::size_t spooled = SpooledChunks(dir);
  ASSERT_GE(spooled, 2u) << "the paused session spooled nothing; nothing to drain";
  EXPECT_EQ(server.completed(), 0u) << "a paused node uploaded";

  auto drainer = MakeUploadNode(dir, server.host(), {{"uploads_paused", true}});
  ASSERT_EQ(drainer->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(server.completed(), 0u) << "configure with uploads paused must not drain";

  const auto result = drainer->set_parameter(rclcpp::Parameter("uploads_paused", false));
  EXPECT_TRUE(result.successful) << result.reason;
  EXPECT_FALSE(drainer->uploads_paused()) << "ros2 param set did not reach the session";
  EXPECT_TRUE(WaitUntil([&] { return SpooledChunks(dir) == 0; }, 10.0))
      << SpooledChunks(dir) << " chunk(s) left after unpausing a configured node";
  EXPECT_EQ(server.completed(), spooled);

  // And back: pausing at runtime holds new data.
  EXPECT_TRUE(drainer->set_parameter(rclcpp::Parameter("uploads_paused", true)).successful);
  EXPECT_TRUE(drainer->uploads_paused());
  drainer->cleanup();
  std::error_code ec;
  fs::remove_all(dir, ec);
}

namespace {

// Every spooled JSON body, parsed. Uploads fail against a dead port, so the spool is the wire.
std::vector<nlohmann::json> SpooledBodies(const fs::path& dir) {
  std::vector<nlohmann::json> bodies;
  std::error_code ec;
  for (const auto& entry : fs::recursive_directory_iterator(dir, ec)) {
    if (!entry.is_regular_file()) continue;
    std::ifstream in(entry.path(), std::ios::binary);
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    try {
      const auto envelope = nlohmann::json::parse(text);
      if (!envelope.contains("body") || !envelope["body"].is_string()) continue;
      bodies.push_back(nlohmann::json::parse(envelope["body"].get<std::string>()));
    } catch (const nlohmann::json::exception&) {
      continue;
    }
  }
  return bodies;
}

nlohmann::json UnionOfProperties(const std::vector<nlohmann::json>& bodies) {
  nlohmann::json sent = nlohmann::json::object();
  for (const auto& body : bodies) {
    if (!body.contains("properties")) continue;
    for (const auto& [key, value] : body["properties"].items()) sent[key] = value;
  }
  return sent;
}

std::vector<rclcpp::Parameter> OfflineSessionOverrides(const fs::path& dir) {
  return {
      {"scene_id", std::string(kSceneId)},
      {"pose_convention", std::string("unity")},
      {"scene_version", std::string("1")},
      {"device_id", std::string("robot")},
      {"spool_dir", dir.string()},
      {"protocol", std::string("http")},
      {"gateway", std::string("127.0.0.1:1")},
      {"topics", std::vector<std::string>{}},
  };
}

}  // namespace

TEST(SimTimeOnWire, IsRefusedAtConfigureWithoutUseSimTime) {
  const auto dir = fs::temp_directory_path() / ("c3d_simwire_cfg_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  const auto make = [&](bool use_sim_time) {
    rclcpp::NodeOptions options;
    auto overrides = OfflineSessionOverrides(dir);
    overrides.push_back({"sim_time_on_wire", true});
    overrides.push_back({"use_sim_time", use_sim_time});
    options.parameter_overrides(overrides);
    return std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
  };

  auto real_clock = make(false);
  EXPECT_NE(real_clock->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_NE(real_clock->configure_error().find("sim_time_on_wire"), std::string::npos)
      << real_clock->configure_error();
  EXPECT_NE(real_clock->configure_error().find("use_sim_time"), std::string::npos)
      << real_clock->configure_error();

  auto sim_clock = make(true);
  EXPECT_EQ(sim_clock->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      << sim_clock->configure_error();

  std::error_code ec;
  fs::remove_all(dir, ec);
}

namespace {

// What one frozen-clock run put on the wire, for the one session under test.
struct FrozenRun {
  double session_start = 0.0;
  double session_end = 0.0;
  double session_length = -1.0;
  double probe = 0.0;
  double remote_variables_unavailable = 0.0;
  // Whether the session timestamp was pushed ahead of wall clock by a same-second id. Only then
  // is an anchor at wall now distinguishable from an anchor at the session's own timestamp.
  bool nudged = false;
  std::vector<double> gaze;
  std::vector<double> battery;
  nlohmann::json properties;
};

double WallNow() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch())
      .count();
}

FrozenRun RunUnderAFrozenClock(bool sim_time_on_wire, const std::string& tag) {
  const auto dir =
      fs::temp_directory_path() / ("c3d_simwire_" + tag + "_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  rclcpp::NodeOptions options;
  auto overrides = OfflineSessionOverrides(dir);
  overrides.push_back({"auto_start_session", false});
  overrides.push_back({"use_sim_time", true});
  overrides.push_back({"discovery_period_s", 0.2});
  if (sim_time_on_wire) overrides.push_back({"sim_time_on_wire", true});
  options.parameter_overrides(overrides);

  auto node = std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
  auto client = std::make_shared<rclcpp::Node>("c3d_test_simwire_client_" + tag);
  auto start = client->create_client<StartSession>("/cognitive3d/start_session");
  auto end = client->create_client<EndSession>("/cognitive3d/end_session");
  auto events =
      client->create_publisher<Event>("/cognitive3d/events", rclcpp::QoS(100).reliable());
  auto battery = client->create_publisher<sensor_msgs::msg::BatteryState>(
      "/battery_state", rclcpp::QoS(10).reliable());
  auto clock_publisher =
      client->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
  tf2_ros::StaticTransformBroadcaster tf(client);
  geometry_msgs::msg::TransformStamped base;
  base.header.frame_id = "map";
  base.child_frame_id = "base_link";
  base.transform.rotation.w = 1.0;
  tf.sendTransform(base);

  EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(client);

  rosgraph_msgs::msg::Clock frozen;
  frozen.clock = rclcpp::Time(static_cast<int64_t>(100e9));
  const auto spin_wall = [&](std::chrono::milliseconds total) {
    const auto deadline = std::chrono::steady_clock::now() + total;
    while (std::chrono::steady_clock::now() < deadline) {
      clock_publisher->publish(frozen);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      executor.spin_some(std::chrono::milliseconds(10));
    }
  };
  for (const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
       std::chrono::steady_clock::now() < deadline &&
       std::abs(node->get_clock()->now().seconds() - 100.0) > 1e-6;) {
    spin_wall(std::chrono::milliseconds(100));
  }
  EXPECT_NEAR(node->get_clock()->now().seconds(), 100.0, 1e-6)
      << "/clock never reached the node under test";
  EXPECT_TRUE(start->wait_for_service(std::chrono::seconds(5)));
  EXPECT_TRUE(end->wait_for_service(std::chrono::seconds(5)));

  FrozenRun run;
  std::string session_id;
  // Off, one session. On, sessions are opened back to back until one lands in the same second
  // as the last and is pushed ahead of wall clock; that is the one recorded against.
  for (int attempt = 0; attempt < (sim_time_on_wire ? 8 : 1); ++attempt) {
    if (!session_id.empty()) {
      auto pending = end->async_send_request(std::make_shared<EndSession::Request>());
      EXPECT_EQ(executor.spin_until_future_complete(pending, std::chrono::seconds(10)),
                rclcpp::FutureReturnCode::SUCCESS);
    }
    auto request = std::make_shared<StartSession::Request>();
    request->session_name = "frozen";
    auto pending = start->async_send_request(request);
    EXPECT_EQ(executor.spin_until_future_complete(pending, std::chrono::seconds(10)),
              rclcpp::FutureReturnCode::SUCCESS);
    const auto response = pending.get();
    EXPECT_TRUE(response->ok) << response->message;
    session_id = response->session_id;
    run.nudged = std::stod(session_id.substr(0, session_id.find('_'))) > WallNow();
    if (run.nudged) break;
  }

  // Five events empty the ~/events budget at the default 5 per second. The probe a second and a
  // half later passes only if that budget refills on real time.
  for (int i = 0; i < 5; ++i) {
    Event burst;
    burst.name = "burst";
    events->publish(burst);
  }
  spin_wall(std::chrono::milliseconds(1500));
  Event event;
  event.name = "probe";
  events->publish(event);
  sensor_msgs::msg::BatteryState state;
  state.percentage = 0.5f;
  battery->publish(state);
  spin_wall(std::chrono::milliseconds(500));
  node->deactivate();
  node->cleanup();

  std::vector<nlohmann::json> bodies;
  for (auto& body : SpooledBodies(dir)) {
    if (body.value("sessionid", std::string()) == session_id) bodies.push_back(std::move(body));
  }
  for (const auto& body : bodies) {
    if (!body.contains("data") || !body["data"].is_array()) continue;
    const bool is_gaze = body.contains("interval");
    for (const auto& record : body["data"]) {
      if (is_gaze && record.contains("time")) run.gaze.push_back(record["time"].get<double>());
      if (record.value("name", std::string()) == "battery.percent" && record.contains("data")) {
        for (const auto& point : record["data"]) run.battery.push_back(point[0].get<double>());
      }
      if (!record.contains("name") || !record.contains("time")) continue;
      const double time = record["time"].get<double>();
      if (record["name"] == "c3d.sessionStart") run.session_start = time;
      if (record["name"] == "probe") run.probe = time;
      if (record["name"] == "c3d.remote_variables.unavailable") {
        run.remote_variables_unavailable = time;
      }
      if (record["name"] == "c3d.sessionEnd") {
        run.session_end = time;
        run.session_length = record["properties"]["sessionlength"].get<double>();
      }
    }
  }
  // Spool order is directory order; the pairing is by time, not by file.
  std::sort(run.gaze.begin(), run.gaze.end());
  run.properties = UnionOfProperties(bodies);
  executor.remove_node(node->get_node_base_interface());
  executor.remove_node(client);
  node.reset();
  std::error_code ec;
  fs::remove_all(dir, ec);
  return run;
}

}  // namespace

// The node wiring, end to end: /clock is held still while wall time passes, which is a real-time
// factor of zero. Anchored, every record the session carries lands on the session's start -- the
// gaze channel, an event, a translated message, the node's own events and the session end -- and
// the wall-timed sample timer puts exactly one pose on the wire. On wall clock each of them lands
// where wall time put it. Asserted both ways round, and the flag is recorded in both positions.
TEST(SimTimeOnWire, AnEventUnderAFrozenClockLandsOnTheSessionStart) {
  const FrozenRun on = RunUnderAFrozenClock(true, "on");
  ASSERT_GT(on.session_start, 1577836800.0) << "the session start is wall clock under either value";
  EXPECT_TRUE(on.nudged) << "no session id was pushed ahead of wall clock, so the anchor's "
                            "source is not under test";
  ASSERT_GT(on.probe, 0.0) << "the probe never reached the spool: the event cap did not refill "
                              "on real time";
  EXPECT_NEAR(on.probe, on.session_start, 0.1)
      << "no ROS time passed, so no wire time may pass either";
  ASSERT_EQ(on.gaze.size(), 1u) << "a frozen clock holds the pose rate at one sample";
  EXPECT_NEAR(on.gaze.front(), on.session_start, 0.1);
  ASSERT_FALSE(on.battery.empty()) << "the battery message never reached the spool";
  for (const double t : on.battery) EXPECT_NEAR(t, on.session_start, 0.1);
  ASSERT_GT(on.remote_variables_unavailable, 0.0);
  EXPECT_NEAR(on.remote_variables_unavailable, on.session_start, 0.1);
  EXPECT_NEAR(on.session_end, on.session_start, 0.1);
  EXPECT_NEAR(on.session_length, 0.0, 0.1);
  ASSERT_TRUE(on.properties.contains("c3d.ros.sim_time_on_wire")) << on.properties.dump();
  EXPECT_EQ(on.properties["c3d.ros.sim_time_on_wire"], true);

  const FrozenRun off = RunUnderAFrozenClock(false, "off");
  ASSERT_GT(off.session_start, 1577836800.0);
  ASSERT_GT(off.probe, 0.0) << "the probe event never reached the spool";
  EXPECT_GT(off.probe - off.session_start, 1.0) << "off, the wire is wall clock, as it always was";
  EXPECT_GT(off.gaze.size(), 10u) << "off, the sample timer's rate is the wire's";
  ASSERT_FALSE(off.battery.empty());
  EXPECT_GT(off.battery.front() - off.session_start, 1.0);
  EXPECT_GT(off.session_length, 1.0);
  EXPECT_GT(off.session_end - off.session_start, 1.0);
  ASSERT_TRUE(off.properties.contains("c3d.ros.sim_time_on_wire")) << off.properties.dump();
  EXPECT_EQ(off.properties["c3d.ros.sim_time_on_wire"], false);
}

void ExpectMediaProperties(const nlohmann::json& sent);

// The session describes its own media configuration, with the value in force after defaults.
// Every name is pinned, and each value is checked at its type: a rate that reached the wire as a
// string or a width as a double is a different property to anything reading it. A lidar records
// the half of its configuration its message type uses, once the topic is subscribed and the
// type is known, and not the other half.
TEST(NodeMediaProperties, EveryConfiguredCameraAndLidarRecordsTheConfigInForce) {
  const auto dir = fs::temp_directory_path() / ("c3d_mediaprops_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  rclcpp::NodeOptions options;
  auto overrides = OfflineSessionOverrides(dir);
  overrides.push_back({"auto_start_session", true});
  overrides.push_back({"cameras", std::vector<std::string>{"front", "rear", "side"}});
  overrides.push_back({"camera.front.topic", std::string("/image")});
  overrides.push_back({"camera.front.rate_hz", 2.5});
  overrides.push_back({"camera.rear.topic", std::string("/rear/image")});
  // A whole number, as a params file writes it: an integer to rclcpp, taken as the double.
  overrides.push_back({"camera.rear.hfov_degrees", 61});
  overrides.push_back({"camera.rear.vfov_degrees", 45.5});
  overrides.push_back({"camera.side.topic", std::string("/side/image_raw/compressed")});
  overrides.push_back({"camera.side.camera_info_topic", std::string("")});
  overrides.push_back({"lidars", std::vector<std::string>{"scan", "depth"}});
  overrides.push_back({"lidar.scan.topic", std::string("/scan")});
  overrides.push_back({"lidar.scan.intensities", true});
  overrides.push_back({"lidar.depth.topic", std::string("/points")});
  overrides.push_back({"lidar.depth.pointcloud.color", std::string("rgb565")});
  overrides.push_back({"lidar.depth.pointcloud.max_points", 5000});
  overrides.push_back({"discovery_period_s", 0.2});
  options.parameter_overrides(overrides);

  auto node = std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
  auto publishers = std::make_shared<rclcpp::Node>("c3d_test_mediaprops_publishers");
  auto scan = publishers->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
  auto points = publishers->create_publisher<sensor_msgs::msg::PointCloud2>("/points", 10);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(publishers);
  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      << node->configure_error();
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  const auto spin = [&](std::chrono::milliseconds total) {
    const auto deadline = std::chrono::steady_clock::now() + total;
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some(std::chrono::milliseconds(20));
    }
  };
  spin(std::chrono::milliseconds(1500));
  std::string first_id;
  std::string reopened_id;
  {
    auto end = publishers->create_client<EndSession>("/cognitive3d/end_session");
    auto start = publishers->create_client<StartSession>("/cognitive3d/start_session");
    ASSERT_TRUE(end->wait_for_service(std::chrono::seconds(5)));
    ASSERT_TRUE(start->wait_for_service(std::chrono::seconds(5)));
    auto ended = end->async_send_request(std::make_shared<EndSession::Request>());
    ASSERT_EQ(executor.spin_until_future_complete(ended, std::chrono::seconds(10)),
              rclcpp::FutureReturnCode::SUCCESS);
    first_id = ended.get()->session_id;
    auto started = start->async_send_request(std::make_shared<StartSession::Request>());
    ASSERT_EQ(executor.spin_until_future_complete(started, std::chrono::seconds(10)),
              rclcpp::FutureReturnCode::SUCCESS);
    const auto response = started.get();
    ASSERT_TRUE(response->ok) << response->message;
    reopened_id = response->session_id;
  }
  spin(std::chrono::milliseconds(300));
  node->deactivate();
  node->cleanup();
  const auto bodies = SpooledBodies(dir);
  const auto of_session = [&bodies](const std::string& id) {
    std::vector<nlohmann::json> out;
    for (const auto& body : bodies) {
      if (body.value("sessionid", std::string()) == id) out.push_back(body);
    }
    return UnionOfProperties(out);
  };
  // The first session opened before discovery ran, so its lidar halves were written mid-session;
  // the reopened one found the subscriptions already in place.
  for (const std::string& id : {first_id, reopened_id}) {
    SCOPED_TRACE("session " + id);
    ExpectMediaProperties(of_session(id));
  }
  executor.remove_node(node->get_node_base_interface());
  executor.remove_node(publishers);
  node.reset();
  std::error_code ec;
  fs::remove_all(dir, ec);
}

void ExpectMediaProperties(const nlohmann::json& sent) {
  const std::string dumped = sent.dump();
  const auto expect = [&](const std::string& key, const nlohmann::json& value) {
    ASSERT_TRUE(sent.contains(key)) << key << " missing from " << dumped;
    const auto kind = [](const nlohmann::json& v) {
      return v.is_boolean() ? "bool" : v.is_number_integer() ? "int"
             : v.is_number_float() ? "double" : v.is_string() ? "string" : "other";
    };
    EXPECT_STREQ(kind(sent[key]), kind(value)) << key << " = " << sent[key].dump();
    EXPECT_EQ(sent[key], value) << key;
  };
  expect("c3d.ros.camera.front.topic", "/image");
  expect("c3d.ros.camera.front.rate_hertz", 2.5);
  expect("c3d.ros.camera.front.max_width_pixels", 640);
  expect("c3d.ros.camera.front.jpeg_quality", 80);
  expect("c3d.ros.camera.front.format", "jpeg");
  // The field of view: from the image topic's sibling CameraInfo by default, from the parameters
  // when set, and from nothing with the CameraInfo switched off. A double either way, 0 unset.
  expect("c3d.ros.camera.front.hfov_degrees", 0.0);
  expect("c3d.ros.camera.front.vfov_degrees", 0.0);
  expect("c3d.ros.camera.front.fov_source", "camera_info");
  expect("c3d.ros.camera.front.camera_info_topic", "/camera_info");
  expect("c3d.ros.camera.rear.hfov_degrees", 61.0);
  expect("c3d.ros.camera.rear.vfov_degrees", 45.5);
  expect("c3d.ros.camera.rear.fov_source", "parameter");
  expect("c3d.ros.camera.rear.camera_info_topic", "/rear/camera_info");
  expect("c3d.ros.camera.side.fov_source", "none");
  expect("c3d.ros.camera.side.camera_info_topic", "");
  expect("c3d.ros.cameras", "front,rear,side");

  expect("c3d.ros.lidar.scan.topic", "/scan");
  expect("c3d.ros.lidar.scan.rate_hertz", 2.0);
  expect("c3d.ros.lidar.scan.message_type", "sensor_msgs/msg/LaserScan");
  expect("c3d.ros.lidar.scan.intensities", true);
  expect("c3d.ros.lidar.depth.topic", "/points");
  expect("c3d.ros.lidar.depth.rate_hertz", 2.0);
  expect("c3d.ros.lidar.depth.message_type", "sensor_msgs/msg/PointCloud2");
  expect("c3d.ros.lidar.depth.pointcloud.voxel_leaf_meters", 0.05);
  expect("c3d.ros.lidar.depth.pointcloud.max_points", 5000);
  expect("c3d.ros.lidar.depth.pointcloud.position", "int16_mm");
  expect("c3d.ros.lidar.depth.pointcloud.color", "rgb565");
  expect("c3d.ros.lidar.depth.pointcloud.intensity", "none");
  expect("c3d.ros.lidar.depth.pointcloud.zstd_level", 1);

  EXPECT_FALSE(sent.contains("c3d.ros.lidar.depth.intensities")) << dumped;
  for (const std::string key : {"voxel_leaf_meters", "max_points", "position", "color",
                                "intensity", "zstd_level"}) {
    EXPECT_FALSE(sent.contains("c3d.ros.lidar.scan.pointcloud." + key)) << key << " in " << dumped;
  }
}

namespace {

// The largest value any sample of the sensor series `name` carries in `bodies`, or -1 when no
// body has the series.
double SeriesMaximum(const std::vector<nlohmann::json>& bodies, const std::string& name) {
  double maximum = -1.0;
  for (const auto& body : bodies) {
    if (!body.contains("data") || !body["data"].is_array()) continue;
    for (const auto& series : body["data"]) {
      if (!series.is_object() || series.value("name", std::string()) != name) continue;
      if (!series.contains("data") || !series["data"].is_array()) continue;
      maximum = std::max(maximum, 0.0);
      for (const auto& point : series["data"]) {
        if (point.is_array() && point.size() == 2 && point[1].is_number()) {
          maximum = std::max(maximum, point[1].get<double>());
        }
      }
    }
  }
  return maximum;
}

}  // namespace

// The gaze channel's frame is recorded under names that echo its parameters: the frame, the
// convention in force once `auto` is resolved, and the series counting the samples whose frame did
// not resolve. The frame here is not in the tree, so every sample falls back to the body and is
// counted, and `auto` reads the name as optical; the second run sets the convention, so the
// property must follow the parameter rather than the name.
TEST(GazeFrame, TheFrameItsConventionAndItsFailuresAreRecordedUnderTheGazeFrameNames) {
  const auto run = [](const std::string& tag, const std::vector<rclcpp::Parameter>& extra,
                      std::chrono::milliseconds active_for) {
    const auto dir = fs::temp_directory_path() /
                     ("c3d_gazeframe_" + tag + "_" + std::to_string(::getpid()));
    fs::create_directories(dir);
    rclcpp::NodeOptions options;
    auto overrides = OfflineSessionOverrides(dir);
    overrides.push_back({"auto_start_session", true});
    overrides.push_back({"remote_variables", false});
    overrides.push_back({"gaze_frame", std::string("head_optical_frame")});
    for (const auto& parameter : extra) overrides.push_back(parameter);
    options.parameter_overrides(overrides);

    auto node = std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
    auto tf_node = std::make_shared<rclcpp::Node>("c3d_test_gazeframe_" + tag);
    tf2_ros::StaticTransformBroadcaster broadcaster(tf_node);
    geometry_msgs::msg::TransformStamped body;
    body.header.frame_id = "map";
    body.child_frame_id = "base_link";
    body.transform.rotation.w = 1.0;
    broadcaster.sendTransform(body);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.add_node(tf_node);
    EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        << node->configure_error();
    EXPECT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    const auto deadline = std::chrono::steady_clock::now() + active_for;
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some(std::chrono::milliseconds(20));
    }
    node->deactivate();
    node->cleanup();
    executor.remove_node(node->get_node_base_interface());
    executor.remove_node(tf_node);
    node.reset();
    auto bodies = SpooledBodies(dir);
    std::error_code ec;
    fs::remove_all(dir, ec);
    return bodies;
  };

  // Past the first self-telemetry record, which can predate the first pose, to the next one.
  const auto inferred = run("auto", {}, std::chrono::milliseconds(6500));
  const auto sent = UnionOfProperties(inferred);
  ASSERT_TRUE(sent.contains("c3d.ros.gaze_frame")) << sent.dump();
  EXPECT_EQ(sent["c3d.ros.gaze_frame"], "head_optical_frame");
  ASSERT_TRUE(sent.contains("c3d.ros.gaze_frame_convention")) << sent.dump();
  EXPECT_EQ(sent["c3d.ros.gaze_frame_convention"], "optical_forward_z")
      << "auto must be recorded as the convention it resolved to";
  EXPECT_GT(SeriesMaximum(inferred, "c3d.gaze_frame_failures"), 0.0)
      << "every sample's frame failed to resolve, and the series must say so";

  const auto explicit_convention =
      UnionOfProperties(run("explicit", {{"gaze_frame_convention", std::string("body_forward_x")}},
                            std::chrono::milliseconds(1000)));
  ASSERT_TRUE(explicit_convention.contains("c3d.ros.gaze_frame_convention"))
      << explicit_convention.dump();
  EXPECT_EQ(explicit_convention["c3d.ros.gaze_frame_convention"], "body_forward_x")
      << "the parameter did not reach the session";

  const auto dir = fs::temp_directory_path() / ("c3d_gazeframe_bad_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  const std::string refused = ConfigureError(dir, {{"gaze_frame_convention", std::string("z")}});
  EXPECT_EQ(refused.rfind("gaze_frame_convention must be", 0), 0u) << refused;
  std::error_code ec;
  fs::remove_all(dir, ec);
}

// The node gates every camera frame and every translated message on when it ARRIVED, by its own
// clock. A camera whose driver never sets the stamp therefore records at its configured rate
// rather than one frame per session, and an allowlisted odometry topic at its type's 30 Hz.
TEST(NodeRateGates, AZeroStampedCameraAndAnOdometryTopicAreGatedOnArrival) {
  const auto dir = fs::temp_directory_path() / ("c3d_arrival_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  rclcpp::NodeOptions options;
  auto overrides = OfflineSessionOverrides(dir);
  overrides.erase(std::remove_if(overrides.begin(), overrides.end(),
                                 [](const rclcpp::Parameter& p) { return p.get_name() == "topics"; }),
                  overrides.end());
  overrides.push_back({"auto_start_session", true});
  overrides.push_back({"remote_variables", false});
  overrides.push_back({"reference_frame", std::string("odom")});
  overrides.push_back({"fallback_reference_frame", std::string("")});
  overrides.push_back({"discovery_period_s", 0.2});
  overrides.push_back({"topics", std::vector<std::string>{"/probe/odom"}});
  overrides.push_back({"cameras", std::vector<std::string>{"front"}});
  overrides.push_back({"camera.front.topic", std::string("/probe/image")});
  overrides.push_back({"camera.front.rate_hz", 2.0});
  options.parameter_overrides(overrides);
  auto node = std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);

  auto publishers = std::make_shared<rclcpp::Node>("c3d_test_arrival_publishers");
  auto image = publishers->create_publisher<sensor_msgs::msg::Image>("/probe/image", 10);
  auto odometry = publishers->create_publisher<nav_msgs::msg::Odometry>("/probe/odom", 10);
  tf2_ros::StaticTransformBroadcaster mounts(publishers);
  {
    geometry_msgs::msg::TransformStamped base;
    base.header.stamp = publishers->now();
    base.header.frame_id = "odom";
    base.child_frame_id = "base_link";
    base.transform.rotation.w = 1.0;
    geometry_msgs::msg::TransformStamped camera = base;
    camera.header.frame_id = "base_link";
    camera.child_frame_id = "camera_link";
    camera.transform.translation.z = 0.3;
    mounts.sendTransform({base, camera});
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(publishers);
  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      << node->configure_error();
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // Past the five-second mark the self-telemetry records the camera's frame count at.
  sensor_msgs::msg::Image frame;
  frame.header.frame_id = "camera_link";  // and the stamp left at zero
  frame.width = 8;
  frame.height = 8;
  frame.encoding = "rgb8";
  frame.step = 24;
  frame.data.assign(8 * 8 * 3, 64);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(6000);
  auto next_frame = std::chrono::steady_clock::now();
  auto next_odometry = next_frame;
  int published = 0;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= next_frame) {
      image->publish(frame);
      next_frame += std::chrono::milliseconds(50);
    }
    if (now >= next_odometry) {
      nav_msgs::msg::Odometry speed;  // a new value each time: sensors are change-gated
      speed.twist.twist.linear.x = 0.01 * published++;
      odometry->publish(speed);
      next_odometry += std::chrono::milliseconds(10);
    }
    executor.spin_some(std::chrono::milliseconds(5));
  }
  ASSERT_EQ(node->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  executor.remove_node(node->get_node_base_interface());
  executor.remove_node(publishers);

  const auto bodies = SpooledBodies(dir);
  const double frames = SeriesMaximum(bodies, "c3d.camera.front.frames");
  EXPECT_GE(frames, 4.0) << "a zero-stamped camera at 20 Hz into a 2 Hz gate froze";
  EXPECT_LE(frames, 13.0) << "a 2 Hz gate over about five seconds";
  std::size_t speeds = 0;
  for (const auto& body : bodies) {
    if (!body.contains("data") || !body["data"].is_array()) continue;
    for (const auto& series : body["data"]) {
      if (series.is_object() && series.value("name", std::string()) == "odom.speed") {
        speeds += series.value("data", nlohmann::json::array()).size();
      }
    }
  }
  EXPECT_GE(speeds, 90u) << "odometry at 100 Hz into its type's 30 Hz gate";
  EXPECT_LE(speeds, 186u) << "a 30 Hz gate over about six seconds";
  std::error_code ec;
  fs::remove_all(dir, ec);
}

// Under sim_time_on_wire the session's timeline is simulated time, and the gates run on it: a
// simulation at a quarter of real time delivers `rate_hz` per simulated second, not per wall one.
TEST(NodeRateGates, UnderSimTimeOnWireTheRateIsPerSimulatedSecond) {
  const auto dir = fs::temp_directory_path() / ("c3d_simgate_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  rclcpp::NodeOptions options;
  auto overrides = OfflineSessionOverrides(dir);
  overrides.erase(std::remove_if(overrides.begin(), overrides.end(),
                                 [](const rclcpp::Parameter& p) { return p.get_name() == "topics"; }),
                  overrides.end());
  overrides.push_back({"auto_start_session", true});
  overrides.push_back({"remote_variables", false});
  overrides.push_back({"use_sim_time", true});
  overrides.push_back({"sim_time_on_wire", true});
  overrides.push_back({"discovery_period_s", 0.2});
  overrides.push_back({"topics", std::vector<std::string>{"/probe/odom"}});
  options.parameter_overrides(overrides);
  auto node = std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
  auto publishers = std::make_shared<rclcpp::Node>("c3d_test_simgate_publishers");
  auto odometry = publishers->create_publisher<nav_msgs::msg::Odometry>("/probe/odom", 10);
  auto clock = publishers->create_publisher<rosgraph_msgs::msg::Clock>("/clock",
                                                                      rclcpp::ClockQoS());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(publishers);

  const auto wall_start = std::chrono::steady_clock::now();
  const auto tick = [&]() {
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
    rosgraph_msgs::msg::Clock now;
    now.clock = rclcpp::Time(static_cast<int64_t>((100.0 + 0.25 * elapsed) * 1e9));
    clock->publish(now);
  };
  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      << node->configure_error();
  const auto clock_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (node->get_clock()->now().seconds() < 99.0 &&
         std::chrono::steady_clock::now() < clock_deadline) {
    tick();
    executor.spin_some(std::chrono::milliseconds(10));
  }
  ASSERT_GT(node->get_clock()->now().seconds(), 99.0) << "/clock never reached the node";
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // Four wall seconds at 40 Hz: one simulated second.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(4000);
  auto next = std::chrono::steady_clock::now();
  int published = 0;
  while (std::chrono::steady_clock::now() < deadline) {
    if (std::chrono::steady_clock::now() >= next) {
      tick();
      nav_msgs::msg::Odometry speed;
      speed.twist.twist.linear.x = 0.01 * published++;
      odometry->publish(speed);
      next += std::chrono::milliseconds(25);
    }
    executor.spin_some(std::chrono::milliseconds(5));
  }
  ASSERT_EQ(node->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  executor.remove_node(node->get_node_base_interface());
  executor.remove_node(publishers);

  std::size_t speeds = 0;
  for (const auto& body : SpooledBodies(dir)) {
    if (!body.contains("data") || !body["data"].is_array()) continue;
    for (const auto& series : body["data"]) {
      if (series.is_object() && series.value("name", std::string()) == "odom.speed") {
        speeds += series.value("data", nlohmann::json::array()).size();
      }
    }
  }
  EXPECT_GE(speeds, 15u) << "the odometry topic was barely recorded";
  EXPECT_LE(speeds, 40u) << "30 Hz per simulated second, over about one simulated second";
  std::error_code ec;
  fs::remove_all(dir, ec);
}

namespace {

// Wire times carry five decimal places; the margin is ten times that resolution.
constexpr double kWireResolution = 1e-4;

// The wire time of every event named `name` in the spool under `dir`.
std::vector<double> EventTimes(const fs::path& dir, const std::string& name) {
  std::vector<double> times;
  for (const auto& body : SpooledBodies(dir)) {
    if (!body.contains("data") || !body["data"].is_array() || body.contains("interval")) continue;
    for (const auto& record : body["data"]) {
      if (record.value("name", std::string()) == name && record.contains("time")) {
        times.push_back(record["time"].get<double>());
      }
    }
  }
  return times;
}

}  // namespace

// An event can carry its own time, and zero means now. The stamp is placed like a header stamp: as
// an age against the node's ROS clock.
TEST_F(LifecycleContractTest, AStampedEventIsRecordedAtItsOwnTime) {
  MakeNode(/*auto_start_session=*/true);
  auto events =
      client_->create_publisher<Event>("/cognitive3d/events", rclcpp::QoS(100).reliable());
  ASSERT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_TRUE(SpinUntil([&] { return events->get_subscription_count() > 0; }));
  // Long enough into the session that the stamp below falls inside it.
  SpinUntil([] { return false; }, std::chrono::milliseconds(3000));

  const double before = WallNow();
  Event stamped;
  stamped.name = "stamped";
  stamped.stamp = rclcpp::Time(node_->get_clock()->now().nanoseconds() - 2'000'000'000LL);
  Event unstamped;
  unstamped.name = "unstamped";
  events->publish(stamped);
  events->publish(unstamped);
  ASSERT_TRUE(events->wait_for_all_acked(std::chrono::seconds(5)));
  SpinUntil([] { return false; }, std::chrono::milliseconds(500));
  const double after = WallNow();
  ASSERT_EQ(node_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  const auto start = EventTimes(dir_, "c3d.sessionStart");
  const auto stamped_times = EventTimes(dir_, "stamped");
  const auto unstamped_times = EventTimes(dir_, "unstamped");
  ASSERT_EQ(start.size(), 1u);
  ASSERT_EQ(stamped_times.size(), 1u);
  ASSERT_EQ(unstamped_times.size(), 1u);
  // The unstamped one arrived between the stamp and the end of the spin; the stamped one sits two
  // seconds before its stamp was taken, however long it took to arrive.
  const double apart = unstamped_times[0] - stamped_times[0];
  EXPECT_GE(apart, 2.0 - kWireResolution)
      << "the stamped event must land where its stamp put it, the unstamped one at arrival";
  EXPECT_LE(apart, 2.0 + (after - before) + kWireResolution)
      << "the stamped event must land where its stamp put it, the unstamped one at arrival";
  EXPECT_GT(stamped_times[0], start[0] + 0.5);
}

// Every line the process logs while one of these lives, for a test that asserts on a warning.
class LogCapture {
 public:
  LogCapture() {
    previous_ = rcutils_logging_get_output_handler();
    instance_ = this;
    rcutils_logging_set_output_handler(&LogCapture::Handle);
  }
  ~LogCapture() {
    rcutils_logging_set_output_handler(previous_);
    instance_ = nullptr;
  }
  std::size_t Count(const std::string& needle) const {
    std::lock_guard<std::mutex> lock(mu_);
    return static_cast<std::size_t>(std::count_if(
        lines_.begin(), lines_.end(),
        [&needle](const std::string& line) { return line.find(needle) != std::string::npos; }));
  }

 private:
  static void Handle(const rcutils_log_location_t* location, int severity, const char* name,
                     rcutils_time_point_value_t timestamp, const char* format, va_list* args) {
    LogCapture* self = instance_;
    if (self == nullptr) return;
    char line[4096];
    va_list copy;
    va_copy(copy, *args);
    std::vsnprintf(line, sizeof(line), format, copy);
    va_end(copy);
    {
      std::lock_guard<std::mutex> lock(self->mu_);
      self->lines_.emplace_back(line);
    }
    if (self->previous_ != nullptr) self->previous_(location, severity, name, timestamp, format, args);
  }

  static inline LogCapture* instance_ = nullptr;
  rcutils_logging_output_handler_t previous_ = nullptr;
  mutable std::mutex mu_;
  std::vector<std::string> lines_;
};

// No record precedes the session it is in. An event stamped before the session began -- a step
// that started earlier, an event from the previous session arriving late -- lands on the session's
// start, and the node says so once per session rather than once per event.
TEST_F(LifecycleContractTest, AStampFromBeforeTheSessionIsPlacedAtItsStart) {
  MakeNode(/*auto_start_session=*/true);
  auto events =
      client_->create_publisher<Event>("/cognitive3d/events", rclcpp::QoS(100).reliable());
  LogCapture log;
  ASSERT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_TRUE(SpinUntil([&] { return events->get_subscription_count() > 0; }));

  const auto publish_early = [&](const std::string& name) {
    Event early;
    early.name = name;
    early.stamp = rclcpp::Time(node_->get_clock()->now().nanoseconds() - 30'000'000'000LL);
    events->publish(early);
  };
  const std::string warning = "before the session it is recorded into began";
  publish_early("early.one");
  publish_early("early.two");
  ASSERT_TRUE(events->wait_for_all_acked(std::chrono::seconds(5)));
  ASSERT_TRUE(SpinUntil([&] { return log.Count(warning) >= 1; }, std::chrono::seconds(10)));
  SpinUntil([] { return false; }, std::chrono::milliseconds(500));
  const std::size_t warned_in_first = log.Count(warning);

  // And said again in the next session.
  auto ended = End();
  ASSERT_NE(ended, nullptr);
  ASSERT_TRUE(ended->ok);
  auto started = Start("second");
  ASSERT_NE(started, nullptr);
  ASSERT_TRUE(started->ok) << started->message;
  publish_early("early.three");
  ASSERT_TRUE(events->wait_for_all_acked(std::chrono::seconds(5)));
  ASSERT_TRUE(SpinUntil([&] { return log.Count(warning) > warned_in_first; },
                        std::chrono::seconds(10)))
      << "said again for the next session";
  ASSERT_EQ(node_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  const auto starts = EventTimes(dir_, "c3d.sessionStart");
  ASSERT_EQ(starts.size(), 2u);
  const double first = std::min(starts[0], starts[1]);
  const double second = std::max(starts[0], starts[1]);
  for (const auto& [name, start] : std::vector<std::pair<std::string, double>>{
           {"early.one", first}, {"early.two", first}, {"early.three", second}}) {
    const auto times = EventTimes(dir_, name);
    ASSERT_EQ(times.size(), 1u) << name;
    EXPECT_GE(times[0], start) << name << " landed before the session it is in";
    EXPECT_NEAR(times[0], start, 0.01) << name;
  }
  EXPECT_EQ(warned_in_first, 1u) << "said once per session, not once per event";
  EXPECT_EQ(log.Count("before the session it is recorded into began"), 2u)
      << "said again for the next session";
}

// The event cap is spent when an event ARRIVES, whatever time it carries: four events arriving
// together are a burst however far apart their stamps are.
TEST_F(LifecycleContractTest, TheEventCapIsChargedAtArrivalNotAtTheStamp) {
  MakeNode(/*auto_start_session=*/true, {{"event_rate_limit_hz", 2.0}});
  auto events =
      client_->create_publisher<Event>("/cognitive3d/events", rclcpp::QoS(100).reliable());
  LogCapture log;
  ASSERT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_TRUE(SpinUntil([&] { return events->get_subscription_count() > 0; }));
  // So that every stamp below falls inside the session.
  SpinUntil([] { return false; }, std::chrono::milliseconds(2500));

  const std::int64_t now = node_->get_clock()->now().nanoseconds();
  for (int i = 0; i < 4; ++i) {
    Event event;
    event.name = "burst";
    event.stamp = rclcpp::Time(now - (3 - i) * 500'000'000LL);  // half a second apart
    events->publish(event);
  }
  ASSERT_TRUE(events->wait_for_all_acked(std::chrono::seconds(5)));
  // The cap said so, so the third had arrived; the rest are dispatched in this spin.
  ASSERT_TRUE(SpinUntil([&] { return log.Count("faster than event_rate_limit_hz") >= 1; },
                        std::chrono::seconds(10)))
      << "the cap never suppressed anything";
  SpinUntil([] { return false; }, std::chrono::milliseconds(300));
  ASSERT_EQ(node_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  EXPECT_EQ(EventTimes(dir_, "burst").size(), 2u)
      << "a burst of four into a 2 per second cap; their stamps are half a second apart";
}

// Under use_sim_time the stamp is simulation time, near zero, and must not reach the wire as such.
TEST_F(LifecycleContractTest, AStampOnTheSimulationClockLandsInTheRightYear) {
  MakeNode(/*auto_start_session=*/true, {{"use_sim_time", true}});
  auto events =
      client_->create_publisher<Event>("/cognitive3d/events", rclcpp::QoS(100).reliable());
  auto clock = client_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
  rosgraph_msgs::msg::Clock now;
  now.clock = rclcpp::Time(static_cast<int64_t>(100e9));
  const auto tick = [&] {
    clock->publish(now);
    return false;
  };
  ASSERT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_TRUE(SpinUntil([&] {
    tick();
    return node_->get_clock()->now().seconds() > 99.0;
  })) << "/clock never reached the node";
  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_TRUE(SpinUntil([&] {
    tick();
    return events->get_subscription_count() > 0;
  }));

  // The session must be older than the stamp's age for the stamp to fall inside it.
  SpinUntil(tick, std::chrono::milliseconds(3000));

  Event stamped;
  stamped.name = "sim.stamped";
  stamped.stamp = rclcpp::Time(static_cast<int64_t>(98e9));  // two simulated seconds ago
  const double published = WallNow();
  events->publish(stamped);
  ASSERT_TRUE(events->wait_for_all_acked(std::chrono::seconds(5)));
  SpinUntil(tick, std::chrono::milliseconds(500));
  const double spun = WallNow();
  ASSERT_EQ(node_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  const auto times = EventTimes(dir_, "sim.stamped");
  const auto start = EventTimes(dir_, "c3d.sessionStart");
  ASSERT_EQ(times.size(), 1u);
  ASSERT_EQ(start.size(), 1u);
  EXPECT_GT(times[0], 1577836800.0) << "ROS time reached the wire: " << times[0];
  // ROS time is frozen, so the stamp is two seconds old whenever it arrives: two seconds before
  // its arrival, which falls between the publish and the end of the spin.
  EXPECT_GE(times[0], published - 2.0 - kWireResolution);
  EXPECT_LE(times[0], spun - 2.0 + kWireResolution);
  EXPECT_GT(times[0], start[0]);
}

// The node resolves both hosts from its process environment, a custom pair included, and the
// gateway parameter moves only the data host.
TEST(NodeDestination, ACustomHostPairComesFromTheEnvironmentAndTheGatewayMovesOnlyTheDataHost) {
  const fs::path dir = fs::temp_directory_path() / ("c3d_hosts_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  ::setenv("C3D_DATA_HOST", "data.example.test", 1);
  ::setenv("C3D_API_HOST", "api.example.test", 1);
  const auto configure = [&dir](const std::string& gateway, const std::string& protocol) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"scene_id", std::string(kSceneId)},
        {"scene_version", std::string("1")},
        {"spool_dir", dir.string()},
        {"pose_convention", std::string("unity")},
        {"protocol", protocol},
        {"gateway", gateway},
    });
    auto node = std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
    EXPECT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        << node->configure_error();
    return node;
  };
  {
    auto node = configure("", "https");
    EXPECT_EQ(node->resolved_destination().environment, "custom");
    EXPECT_EQ(node->resolved_config().gateway, "data.example.test");
    EXPECT_EQ(node->resolved_destination().hosts.api, "api.example.test");
    node->cleanup();
  }
  {
    auto node = configure("127.0.0.1:8787", "http");
    EXPECT_EQ(node->resolved_config().gateway, "127.0.0.1:8787");
    EXPECT_EQ(node->resolved_destination().hosts.api, "api.example.test")
        << "the gateway parameter must never move the api host";
    node->cleanup();
  }
  ::unsetenv("C3D_API_HOST");
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"scene_id", std::string(kSceneId)},
        {"scene_version", std::string("1")},
        {"spool_dir", dir.string()},
        {"pose_convention", std::string("unity")},
    });
    auto node = std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
    EXPECT_NE(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_NE(node->configure_error().find("C3D_API_HOST"), std::string::npos)
        << node->configure_error();
  }
  ::unsetenv("C3D_DATA_HOST");
  std::error_code ec;
  fs::remove_all(dir, ec);
}

// A gateway naming another environment's data host is the cross-environment split itself, in
// every spelling that reaches that host, and a host variable is a bare host.
TEST(NodeDestination, AGatewayNamingAnotherEnvironmentsDataHostIsRefusedInAnySpelling) {
  const fs::path dir = fs::temp_directory_path() / ("c3d_split_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  ::setenv("C3D_DATA_HOST", "data.example.test", 1);
  ::setenv("C3D_API_HOST", "api.example.test", 1);
  const std::string owner = "is the " + std::string(c3d::kDefaultEnvironment) + " data host";
  for (const std::string spelling : {"data.cognitive3d.com", "DATA.COGNITIVE3D.COM",
                                     "data.cognitive3d.com:443", "data.cognitive3d.com."}) {
    const std::string refused =
        ConfigureError(dir, {{"protocol", std::string("https")}, {"gateway", spelling}});
    EXPECT_NE(refused.find(owner), std::string::npos) << "'" << spelling << "': " << refused;
  }
  EXPECT_EQ(ConfigureError(dir, {{"protocol", std::string("https")},
                                 {"gateway", std::string("127.0.0.1:8787")}}),
            "")
      << "a capture stub is no environment's host";

  ::setenv("C3D_DATA_HOST", "https://data.example.test", 1);
  const std::string scheme = ConfigureError(dir, {});
  EXPECT_NE(scheme.find("C3D_DATA_HOST must be a bare host"), std::string::npos) << scheme;
  ::unsetenv("C3D_DATA_HOST");
  ::unsetenv("C3D_API_HOST");
  std::error_code ec;
  fs::remove_all(dir, ec);
}

// A spool directory another writer holds is refused at configure, naming the directory and the
// holder, rather than at every session start after a configure that succeeded. Refused before
// the transition created anything, so the same node configures once the holder lets go.
TEST(SpoolDirectory, ADirectoryAnotherWriterHoldsIsRefusedAtConfigure) {
  const fs::path dir = fs::temp_directory_path() / ("c3d_held_" + std::to_string(::getpid()));
  fs::remove_all(dir);
  fs::create_directories(dir);
  auto holder = MakeUploadNode(dir, "127.0.0.1:1", {{"auto_start_session", false}});
  ASSERT_EQ(holder->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      << holder->configure_error();

  auto second = MakeUploadNode(dir, "127.0.0.1:1", {{"auto_start_session", false}});
  EXPECT_EQ(second->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  const std::string refusal = second->configure_error();
  EXPECT_NE(refusal.find("spool_dir " + dir.string()), std::string::npos) << refusal;
  EXPECT_NE(refusal.find("in use by process " + std::to_string(::getpid())), std::string::npos)
      << refusal;

  holder->cleanup();
  EXPECT_EQ(second->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      << second->configure_error();
  second->cleanup();
  std::error_code ec;
  fs::remove_all(dir, ec);
}

namespace {

// Every sample of one series in the spooled sensor payloads.
std::vector<double> SpooledSeries(const fs::path& dir, const std::string& name) {
  std::vector<double> values;
  for (const auto& body : SpooledBodies(dir)) {
    for (const auto& series : body.value("data", nlohmann::json::array())) {
      if (!series.is_object() || series.value("name", "") != name) continue;
      for (const auto& point : series.value("data", nlohmann::json::array())) {
        if (point.is_array() && point.size() >= 2) values.push_back(point[1].get<double>());
      }
    }
  }
  return values;
}

}  // namespace

// A refused key keeps every chunk and sends none, which from outside looks like a slow link. The
// node says which it is in ~/diagnostics, in its periodic report and in the session's c3d.* series.
TEST(NodeUploadHealth, ARefusedKeyIsSaidInTheDiagnosticsTheReportAndTheSession) {
  const fs::path dir = fs::temp_directory_path() / ("c3d_refused_" + std::to_string(::getpid()));
  fs::remove_all(dir);
  fs::create_directories(dir);
  c3d::testing_support::ThrottledHttpServer::Options refusing;
  refusing.response = "HTTP/1.1 401 Unauthorized\r\nContent-Length: 0\r\n\r\n";
  c3d::testing_support::ThrottledHttpServer server(refusing);
  {
    // A backlog for the refused key to meet as soon as the node is configured.
    auto recorder = MakeUploadNode(dir, server.host(), {{"uploads_paused", true}});
    ASSERT_EQ(recorder->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        << recorder->configure_error();
    ASSERT_EQ(recorder->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    recorder->deactivate();
    recorder->cleanup();
  }
  ASSERT_GE(SpooledChunks(dir), 2u);
  ASSERT_EQ(server.completed(), 0u);

  LogCapture log;
  auto node = MakeUploadNode(dir, server.host(), {{"report_period_s", 0.5}});
  auto listener = std::make_shared<rclcpp::Node>("upload_health_listener");
  std::mutex mu;
  std::vector<diagnostic_msgs::msg::DiagnosticStatus> said;
  auto subscription = listener->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/cognitive3d/diagnostics", 10,
      [&](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr array) {
        std::lock_guard<std::mutex> lock(mu);
        for (const auto& status : array->status) {
          if (status.name == "cognitive3d: uploads") said.push_back(status);
        }
      });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(listener);

  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      << node->configure_error();
  // The allowance is two refusals a second apart; the pause follows the second.
  ASSERT_TRUE(WaitUntil([&] { return server.completed() >= 2; }, 10.0)) << server.completed();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1800);
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(std::chrono::milliseconds(20));
  }
  ASSERT_EQ(node->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  executor.remove_node(listener);
  executor.remove_node(node->get_node_base_interface());

  {
    std::lock_guard<std::mutex> lock(mu);
    ASSERT_FALSE(said.empty()) << "~/diagnostics carried no uploads status";
    const auto& last = said.back();
    EXPECT_EQ(last.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR) << last.message;
    EXPECT_NE(last.message.find("PAUSED because the credentials were refused"), std::string::npos)
        << last.message;
    bool rejections = false;
    for (const auto& item : last.values) {
      if (item.key == "auth_rejections") rejections = std::stoi(item.value) >= 2;
    }
    EXPECT_TRUE(rejections) << "the status does not count the refusals";
  }
  EXPECT_GE(log.Count("[UPLOADS PAUSED: credentials refused]"), 1u)
      << "the periodic report did not flag the refused key";
  const std::vector<double> paused = SpooledSeries(dir, "c3d.upload_auth_paused");
  EXPECT_TRUE(std::find(paused.begin(), paused.end(), 1.0) != paused.end())
      << "the session never recorded the pause";
  const std::vector<double> refused = SpooledSeries(dir, "c3d.upload_auth_rejections");
  EXPECT_TRUE(std::any_of(refused.begin(), refused.end(), [](double v) { return v >= 2.0; }))
      << "the session never recorded the refusals";
  node->cleanup();
  std::error_code ec;
  fs::remove_all(dir, ec);
}

namespace {

// A wall clock reading 1970-01-02, as a board with no real-time clock reads before NTP, until a
// test sets it.
class UnsetClockNode : public cognitive3d_ros::Cognitive3DNode {
 public:
  using Cognitive3DNode::Cognitive3DNode;
  std::atomic<bool> clock_set{false};

 protected:
  double WallClockNow() const override {
    return clock_set ? Cognitive3DNode::WallClockNow() : 86400.0;
  }
};

}  // namespace

// A session stamped from an unset clock is accepted by the platform and never appears. None
// begins: ~/start_session says why and opens nothing, and an auto-started activation fails.
TEST(WallClock, ASessionStartOnAnUnsetClockIsRefusedWithTheReason) {
  const fs::path dir =
      fs::temp_directory_path() / ("c3d_unset_clock_" + std::to_string(::getpid()));
  fs::remove_all(dir);
  fs::create_directories(dir);
  const auto make = [&dir](bool auto_start) {
    auto overrides = UploadTestOverrides(dir, "127.0.0.1:1");
    overrides.emplace_back("auto_start_session", auto_start);
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    return std::make_shared<UnsetClockNode>(options);
  };
  {
    auto node = make(false);
    auto client = std::make_shared<rclcpp::Node>("wall_clock_client");
    auto start = client->create_client<StartSession>("/cognitive3d/start_session");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.add_node(client);
    ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        << node->configure_error();
    ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    ASSERT_TRUE(start->wait_for_service(std::chrono::seconds(5)));
    auto future = start->async_send_request(std::make_shared<StartSession::Request>());
    ASSERT_EQ(executor.spin_until_future_complete(future, std::chrono::seconds(10)),
              rclcpp::FutureReturnCode::SUCCESS);
    const auto response = future.get();
    EXPECT_FALSE(response->ok);
    EXPECT_TRUE(response->session_id.empty()) << response->session_id;
    EXPECT_NE(response->message.find("the wall clock reads 1970-01-02"), std::string::npos)
        << response->message;
    EXPECT_NE(response->message.find("NTP"), std::string::npos) << response->message;
    std::size_t markers = 0;
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(dir / "sessions", ec)) {
      if (entry.path().extension() == ".c3dsession") ++markers;
    }
    EXPECT_EQ(markers, 0u) << "a refused start wrote a session marker";
    executor.remove_node(client);
    executor.remove_node(node->get_node_base_interface());
    node->deactivate();
    node->cleanup();
  }
  {
    // A refused activation leaves the node inactive, and an inactive node starts no session even
    // once the clock is right: one opened then would have no ingest behind it.
    auto node = make(true);
    auto client = std::make_shared<rclcpp::Node>("wall_clock_activation_client");
    auto start = client->create_client<StartSession>("/cognitive3d/start_session");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.add_node(client);
    ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    EXPECT_NE(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        << "activation opened a session on an unset clock";
    node->clock_set = true;
    ASSERT_TRUE(start->wait_for_service(std::chrono::seconds(5)));
    auto future = start->async_send_request(std::make_shared<StartSession::Request>());
    ASSERT_EQ(executor.spin_until_future_complete(future, std::chrono::seconds(10)),
              rclcpp::FutureReturnCode::SUCCESS);
    const auto response = future.get();
    EXPECT_FALSE(response->ok) << "a session opened on a node whose activation was refused";
    EXPECT_TRUE(response->session_id.empty()) << response->session_id;
    EXPECT_NE(response->message.find("inactive"), std::string::npos) << response->message;
    EXPECT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        << "with the clock set, activation opens its session";
    executor.remove_node(client);
    executor.remove_node(node->get_node_base_interface());
    node->deactivate();
    node->cleanup();
  }
  std::error_code ec;
  fs::remove_all(dir, ec);
}

// The odometry translator is one instance for every Odometry topic, writing odom.*, so a second
// allowlisted Odometry topic interleaves with the first. Said once, naming both, for the topics
// subscribed now; a per-topic translator's type is never warned about.
TEST_F(LifecycleContractTest, ASecondTopicOfAFixedSeriesTypeIsWarnedAboutOnceNamingBoth) {
  LogCapture log;
  MakeNode(false, {{"topics", std::vector<std::string>{"/odom_a", "/odom_b"}},
                   {"discovery_period_s", 0.5}});
  auto odom_a = client_->create_publisher<nav_msgs::msg::Odometry>("/odom_a", 10);
  auto odom_b = client_->create_publisher<nav_msgs::msg::Odometry>("/odom_b", 10);
  auto battery_a = client_->create_publisher<sensor_msgs::msg::BatteryState>("/battery_a", 10);
  auto battery_b = client_->create_publisher<sensor_msgs::msg::BatteryState>("/battery_b", 10);
  const auto subscribed = [this](const std::vector<std::string>& topics) {
    for (const auto& topic : topics) {
      const auto it = node_->topic_stats().find(topic);
      if (it == node_->topic_stats().end() || !it->second.subscribed) return false;
    }
    return true;
  };
  ASSERT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_TRUE(SpinUntil(
      [&] { return subscribed({"/odom_a", "/odom_b", "/battery_a", "/battery_b"}); },
      std::chrono::seconds(10)));
  SpinUntil([] { return false; }, std::chrono::milliseconds(1500));
  EXPECT_EQ(log.Count("/odom_b and /odom_a are both nav_msgs/msg/Odometry") +
                log.Count("/odom_a and /odom_b are both nav_msgs/msg/Odometry"),
            1u);
  EXPECT_EQ(log.Count("are both sensor_msgs/msg/BatteryState"), 0u);

  // Re-subscribed with only one Odometry topic left on the graph: nothing to warn about.
  ASSERT_EQ(node_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  odom_a.reset();
  ASSERT_TRUE(SpinUntil([this] {
    for (const auto& entry : client_->get_topic_names_and_types()) {
      if (entry.first == "/odom_a") return false;
    }
    return true;
  }, std::chrono::seconds(10)));
  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_TRUE(SpinUntil([&] { return subscribed({"/odom_b"}); }, std::chrono::seconds(10)));
  SpinUntil([] { return false; }, std::chrono::milliseconds(1000));
  EXPECT_EQ(log.Count("are both nav_msgs/msg/Odometry"), 1u)
      << "warned about a topic no longer subscribed";
  node_->deactivate();
}

namespace {

// The properties of every spooled event of this name.
std::vector<nlohmann::json> EventProperties(const fs::path& dir, const std::string& name) {
  std::vector<nlohmann::json> found;
  for (const auto& body : SpooledBodies(dir)) {
    if (!body.contains("data") || !body["data"].is_array() || body.contains("interval")) continue;
    for (const auto& record : body["data"]) {
      if (record.value("name", std::string()) != name) continue;
      found.push_back(record.value("properties", nlohmann::json::object()));
    }
  }
  return found;
}

// The largest value each session recorded for one series, keyed by session id.
std::map<std::string, double> SeriesMaximumBySession(const fs::path& dir, const std::string& name) {
  std::map<std::string, double> maximum;
  for (const auto& body : SpooledBodies(dir)) {
    const std::string session = body.value("sessionid", std::string());
    for (const auto& series : body.value("data", nlohmann::json::array())) {
      if (!series.is_object() || series.value("name", "") != name) continue;
      for (const auto& point : series.value("data", nlohmann::json::array())) {
        if (!point.is_array() || point.size() < 2) continue;
        const double value = point[1].get<double>();
        const auto it = maximum.find(session);
        if (it == maximum.end() || value > it->second) maximum[session] = value;
      }
    }
  }
  return maximum;
}

}  // namespace

// A published event's `c3d.*` properties would contradict or duplicate the SDK's own, among them
// `c3d.position_unknown`. Each is dropped, each key is warned about once a session, and every drop
// is counted in the session it happened in.
TEST_F(LifecycleContractTest, AnEventsSdkPropertiesAreDroppedWarnedOnceAndCounted) {
  LogCapture log;
  MakeNode(/*auto_start_session=*/true);
  auto events =
      client_->create_publisher<Event>("/cognitive3d/events", rclcpp::QoS(100).reliable());
  ASSERT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_TRUE(SpinUntil([&] { return events->get_subscription_count() > 0; }));
  Event event;
  event.name = "mission.leg";
  event.properties_json = R"({"leg":3,"c3d.position_unknown":false,"c3d.app.version":"9.9"})";
  for (int i = 0; i < 2; ++i) {
    events->publish(event);
    SpinUntil([] { return false; }, std::chrono::milliseconds(300));
  }
  // Past the next five-second telemetry record.
  SpinUntil([] { return false; }, std::chrono::milliseconds(6000));
  ASSERT_EQ(node_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(log.Count("property 'c3d.app.version' dropped"), 1u);
  EXPECT_EQ(log.Count("property 'c3d.position_unknown' dropped"), 1u);

  // A second session counts and warns from nothing.
  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_TRUE(SpinUntil([&] { return events->get_subscription_count() > 0; }));
  events->publish(event);
  SpinUntil([] { return false; }, std::chrono::milliseconds(6000));
  ASSERT_EQ(node_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  const std::vector<nlohmann::json> recorded = EventProperties(dir_, "mission.leg");
  ASSERT_EQ(recorded.size(), 3u);
  for (const auto& properties : recorded) {
    EXPECT_EQ(properties.value("leg", 0), 3) << properties.dump();
    EXPECT_FALSE(properties.contains("c3d.app.version")) << properties.dump();
    // No pose has been sampled, so the node's own answer is the one that stands.
    EXPECT_EQ(properties.value("c3d.position_unknown", false), true) << properties.dump();
  }
  EXPECT_EQ(log.Count("property 'c3d.app.version' dropped"), 2u);
  EXPECT_EQ(log.Count("property 'c3d.position_unknown' dropped"), 2u);
  const std::map<std::string, double> dropped =
      SeriesMaximumBySession(dir_, "c3d.event_properties_dropped");
  ASSERT_EQ(dropped.size(), 2u) << "each session records its own count";
  std::vector<double> maxima;
  for (const auto& [session, maximum] : dropped) maxima.push_back(maximum);
  std::sort(maxima.begin(), maxima.end());
  EXPECT_EQ(maxima, (std::vector<double>{2.0, 4.0}));
}

// A topic refused by `topic_config` and one refused for being a bulk type take different fixes,
// so the report and `~/diagnostics` each name the reason. A battery is taken by its type, so its
// rule's `refuse` is what keeps it out.
TEST_F(LifecycleContractTest, ARefusedTopicIsReportedWithItsOwnReason) {
  LogCapture log;
  MakeNode(false, {{"topics", std::vector<std::string>{"/battery_state", "/camera/image"}},
                   {"topic_config.battery_state.ingest", std::string("refuse")},
                   {"discovery_period_s", 0.2},
                   {"report_period_s", 0.5}});
  auto battery =
      client_->create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", 10);
  auto camera = client_->create_publisher<sensor_msgs::msg::Image>("/camera/image", 10);
  std::map<std::string, std::string> said;
  auto listener = client_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/cognitive3d/diagnostics", 10,
      [&said](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr array) {
        for (const auto& status : array->status) said[status.name] = status.message;
      });
  ASSERT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_TRUE(SpinUntil(
      [&] {
        return said.count("cognitive3d: /battery_state") &&
               said.count("cognitive3d: /camera/image");
      },
      std::chrono::seconds(10)));
  EXPECT_EQ(said["cognitive3d: /battery_state"],
            "refused (ingest is `refuse` in `topic_config`, not subscribed)");
  EXPECT_EQ(said["cognitive3d: /camera/image"], "refused (bulk type, not subscribed)");
  EXPECT_GE(log.Count("REFUSED (sensor_msgs/msg/BatteryState, ingest is `refuse` in "
                      "`topic_config`, never subscribed)"),
            1u);
  EXPECT_GE(log.Count("REFUSED (sensor_msgs/msg/Image, bulk type, never subscribed)"), 1u);
  node_->deactivate();
}

// `~/diagnostics` gives a topic's `ingest` as its `subscribed` line does: a value that changes what
// is recorded, or one that was set, marked when it changes nothing, and nothing for a message
// type's default that changes nothing. No value but `refuse` changes what arrives, so a topic's
// observed rate is read whatever its `ingest`.
TEST_F(LifecycleContractTest, DiagnosticsGiveAnIngestOnlyWhereItActsOrWasSet) {
  MakeNode(false,
           {{"topics", std::vector<std::string>{"/odom", "/probe/temperature", "/camera/image"}},
            {"topic_config.probe_temperature.ingest", std::string("metadata_once")},
            {"discovery_period_s", 0.1},
            {"report_period_s", 0.3}});
  auto odom = client_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  auto temperature =
      client_->create_publisher<sensor_msgs::msg::Temperature>("/probe/temperature", 10);
  auto camera = client_->create_publisher<sensor_msgs::msg::Image>("/camera/image", 10);
  std::map<std::string, std::map<std::string, std::string>> said;
  auto listener = client_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/cognitive3d/diagnostics", 10,
      [&said](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr array) {
        for (const auto& status : array->status) {
          auto& values = said[status.name];
          values.clear();
          for (const auto& item : status.values) values[item.key] = item.value;
        }
      });
  ASSERT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  const auto rate_read = [&said] {
    const auto it = said.find("cognitive3d: /probe/temperature");
    return it != said.end() && it->second.count("observed_hz") != 0 &&
           it->second.at("observed_hz") != "n/a";
  };
  // Publishes on every poll: a rate is read only from 3 messages over a second or more.
  ASSERT_TRUE(SpinUntil(
      [&] {
        odom->publish(nav_msgs::msg::Odometry());
        temperature->publish(sensor_msgs::msg::Temperature());
        return rate_read() && said.count("cognitive3d: /odom") != 0 &&
               said.count("cognitive3d: /camera/image") != 0;
      },
      std::chrono::seconds(10)))
      << "no observed rate for a topic whose ingest is metadata_once";
  EXPECT_EQ(said["cognitive3d: /odom"].count("ingest"), 0u)
      << "a type default that changes nothing: " << said["cognitive3d: /odom"]["ingest"];
  EXPECT_EQ(said["cognitive3d: /probe/temperature"]["ingest"],
            "metadata_once (configured, has no effect)");
  EXPECT_EQ(said["cognitive3d: /camera/image"]["ingest"], "refuse");
  node_->deactivate();
}

// Only some translators read `topic_config.<key>.series`. Set on any other topic it is ignored,
// which the plan says at configure and the node says, naming the type, once it subscribes.
TEST_F(LifecycleContractTest, ASeriesNameItsTypeDoesNotReadIsWarnedAboutAtSubscription) {
  LogCapture log;
  MakeNode(false, {{"topics", std::vector<std::string>{"/odom_x", "/ultra"}},
                   {"topic_config.odom_x.series", std::string("custom.speed")},
                   {"topic_config.ultra.series", std::string("ultra.distance_meters")},
                   {"discovery_period_s", 0.2}});
  auto odometry = client_->create_publisher<nav_msgs::msg::Odometry>("/odom_x", 10);
  auto ultra = client_->create_publisher<std_msgs::msg::Float32>("/ultra", 10);
  ASSERT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(log.Count("series=custom.speed (if the topic is std_msgs/msg/Float32 or "
                      "std_msgs/msg/Int32MultiArray)"),
            1u);
  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  const auto subscribed = [this](const std::string& topic) {
    const auto it = node_->topic_stats().find(topic);
    return it != node_->topic_stats().end() && it->second.subscribed;
  };
  ASSERT_TRUE(SpinUntil([&] { return subscribed("/odom_x") && subscribed("/ultra"); },
                        std::chrono::seconds(10)));
  EXPECT_EQ(log.Count("topic_config.odom_x.series ('custom.speed') is ignored: /odom_x is "
                      "nav_msgs/msg/Odometry"),
            1u);
  EXPECT_EQ(log.Count("topic_config.ultra.series ('ultra.distance_meters') is ignored"), 0u)
      << "a Float32 topic records under the name it was given";
  node_->deactivate();
}

// A described sensor's kind says which type its readings are, and so which series they are
// recorded under. A topic of another type records under other names, or none, so the node says so
// once it subscribes, naming both parameters and both types; a topic of the kind's type is fine.
TEST_F(LifecycleContractTest, ASensorTopicOfAnotherTypeThanItsKindIsAnErrorAtSubscription) {
  LogCapture log;
  MakeNode(false, {{"topics", std::vector<std::string>{"/sonar", "/line", "/ultra"}},
                   {"sensors", std::vector<std::string>{"sonar", "line", "ultra"}},
                   {"sensor.sonar.kind", std::string("cone_rangefinder")},
                   {"sensor.sonar.topic", std::string("/sonar")},
                   {"sensor.sonar.units", std::string("m")},
                   {"sensor.sonar.cone_half_angle_deg", 15.0},
                   {"sensor.line.kind", std::string("binary_array")},
                   {"sensor.line.topic", std::string("/line")},
                   {"sensor.line.element_count", 4},
                   {"sensor.ultra.kind", std::string("cone_rangefinder")},
                   {"sensor.ultra.topic", std::string("/ultra")},
                   {"sensor.ultra.units", std::string("cm")},
                   {"sensor.ultra.cone_half_angle_deg", 7.5},
                   {"discovery_period_s", 0.2}});
  auto range = client_->create_publisher<sensor_msgs::msg::Range>("/sonar", 10);
  auto line = client_->create_publisher<std_msgs::msg::Float32>("/line", 10);
  auto ultra = client_->create_publisher<std_msgs::msg::Float32>("/ultra", 10);
  ASSERT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      << node_->configure_error();
  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  const auto subscribed = [this](const std::string& topic) {
    const auto it = node_->topic_stats().find(topic);
    return it != node_->topic_stats().end() && it->second.subscribed;
  };
  ASSERT_TRUE(SpinUntil(
      [&] { return subscribed("/sonar") && subscribed("/line") && subscribed("/ultra"); },
      std::chrono::seconds(10)));
  // Discovery runs again while active, and the topic is subscribed once.
  const auto settle = std::chrono::steady_clock::now() + std::chrono::milliseconds(600);
  while (std::chrono::steady_clock::now() < settle) {
    executor_.spin_some(std::chrono::milliseconds(20));
  }
  EXPECT_EQ(log.Count("sensor.sonar.kind is cone_rangefinder, whose readings are "
                      "std_msgs/msg/Float32, and sensor.sonar.topic /sonar is "
                      "sensor_msgs/msg/Range on the graph"),
            1u);
  EXPECT_EQ(log.Count("sensor.line.kind is binary_array, whose readings are "
                      "std_msgs/msg/Int32MultiArray, and sensor.line.topic /line is "
                      "std_msgs/msg/Float32 on the graph"),
            1u);
  EXPECT_EQ(log.Count("sensor.ultra.kind"), 0u) << "a Float32 topic is a cone rangefinder's";
  node_->deactivate();
}

// The node asks the graph whether a camera's CameraInfo topic has a publisher: a camera whose field
// of view would come from one nothing publishes is said once, at its first frame, and one whose
// publisher has not sent yet is not, though its frames are recorded.
TEST_F(LifecycleContractTest, ACameraInfoTopicNothingPublishesIsSaidAtTheFirstFrame) {
  LogCapture log;
  MakeNode(true, {{"reference_frame", std::string("odom")},
                  {"fallback_reference_frame", std::string("")},
                  {"discovery_period_s", 0.2},
                  {"cameras", std::vector<std::string>{"front", "rear"}},
                  {"camera.front.topic", std::string("/front/image")},
                  {"camera.rear.topic", std::string("/rear/image")}});
  auto front = client_->create_publisher<sensor_msgs::msg::Image>("/front/image", 10);
  auto rear = client_->create_publisher<sensor_msgs::msg::Image>("/rear/image", 10);
  auto rear_info = client_->create_publisher<sensor_msgs::msg::CameraInfo>("/rear/camera_info", 10);
  tf2_ros::StaticTransformBroadcaster mounts(client_);
  geometry_msgs::msg::TransformStamped base;
  base.header.stamp = client_->now();
  base.header.frame_id = "odom";
  base.child_frame_id = "base_link";
  base.transform.rotation.w = 1.0;
  geometry_msgs::msg::TransformStamped camera = base;
  camera.header.frame_id = "base_link";
  camera.child_frame_id = "camera_link";
  mounts.sendTransform({base, camera});
  ASSERT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      << node_->configure_error();
  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // Past the five-second mark the self-telemetry records each camera's frame count at.
  sensor_msgs::msg::Image frame;
  frame.header.frame_id = "camera_link";
  frame.width = 8;
  frame.height = 8;
  frame.encoding = "rgb8";
  frame.step = 24;
  frame.data.assign(8 * 8 * 3, 64);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(6000);
  auto next_frame = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() < deadline) {
    if (std::chrono::steady_clock::now() >= next_frame) {
      front->publish(frame);
      rear->publish(frame);
      next_frame += std::chrono::milliseconds(100);
    }
    executor_.spin_some(std::chrono::milliseconds(5));
  }
  ASSERT_EQ(node_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(log.Count("camera 'front': its frames take their field of view from "
                      "/front/camera_info, which has no publisher"),
            1u);
  EXPECT_EQ(log.Count("camera 'rear': its frames take their field of view"), 0u);
  EXPECT_GE(SeriesMaximum(SpooledBodies(dir_), "c3d.camera.rear.frames"), 1.0)
      << "the rear camera recorded no frame, so its silence says nothing";
}

namespace {

// The `Reason` of every session end in the spool under `dir`.
std::vector<std::string> EndReasons(const fs::path& dir) {
  std::vector<std::string> reasons;
  for (const auto& body : SpooledBodies(dir)) {
    if (!body.contains("data") || !body["data"].is_array() || body.contains("interval")) continue;
    for (const auto& record : body["data"]) {
      if (record.value("name", std::string()) != "c3d.sessionEnd") continue;
      reasons.push_back(record.value("properties", nlohmann::json::object())
                            .value("Reason", std::string("(none)")));
    }
  }
  return reasons;
}

std::size_t OpenSessionMarkers(const fs::path& dir) {
  std::size_t markers = 0;
  std::error_code ec;
  for (const auto& entry : fs::directory_iterator(dir / "sessions", ec)) {
    if (entry.path().extension() == ".c3dsession") ++markers;
  }
  return markers;
}

// The report timer's period in the test below, which is the timer that node refuses to create.
constexpr double kRefusedTimerPeriod = 7.0;

// A node whose activation rclcpp refuses at its last step, the report timer, as it refuses a
// period that does not fit in nanoseconds: after the session has opened and every subscription
// exists.
class RefusedTimerNode : public cognitive3d_ros::Cognitive3DNode {
 public:
  using Cognitive3DNode::Cognitive3DNode;
  std::atomic<bool> refuse{true};

 protected:
  rclcpp::TimerBase::SharedPtr CreateActivationTimer(std::chrono::duration<double> period,
                                                     std::function<void()> callback) override {
    if (refuse && period.count() == kRefusedTimerPeriod) {
      throw std::invalid_argument("timer period must be less than std::chrono::nanoseconds::max()");
    }
    return Cognitive3DNode::CreateActivationTimer(period, std::move(callback));
  }
};

}  // namespace

// An activation that throws part-way is undone whole. The node is inactive, as it was before, not
// unconfigured with its subscriptions and session still live: nothing is subscribed, the session
// the activation opened is ended, and ~/start_session is refused as for any inactive node.
// Activated again, it works.
TEST_F(LifecycleContractTest, AnActivationThatThrowsIsUndoneWhole) {
  auto battery = client_->create_publisher<sensor_msgs::msg::BatteryState>("/probe/battery", 10);
  auto node = MakeNodeOf<RefusedTimerNode>(
      /*auto_start_session=*/true, {{"topics", std::vector<std::string>{"/probe/battery"}},
                                    {"report_period_s", kRefusedTimerPeriod}});
  const auto subscribers = [&] {
    return client_->count_subscribers("/tf") + client_->count_subscribers("/tf_static") +
           client_->count_subscribers("/cognitive3d/events") + battery->get_subscription_count();
  };
  ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      << node->configure_error();
  ASSERT_TRUE(SpinUntil([&] { return node->count_publishers("/probe/battery") > 0; }))
      << "the allowlisted topic must be on the node's graph before it activates";

  LogCapture log;
  EXPECT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      << "a failed activation leaves the node where it was";
  EXPECT_EQ(log.Count("activation failed: timer period must be less than"), 1u);
  EXPECT_TRUE(SpinUntil([&] { return subscribers() == 0; }))
      << "a subscription outlived the failed activation: " << subscribers();
  auto ended = End();
  ASSERT_NE(ended, nullptr);
  EXPECT_FALSE(ended->ok) << "a session outlived the failed activation: " << ended->session_id;
  auto refused = Start("after-a-failed-activation");
  ASSERT_NE(refused, nullptr);
  EXPECT_FALSE(refused->ok);
  EXPECT_TRUE(refused->session_id.empty()) << refused->session_id;
  EXPECT_NE(refused->message.find("activate"), std::string::npos) << refused->message;
  EXPECT_EQ(EndReasons(dir_), std::vector<std::string>{"activation_failed"})
      << "the session the activation opened is ended, and says why";
  EXPECT_EQ(OpenSessionMarkers(dir_), 0u);

  node->refuse = false;
  ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_TRUE(SpinUntil([&] { return subscribers() == 4; })) << subscribers();
  auto second = End();
  ASSERT_NE(second, nullptr);
  EXPECT_TRUE(second->ok) << "the second activation opened no session";
  node->deactivate();
}

// Sessions shorter than a second, back to back. Each id is a whole second past the last, so a
// session's start can lead the wall clock, by up to the number of sessions begun in one second.
// Every record a session carries is at or after its own start all the same -- its poses, an
// unstamped event and its end -- and the event is where it arrived on that session's timeline,
// not moved to the start and not blamed on a stamp it does not have.
TEST_F(LifecycleContractTest, ASessionStartedAheadOfTheWallClockRecordsNothingBeforeItsStart) {
  MakeNode(/*auto_start_session=*/false, {{"fallback_reference_frame", std::string("")}});
  tf2_ros::StaticTransformBroadcaster tf(client_);
  geometry_msgs::msg::TransformStamped base;
  base.header.frame_id = "map";
  base.child_frame_id = "base_link";
  base.transform.rotation.w = 1.0;
  tf.sendTransform(base);
  auto events =
      client_->create_publisher<Event>("/cognitive3d/events", rclcpp::QoS(100).reliable());
  LogCapture log;
  ASSERT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  ASSERT_TRUE(SpinUntil([&] {
    return events->get_subscription_count() > 0 &&
           node_->tf_buffer()->canTransform("map", "base_link", tf2::TimePointZero);
  }));

  std::vector<std::string> ids;
  bool led = false;
  // Four at least, and more until one leads: a session leads only when it begins inside the wall
  // second its predecessor's id names, so whether four do depends on how long Begin and End take.
  for (int i = 0; i < 12 && (i < 4 || !led); ++i) {
    auto started = Start("short-" + std::to_string(i));
    ASSERT_NE(started, nullptr);
    ASSERT_TRUE(started->ok) << started->message;
    ids.push_back(started->session_id);
    const double wall = std::chrono::duration<double>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();
    led = led || std::stod(started->session_id.substr(0, started->session_id.find('_'))) > wall;
    Event unstamped;
    unstamped.name = "trial.started";
    events->publish(unstamped);
    // Held by the node's reader before its session can end, however slowly it is delivered.
    ASSERT_TRUE(events->wait_for_all_acked(std::chrono::seconds(5)));
    SpinUntil([] { return false; }, std::chrono::milliseconds(250));
    auto ended = End();
    ASSERT_NE(ended, nullptr);
    ASSERT_TRUE(ended->ok);
  }
  ASSERT_EQ(node_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_TRUE(led) << "no session started ahead of the wall clock, so nothing here was tested";

  // Wire times carry five decimal places.
  constexpr double kResolution = 1e-4;
  for (const std::string& id : ids) {
    double start = -1.0, end = -1.0, event = -1.0;
    std::vector<double> poses;
    for (const auto& body : SpooledBodies(dir_)) {
      if (body.value("sessionid", std::string()) != id || !body.contains("data")) continue;
      for (const auto& record : body["data"]) {
        if (!record.contains("time")) continue;
        const double time = record["time"].get<double>();
        if (body.contains("interval")) poses.push_back(time);
        const std::string name = record.value("name", std::string());
        if (name == "c3d.sessionStart") start = time;
        if (name == "c3d.sessionEnd") end = time;
        if (name == "trial.started") event = time;
      }
    }
    ASSERT_GT(start, 0.0) << id;
    EXPECT_FALSE(poses.empty()) << id << " recorded no pose";
    for (const double pose : poses) {
      EXPECT_GE(pose, start - kResolution) << id << ": a pose " << start - pose
                                           << " s before its session's start";
    }
    ASSERT_GT(event, 0.0) << id;
    // Strictly inside its session, the one bound on its arrival that holds on any machine. An
    // event moved onto the start carries the start's own time, and one given its session's lead
    // twice lands a second or more past the start, beyond the end of a quarter-second session.
    EXPECT_GT(event, start)
        << id << ": the event was moved onto the start rather than placed at its arrival";
    EXPECT_LT(event, end) << id << ": the event is not where it arrived";
    EXPECT_GE(end, start - kResolution) << id << ": the session ends before it starts";
  }
  EXPECT_EQ(log.Count("before the session it is recorded into began"), 0u)
      << "an unstamped event was blamed on a stamp";
}

// A stop signal shuts the context down before the process destroys the node, as it does for a
// component container's nodes, so a session still open then ends as the orderly stop it is,
// `shutdown`. A node destroyed while its context still runs, as a container unloading it, ends
// its session as `destructor`. Either way the session's end is spooled and its marker closed.
TEST(NodeShutdown, ASessionOpenWhenTheContextShutsDownEndsAsAShutdown) {
  for (const bool context_shut_down_first : {true, false}) {
    const fs::path dir = fs::temp_directory_path() /
                         ("c3d_context_shutdown_" + std::to_string(::getpid()) + "_" +
                          std::to_string(context_shut_down_first));
    fs::remove_all(dir);
    fs::create_directories(dir);
    auto context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr);
    rclcpp::NodeOptions options;
    options.context(context);
    auto overrides = UploadTestOverrides(dir, "127.0.0.1:1");
    overrides.emplace_back("auto_start_session", true);
    options.parameter_overrides(overrides);
    auto node = std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
    ASSERT_EQ(node->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        << node->configure_error();
    ASSERT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    if (context_shut_down_first) context->shutdown("a stop signal");
    node.reset();
    if (!context_shut_down_first) context->shutdown("the test is over");
    EXPECT_EQ(EndReasons(dir),
              std::vector<std::string>{context_shut_down_first ? "shutdown" : "destructor"});
    EXPECT_EQ(OpenSessionMarkers(dir), 0u);
    std::error_code ec;
    fs::remove_all(dir, ec);
  }
}

// A parameter read at configure is not set while the node is configured, active or not: the set
// is rejected, saying how to apply it, and `ros2 param get` keeps naming the value in force. Once
// the node is cleaned up the same set is accepted, and in force at the next configure. The three
// a configured node reads again or applies at once are set as before.
TEST_F(LifecycleContractTest, AParameterReadAtConfigureIsNotSetWhileConfigured) {
  MakeNode(/*auto_start_session=*/false,
           {{"topics", std::vector<std::string>{"/probe/battery"}}});
  ASSERT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  for (const rclcpp::Parameter& late : std::vector<rclcpp::Parameter>{
           {"sample_rate_hz", 20.0},
           {"topic_config.probe_battery.rate_hz", 2.0},
           {"use_sim_time", true},
           {"remote_variables", true},
           {"scene_version", std::string("2")}}) {
    const auto result = node_->set_parameter(late);
    EXPECT_FALSE(result.successful) << late.get_name();
    EXPECT_EQ(result.reason.rfind(late.get_name() + " is read at configure", 0), 0u)
        << result.reason;
    EXPECT_NE(result.reason.find("ros2 lifecycle set /cognitive3d cleanup"), std::string::npos)
        << result.reason;
  }
  EXPECT_DOUBLE_EQ(node_->get_parameter("sample_rate_hz").as_double(), 30.0);
  EXPECT_FALSE(node_->set_parameters_atomically(
                        {{"session_name", std::string("mixed")}, {"sample_rate_hz", 20.0}})
                   .successful)
      << "one configure-only parameter rejects the whole atomic set";
  EXPECT_EQ(node_->get_parameter("session_name").as_string(), "");

  EXPECT_TRUE(node_->set_parameter({"session_name", std::string("named")}).successful);
  EXPECT_TRUE(
      node_->set_parameter({"session_properties", std::vector<std::string>{"site=w3"}}).successful);
  EXPECT_TRUE(node_->set_parameter({"uploads_paused", true}).successful);
  EXPECT_TRUE(node_->uploads_paused());
  EXPECT_TRUE(node_->set_parameter({"uploads_paused", false}).successful);

  ASSERT_EQ(node_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_FALSE(node_->set_parameter({"sample_rate_hz", 20.0}).successful) << "while active";
  ASSERT_EQ(node_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(node_->cleanup().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  EXPECT_TRUE(node_->set_parameter({"sample_rate_hz", 20.0}).successful);
  ASSERT_EQ(node_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_DOUBLE_EQ(node_->resolved_sampler_config().rate_hz, 20.0);
  node_->cleanup();
}

namespace {

// The properties two sessions of one node carried on the wire: the one activation opened, and
// the one `~/start_session` opened after it was ended. A reopened session has to describe itself
// as fully as the first. With `scan`, a LaserScan is published on /scan so a lidar there is
// subscribed and records its type's half.
std::vector<nlohmann::json> TwoSessionsOnTheWire(const std::string& tag,
                                                 const std::vector<rclcpp::Parameter>& extra,
                                                 bool scan = false) {
  const auto dir = fs::temp_directory_path() / ("c3d_" + tag + "_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  auto overrides = OfflineSessionOverrides(dir);
  overrides.push_back({"auto_start_session", true});
  overrides.push_back({"discovery_period_s", 0.2});
  for (const auto& parameter : extra) {
    overrides.erase(std::remove_if(overrides.begin(), overrides.end(),
                                   [&parameter](const rclcpp::Parameter& existing) {
                                     return existing.get_name() == parameter.get_name();
                                   }),
                    overrides.end());
    overrides.push_back(parameter);
  }
  rclcpp::NodeOptions options;
  options.parameter_overrides(overrides);
  auto node = std::make_shared<cognitive3d_ros::Cognitive3DNode>(options);
  auto publishers = std::make_shared<rclcpp::Node>("c3d_test_" + tag + "_publishers");
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher;
  if (scan) scan_publisher = publishers->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(publishers);
  std::vector<nlohmann::json> sessions;
  const auto finish = [&] {
    executor.remove_node(node->get_node_base_interface());
    executor.remove_node(publishers);
    node.reset();
    std::error_code ec;
    fs::remove_all(dir, ec);
  };
  if (node->configure().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    ADD_FAILURE() << node->configure_error();
    finish();
    return sessions;
  }
  EXPECT_EQ(node->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  const auto spin = [&](std::chrono::milliseconds total) {
    const auto deadline = std::chrono::steady_clock::now() + total;
    while (std::chrono::steady_clock::now() < deadline) {
      executor.spin_some(std::chrono::milliseconds(20));
    }
  };
  spin(std::chrono::milliseconds(scan ? 1500 : 300));
  std::string first_id;
  std::string reopened_id;
  {
    auto end = publishers->create_client<EndSession>("/cognitive3d/end_session");
    auto start = publishers->create_client<StartSession>("/cognitive3d/start_session");
    EXPECT_TRUE(end->wait_for_service(std::chrono::seconds(5)));
    EXPECT_TRUE(start->wait_for_service(std::chrono::seconds(5)));
    auto ended = end->async_send_request(std::make_shared<EndSession::Request>());
    EXPECT_EQ(executor.spin_until_future_complete(ended, std::chrono::seconds(10)),
              rclcpp::FutureReturnCode::SUCCESS);
    first_id = ended.get()->session_id;
    auto started = start->async_send_request(std::make_shared<StartSession::Request>());
    EXPECT_EQ(executor.spin_until_future_complete(started, std::chrono::seconds(10)),
              rclcpp::FutureReturnCode::SUCCESS);
    const auto response = started.get();
    EXPECT_TRUE(response->ok) << response->message;
    reopened_id = response->session_id;
  }
  spin(std::chrono::milliseconds(300));
  node->deactivate();
  node->cleanup();
  const auto bodies = SpooledBodies(dir);
  for (const std::string& id : {first_id, reopened_id}) {
    std::vector<nlohmann::json> of_session;
    for (const auto& body : bodies) {
      if (body.value("sessionid", std::string()) == id) of_session.push_back(body);
    }
    sessions.push_back(UnionOfProperties(of_session));
  }
  finish();
  return sessions;
}

// A property value as the wire writes it.
nlohmann::json OnTheWire(const c3d::PropertyValue& value) {
  switch (value.kind()) {
    case c3d::PropertyValue::Kind::kString: return value.as_string();
    case c3d::PropertyValue::Kind::kDouble: return value.as_double();
    case c3d::PropertyValue::Kind::kInt: return value.as_int();
    case c3d::PropertyValue::Kind::kBool: return value.as_bool();
  }
  return nullptr;
}

const char* JsonKind(const nlohmann::json& v) {
  return v.is_boolean() ? "bool" : v.is_number_integer() ? "int"
         : v.is_number_float() ? "double" : v.is_string() ? "string" : "other";
}

}  // namespace

// Session Replay draws a cone only for a cone_rangefinder named `ultrasonic`. One named anything
// else configures and records its description, and configure says that it will be drawn with no
// cone.
TEST(NodeSensorProperties, AConeRangefinderNamedOtherThanUltrasonicIsToldItIsDrawnWithNoCone) {
  const fs::path dir = fs::temp_directory_path() / ("c3d_cone_name_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  const auto described_as = [](const std::string& name) {
    return std::vector<rclcpp::Parameter>{
        {"topics", std::vector<std::string>{"/sonar"}},
        {"sensors", std::vector<std::string>{name}},
        {"sensor." + name + ".kind", std::string("cone_rangefinder")},
        {"sensor." + name + ".topic", std::string("/sonar")},
        {"sensor." + name + ".units", std::string("m")},
        {"sensor." + name + ".cone_half_angle_deg", 15.0}};
  };
  const std::string said = "Session Replay draws a cone only for a cone_rangefinder named "
                           "'ultrasonic'";
  {
    LogCapture log;
    EXPECT_EQ(ConfigureError(dir, described_as("front_sonar")), "");
    EXPECT_EQ(log.Count("sensor 'front_sonar' is a cone_rangefinder"), 1u);
    EXPECT_EQ(log.Count(said), 1u);
  }
  {
    LogCapture log;
    EXPECT_EQ(ConfigureError(dir, described_as("ultrasonic")), "");
    EXPECT_EQ(log.Count(said), 0u);
  }
  std::error_code ec;
  fs::remove_all(dir, ec);
}

// The compatibility contract. A robot that described its sensors with hand-written
// `c3d.ros.sensor.<name>.*` session properties gets the same keys and values from the `sensors`
// parameters, and Session Replay reads those keys off sessions recorded either way: every one of
// them reaches the wire with the value and the JSON type the hand-written entry was parsed to. The
// only keys added are the line sensor's topic and the sensor list.
TEST(NodeSensorProperties, HandWrittenSensorPropertiesAreReproducedFromParameters) {
  const std::vector<std::string> before = {
      "c3d.ros.sensor.ultrasonic.kind=cone_rangefinder",
      "c3d.ros.sensor.ultrasonic.series=ultrasonic.distance_centimeters",
      "c3d.ros.sensor.ultrasonic.units=centimeters",
      "c3d.ros.sensor.ultrasonic.topic=/ultrasonic",
      "c3d.ros.sensor.ultrasonic.cone_half_angle_degrees=7.5",
      "c3d.ros.sensor.ultrasonic.range_min_centimeters=2",
      "c3d.ros.sensor.ultrasonic.range_max_centimeters=400",
      "c3d.ros.sensor.ultrasonic.frame=ultrasonic_link",
      "c3d.ros.sensor.ultrasonic.mount_parent=base_link",
      "c3d.ros.sensor.ultrasonic.mount_xyz_meters=0.10,0.00,0.05",
      "c3d.ros.sensor.ultrasonic.mount_yaw_radians=0.0",
      "c3d.ros.sensor.ultrasonic.mount_measured=false",
      "c3d.ros.sensor.line.kind=binary_array",
      "c3d.ros.sensor.line.series=line.0,line.1,line.2,line.3",
      "c3d.ros.sensor.line.element_count=4",
      "c3d.ros.sensor.line.meaning_1=no_return",
      "c3d.ros.sensor.line.meaning_0=surface_present",
      "c3d.ros.sensor.line.mount_measured=false",
  };
  const auto sessions = TwoSessionsOnTheWire(
      "described_sensors",
      {{"topics", std::vector<std::string>{"/ultrasonic", "/line_sensor"}},
       {"topic_config.ultrasonic.series", std::string("ultrasonic.distance_centimeters")},
       {"topic_config.line_sensor.series", std::string("line")},
       {"sensors", std::vector<std::string>{"ultrasonic", "line"}},
       {"sensor.ultrasonic.kind", std::string("cone_rangefinder")},
       {"sensor.ultrasonic.topic", std::string("/ultrasonic")},
       {"sensor.ultrasonic.units", std::string("centimeters")},
       {"sensor.ultrasonic.cone_half_angle_deg", 7.5},
       {"sensor.ultrasonic.range_min_m", 0.02},
       {"sensor.ultrasonic.range_max_m", 4.0},
       {"sensor.ultrasonic.frame", std::string("ultrasonic_link")},
       {"sensor.ultrasonic.mount_parent", std::string("base_link")},
       {"sensor.ultrasonic.mount_xyz_m", std::vector<double>{0.10, 0.00, 0.05}},
       {"sensor.ultrasonic.mount_yaw_deg", 0.0},
       {"sensor.ultrasonic.mount_measured", false},
       {"sensor.line.kind", std::string("binary_array")},
       {"sensor.line.topic", std::string("/line_sensor")},
       {"sensor.line.element_count", 4},
       {"sensor.line.meaning_0", std::string("surface_present")},
       {"sensor.line.meaning_1", std::string("no_return")},
       {"sensor.line.mount_measured", false}});
  ASSERT_EQ(sessions.size(), 2u);
  for (const auto& sent : sessions) {
    const std::string dumped = sent.dump();
    std::set<std::string> described;
    for (const auto& [key, value] : sent.items()) {
      if (key.rfind("c3d.ros.sensor", 0) == 0) {
        described.insert(key);
      }
    }
    for (const std::string& entry : before) {
      const std::string key = entry.substr(0, entry.find('='));
      described.erase(key);
      const nlohmann::json expected =
          OnTheWire(c3d::ParseConfiguredProperty(key, entry.substr(entry.find('=') + 1)));
      ASSERT_TRUE(sent.contains(key)) << key << " missing from " << dumped;
      EXPECT_STREQ(JsonKind(sent[key]), JsonKind(expected)) << key << " = " << sent[key].dump();
      EXPECT_EQ(sent[key], expected) << key;
    }
    EXPECT_EQ(described, (std::set<std::string>{"c3d.ros.sensor.line.topic", "c3d.ros.sensors"}));
    EXPECT_EQ(sent.value("c3d.ros.sensor.line.topic", std::string()), "/line_sensor");
    EXPECT_EQ(sent.value("c3d.ros.sensors", std::string()), "ultrasonic,line");
  }
}
