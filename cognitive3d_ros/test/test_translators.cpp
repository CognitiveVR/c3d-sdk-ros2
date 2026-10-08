// Translator tests, driven by hand-built messages serialized the same way a
// generic subscription delivers them.
//
// These deliberately do not need a ROS graph, a simulator, or a network. A
// simulator hands you SUCCEEDED about 95% of the time; the bugs live in
// ABORTED, CANCELED, preemption and goals that vanish. Those are only reachable
// by constructing the message yourself.

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <map>
#include <memory>
#include <vector>

#include "c3d/spool.hpp"
#include "cognitive3d_ros/session_runner.hpp"
#include "cognitive3d_ros/translators.hpp"
#include "cognitive3d_ros/translators/battery_translator.hpp"
#include "cognitive3d_ros/translators/diagnostics_translator.hpp"
#include "cognitive3d_ros/translators/float32_translator.hpp"
#include "cognitive3d_ros/translators/imu_translator.hpp"
#include "cognitive3d_ros/translators/int32_multi_array_translator.hpp"
#include "cognitive3d_ros/translators/joint_state_translator.hpp"
#include "cognitive3d_ros/translators/laser_scan_translator.hpp"
#include "cognitive3d_ros/translators/objective_tracker.hpp"
#include "cognitive3d_ros/translators/odometry_translator.hpp"

using namespace cognitive3d_ros;
namespace fs = std::filesystem;

namespace {

// Captures every payload instead of sending it, so a test can assert on what
// the session would have transmitted.
class CapturingTransport : public c3d::ITransport {
 public:
  c3d::HttpResponse Post(const std::string& url, const std::string& body,
                         const std::vector<std::string>&) override {
    urls.push_back(url);
    bodies.push_back(body);
    c3d::HttpResponse ok;
    ok.status = 200;
    ok.saw_request_time_header = true;
    return ok;
  }
  std::vector<std::string> urls;
  std::vector<std::string> bodies;

  bool BodyContains(const std::string& needle) const {
    for (const auto& b : bodies) {
      if (b.find(needle) != std::string::npos) return true;
    }
    return false;
  }
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

template <typename MessageT>
rclcpp::SerializedMessage Serialize(const MessageT& msg) {
  rclcpp::SerializedMessage out;
  static rclcpp::Serialization<MessageT> serializer;
  serializer.serialize_message(&msg, &out);
  return out;
}

// The sensor series a transport was actually handed, read back off the wire.
//
// Asserting on the payload rather than on a value the translator was asked to
// produce is the difference between checking a derivation and restating it. A
// substring count over the whole body is not an alternative: it matches any
// other number on the wire, and so couples a test to values it says nothing
// about.
std::map<std::string, std::vector<double>> SensorSeriesIn(const CapturingTransport& transport) {
  std::map<std::string, std::vector<double>> out;
  for (std::size_t i = 0; i < transport.urls.size(); ++i) {
    if (transport.urls[i].find("/sensors/") == std::string::npos) continue;
    const auto payload = nlohmann::json::parse(transport.bodies[i]);
    for (const auto& series : payload.value("data", nlohmann::json::array())) {
      if (!series.contains("name")) continue;
      auto& values = out[series["name"].get<std::string>()];
      for (const auto& point : series.value("data", nlohmann::json::array())) {
        if (point.size() == 2) values.push_back(point[1].get<double>());
      }
    }
  }
  return out;
}

// The event records a transport was actually handed, read back off the wire.
//
// A substring search over the body says an event is there; only the parsed record
// says WHERE it is, and the position is the thing these tests are about.
struct WireEvent {
  std::string name;
  double x = 0.0, y = 0.0, z = 0.0;
  nlohmann::json properties = nlohmann::json::object();
};

std::vector<WireEvent> EventsIn(const CapturingTransport& transport) {
  std::vector<WireEvent> out;
  for (std::size_t i = 0; i < transport.urls.size(); ++i) {
    if (transport.urls[i].find("/events/") == std::string::npos) continue;
    const auto payload = nlohmann::json::parse(transport.bodies[i]);
    for (const auto& record : payload.value("data", nlohmann::json::array())) {
      WireEvent event;
      event.name = record.value("name", std::string());
      const auto point = record.value("point", nlohmann::json::array());
      if (point.size() == 3) {
        event.x = point[0].get<double>();
        event.y = point[1].get<double>();
        event.z = point[2].get<double>();
      }
      if (record.contains("properties")) event.properties = record["properties"];
      out.push_back(std::move(event));
    }
  }
  return out;
}

const WireEvent* FindEvent(const std::vector<WireEvent>& events, const std::string& name) {
  for (const auto& event : events) {
    if (event.name == name) return &event;
  }
  return nullptr;
}

// A quaternion for a roll and a pitch in degrees with zero yaw, composed here
// rather than taken from the SDK, so a change to the SDK's own maths cannot
// silently agree with a matching change in the expectation.
geometry_msgs::msg::Quaternion RollPitch(double roll_degrees, double pitch_degrees) {
  const double r = roll_degrees * M_PI / 360.0, p = pitch_degrees * M_PI / 360.0;
  geometry_msgs::msg::Quaternion q;
  q.w = std::cos(r) * std::cos(p);
  q.x = std::sin(r) * std::cos(p);
  q.y = std::cos(r) * std::sin(p);
  q.z = -std::sin(r) * std::sin(p);
  return q;
}

class TranslatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() /
           ("c3d_xlat_" + std::to_string(::getpid()) + "_" + std::to_string(counter_++));
    fs::create_directories(dir_);

    c3d::Config cfg;
    cfg.application_key = "KEY";
    cfg.scene_id = "SCENE";
    cfg.scene_version = "1";
    cfg.device_id = "robot01";
    cfg.spool_dir = dir_.string();
    cfg.auto_send_interval_s = 3600.0;
    cfg.worker_tick_s = 0.01;
    cfg.retry_min_s = 0.02;
    cfg.retry_max_s = 0.05;
    // Off, so a test that sends the same value twice sees both.
    cfg.sensor_change_threshold = -1.0;

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
  TranslationContext Ctx(const std::string& topic, double t, double rate = 0.0) {
    TranslationContext ctx;
    ctx.topic = topic;
    ctx.wire_time = t;
    ctx.ros_time = t;
    ctx.gate_time = t;
    ctx.rate_hz = rate;
    return ctx;
  }
  TranslationContext CtxAt(const std::string& topic, double t, const c3d::Vec3& position) {
    TranslationContext ctx = Ctx(topic, t);
    ctx.body_position = position;
    ctx.position_known = true;
    return ctx;
  }

  // A second session on a different pose convention, because the fixture's runner
  // is rep103 -- where the conversion is the identity and therefore proves nothing
  // about whether an event point takes it.
  struct Rig {
    std::shared_ptr<CapturingTransport> transport;
    std::shared_ptr<c3d::DiskSpool> spool;
    std::unique_ptr<SessionRunner> runner;
    void Flush() {
      runner->session()->Flush();
      runner->session()->PumpUploads();
    }
    ~Rig() {
      if (runner) runner->End("test");
    }
  };
  std::unique_ptr<Rig> MakeRig(PoseConvention convention) {
    const fs::path dir = dir_ / ("rig_" + std::to_string(counter_++));
    fs::create_directories(dir);
    c3d::Config cfg;
    cfg.application_key = "KEY";
    cfg.scene_id = "SCENE";
    cfg.scene_version = "1";
    cfg.device_id = "robot01";
    cfg.spool_dir = dir.string();
    cfg.auto_send_interval_s = 3600.0;
    cfg.worker_tick_s = 0.01;
    cfg.retry_min_s = 0.02;
    cfg.retry_max_s = 0.05;
    cfg.sensor_change_threshold = -1.0;

    auto rig = std::make_unique<Rig>();
    rig->transport = std::make_shared<CapturingTransport>();
    rig->spool = std::make_shared<c3d::DiskSpool>(dir.string(), 64ull << 20,
                                                  std::make_shared<c3d::NullLogger>());
    rig->runner = std::make_unique<SessionRunner>(cfg, convention, rig->transport, rig->spool,
                                                  std::make_shared<c3d::NullLogger>());
    rig->runner->Begin();
    return rig;
  }

  fs::path dir_;
  std::shared_ptr<CapturingTransport> transport_;
  std::shared_ptr<c3d::DiskSpool> spool_;
  std::unique_ptr<SessionRunner> runner_;
  static int counter_;
};
int TranslatorTest::counter_ = 0;

}  // namespace

// ---------------------------------------------------------------------------
// Battery
// ---------------------------------------------------------------------------

TEST_F(TranslatorTest, BatteryPercentageIsScaledFromZeroToOne) {
  // sensor_msgs/BatteryState.percentage is 0.0 to 1.0, NOT 0 to 100. Publishing
  // it raw gives a battery track pinned to the bottom of the chart forever.
  BatteryTranslator t;
  sensor_msgs::msg::BatteryState msg;
  msg.percentage = 0.42f;
  msg.voltage = 24.1f;
  t.Translate(Serialize(msg), Ctx("/battery_state", 1000.0), *runner_);
  Flush();
  EXPECT_TRUE(transport_->BodyContains("42.0")) << "percentage was not scaled to 0-100";
  EXPECT_TRUE(transport_->BodyContains("battery.voltage"));
}

TEST_F(TranslatorTest, BatteryStatusChangeEmitsAnEventButSteadyStateDoesNot) {
  BatteryTranslator t;
  sensor_msgs::msg::BatteryState msg;
  msg.percentage = 0.9f;
  msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  for (int i = 0; i < 5; ++i) {
    t.Translate(Serialize(msg), Ctx("/battery_state", 1000.0 + i), *runner_);
  }
  Flush();
  EXPECT_EQ(transport_->CountIn("robot.power.status_changed"), 0)
      << "an unchanging status must not produce events";

  msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  t.Translate(Serialize(msg), Ctx("/battery_state", 1010.0), *runner_);
  Flush();
  EXPECT_EQ(transport_->CountIn("robot.power.status_changed"), 1);
  EXPECT_TRUE(transport_->BodyContains("discharging"));
  EXPECT_TRUE(transport_->BodyContains("charging"));
}

TEST_F(TranslatorTest, BatteryIgnoresNaN) {
  // Drivers publish NaN for fields they cannot measure. NaN is not valid JSON
  // and would corrupt the whole payload.
  BatteryTranslator t;
  sensor_msgs::msg::BatteryState msg;
  msg.percentage = std::numeric_limits<float>::quiet_NaN();
  msg.voltage = std::numeric_limits<float>::quiet_NaN();
  msg.current = 3.0f;
  t.Translate(Serialize(msg), Ctx("/battery_state", 1000.0), *runner_);
  Flush();
  for (const auto& body : transport_->bodies) {
    EXPECT_EQ(body.find("nan"), std::string::npos) << body;
    EXPECT_EQ(body.find("NaN"), std::string::npos) << body;
  }
  EXPECT_TRUE(transport_->BodyContains("battery.current"));
  EXPECT_FALSE(transport_->BodyContains("battery.percent"));
}

TEST_F(TranslatorTest, ABatteryHealthChangeTheCapDiscardedIsOfferedAgain) {
  // Two fields change in ONE message and there is one token for the pair, so the
  // second event is discarded. Remembered anyway, the health change is lost
  // outright: no later message re-offers it, and the session says the battery has
  // been overheating since it was good.
  using B = sensor_msgs::msg::BatteryState;
  EventRateLimiter limiter;
  limiter.set_limit_hz(1.0);
  BatteryTranslator t;
  auto at = [this, &limiter](double when) {
    TranslationContext ctx = Ctx("/battery_state", when);
    ctx.event_limiter = &limiter;
    return ctx;
  };

  B msg;
  msg.percentage = 0.8f;
  msg.power_supply_status = B::POWER_SUPPLY_STATUS_DISCHARGING;
  msg.power_supply_health = B::POWER_SUPPLY_HEALTH_GOOD;
  t.Translate(Serialize(msg), at(1000.0), *runner_);

  msg.power_supply_status = B::POWER_SUPPLY_STATUS_NOT_CHARGING;
  msg.power_supply_health = B::POWER_SUPPLY_HEALTH_OVERHEAT;
  t.Translate(Serialize(msg), at(1000.0), *runner_);
  ASSERT_EQ(limiter.suppressed_for("/battery_state"), 1u)
      << "the case under test did not happen: nothing was suppressed";

  // The same reading again, with budget this time.
  t.Translate(Serialize(msg), at(1002.0), *runner_);
  Flush();
  EXPECT_EQ(transport_->CountIn("robot.power.status_changed"), 1);
  EXPECT_EQ(transport_->CountIn("robot.power.health_changed"), 1)
      << "the suppressed health change was never re-offered, so the session lost it";
}

TEST_F(TranslatorTest, TwoBatteryTopicsKeepSeparateStateAndSeparateSeries) {
  // ONE instance serves every BatteryState topic, because the registry keys
  // translators by type and `auto_subscribe_standard_types` takes the type
  // wherever it appears. Sharing the state across topics makes two batteries in
  // different power states emit a transition on EVERY message, and puts both
  // charge curves on one series where neither is readable.
  BatteryTranslator t;
  sensor_msgs::msg::BatteryState main;
  main.percentage = 0.80f;
  main.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  sensor_msgs::msg::BatteryState aux;
  aux.percentage = 0.20f;
  aux.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;

  for (int i = 0; i < 5; ++i) {
    t.Translate(Serialize(main), Ctx("/battery_state", 1000.0 + i), *runner_);
    t.Translate(Serialize(aux), Ctx("/robot1/aux_battery", 1000.0 + i), *runner_);
  }
  Flush();
  EXPECT_EQ(transport_->CountIn("robot.power.status_changed"), 0)
      << "neither battery changed state; only a state shared between them did";

  const auto series = SensorSeriesIn(*transport_);
  // The first topic of the type keeps the bare names, so a single-battery robot's
  // dashboard series carry no topic in their names.
  ASSERT_TRUE(series.count("battery.percent")) << "the first topic must keep the bare series";
  ASSERT_TRUE(series.count("battery.robot1.aux_battery.percent"));
  for (const double value : series.at("battery.percent")) EXPECT_DOUBLE_EQ(value, 80.0);
  for (const double value : series.at("battery.robot1.aux_battery.percent")) {
    EXPECT_DOUBLE_EQ(value, 20.0);
  }

  // And a real transition on one topic is still reported, once, naming it.
  main.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  t.Translate(Serialize(main), Ctx("/battery_state", 1010.0), *runner_);
  Flush();
  const auto events = EventsIn(*transport_);
  const WireEvent* changed = FindEvent(events, "robot.power.status_changed");
  ASSERT_NE(changed, nullptr);
  EXPECT_EQ(changed->properties.value("topic", std::string()), "/battery_state");
  EXPECT_EQ(changed->properties.value("from", std::string()), "discharging");
}

// `rate_hz` holds a BatteryState topic as it holds any other, and each topic is gated on its own:
// a battery publishing fast must not spend another's rate.
TEST_F(TranslatorTest, EachBatteryTopicIsHeldToTheConfiguredRateOnItsOwn) {
  BatteryTranslator t;
  for (int i = 0; i < 50; ++i) {  // five seconds at 10 Hz into a 1 Hz gate
    sensor_msgs::msg::BatteryState fast;
    fast.percentage = 0.5f + 0.001f * static_cast<float>(i);
    t.Translate(Serialize(fast), Ctx("/battery_state", 1000.0 + 0.1 * i, 1.0), *runner_);
    if (i % 10 == 0) {  // and once a second, between those, on a second battery
      sensor_msgs::msg::BatteryState slow;
      slow.percentage = 0.2f + 0.01f * static_cast<float>(i);
      t.Translate(Serialize(slow), Ctx("/aux_battery", 1000.5 + i / 10, 1.0), *runner_);
    }
  }
  Flush();
  const auto series = SensorSeriesIn(*transport_);
  ASSERT_EQ(series.count("battery.percent"), 1u);
  EXPECT_LE(series.at("battery.percent").size(), 6u) << "of 50 at 10 Hz into a 1 Hz gate";
  EXPECT_GE(series.at("battery.percent").size(), 4u);
  ASSERT_EQ(series.count("battery.aux_battery.percent"), 1u);
  EXPECT_EQ(series.at("battery.aux_battery.percent").size(), 5u)
      << "a battery at the gate's own rate lost messages to the other one";
}

// The gates are per session, like the rest of the translator's state: a session's first battery
// message is recorded however soon after the previous session's last one it arrives.
TEST_F(TranslatorTest, ABatteryGateStartsOverWithEachSession) {
  BatteryTranslator t;
  sensor_msgs::msg::BatteryState state;
  state.percentage = 0.5f;
  t.Translate(Serialize(state), Ctx("/battery_state", 1000.0, 1.0), *runner_);
  t.Reset();
  state.percentage = 0.75f;
  t.Translate(Serialize(state), Ctx("/battery_state", 1000.3, 1.0), *runner_);
  Flush();
  const auto series = SensorSeriesIn(*transport_);
  ASSERT_EQ(series.count("battery.percent"), 1u);
  EXPECT_EQ(series.at("battery.percent"), (std::vector<double>{50.0, 75.0}))
      << "the next session's first message was held to the last session's rate";
}

// ---------------------------------------------------------------------------
// Joint states
// ---------------------------------------------------------------------------

TEST_F(TranslatorTest, JointStateEmitsAggregatesNotPerJointByDefault) {
  // A 12-joint arm at 50 Hz is 600 scalars per second per channel. Aggregates
  // are one scalar each regardless of joint count, which is what makes this
  // affordable at all.
  JointStateTranslator t;
  sensor_msgs::msg::JointState msg;
  msg.name = {"j1", "j2", "j3"};
  msg.position = {0.1, 0.2, 0.3};
  msg.velocity = {0.5, -1.5, 0.0};
  msg.effort = {3.0, 4.0, 0.0};
  t.Translate(Serialize(msg), Ctx("/joint_states", 1000.0), *runner_);
  Flush();

  EXPECT_TRUE(transport_->BodyContains("joints.effort_rms"));
  EXPECT_TRUE(transport_->BodyContains("joints.effort_max"));
  EXPECT_TRUE(transport_->BodyContains("joints.velocity_max"));
  EXPECT_FALSE(transport_->BodyContains("joint.effort.j1"))
      << "per-joint channels must be opt-in";
  // max(|3|, |4|, |0|) = 4, max(|0.5|, |-1.5|, |0|) = 1.5
  EXPECT_TRUE(transport_->BodyContains("4.0"));
  EXPECT_TRUE(transport_->BodyContains("1.5"));
}

TEST_F(TranslatorTest, JointStateDecimatesToTheConfiguredRate) {
  JointStateTranslator t;
  sensor_msgs::msg::JointState msg;
  msg.name = {"j1"};
  msg.effort = {1.0};
  // 100 messages spanning 2 seconds, asked for 5 Hz.
  for (int i = 0; i < 100; ++i) {
    t.Translate(Serialize(msg), Ctx("/joint_states", 1000.0 + i * 0.02, 5.0), *runner_);
  }
  Flush();
  const int samples = transport_->CountIn("joints.count");
  EXPECT_LE(samples, 12) << "decimation did not take effect";
  EXPECT_GE(samples, 1);
}

TEST_F(TranslatorTest, JointStateToleratesEmptyAndMismatchedArrays) {
  // name, position, velocity and effort are independent arrays and need not be
  // the same length. Indexing them together is how you get a crash on a robot.
  JointStateTranslator t;
  sensor_msgs::msg::JointState msg;
  msg.name = {"j1", "j2", "j3"};
  msg.effort = {1.0};  // shorter than name
  msg.position = {};   // empty
  t.record_per_joint_effort = true;
  EXPECT_NO_THROW(t.Translate(Serialize(msg), Ctx("/joint_states", 1000.0), *runner_));

  sensor_msgs::msg::JointState empty;
  EXPECT_NO_THROW(t.Translate(Serialize(empty), Ctx("/joint_states", 1001.0), *runner_));
}

// ---------------------------------------------------------------------------
// Odometry
// ---------------------------------------------------------------------------

TEST_F(TranslatorTest, OdometryIntegratesDistanceAndIgnoresTeleports) {
  OdometryTranslator t;
  nav_msgs::msg::Odometry msg;
  for (int i = 0; i <= 10; ++i) {
    msg.pose.pose.position.x = i * 1.0;  // 10 metres in 1m steps
    t.Translate(Serialize(msg), Ctx("/odom", 1000.0 + i), *runner_);
  }
  // A 500m jump is a reset, not motion, and must not be added to the total.
  msg.pose.pose.position.x = 500.0;
  t.Translate(Serialize(msg), Ctx("/odom", 1020.0), *runner_);
  Flush();
  const auto series = SensorSeriesIn(*transport_);
  const auto distance = series.find("odom.distance_travelled");
  ASSERT_NE(distance, series.end());
  ASSERT_FALSE(distance->second.empty());
  EXPECT_DOUBLE_EQ(*std::max_element(distance->second.begin(), distance->second.end()), 10.0)
      << "a teleport was integrated as distance travelled";
}

TEST_F(TranslatorTest, OdometryRecordsSpeedNotPose) {
  // tf2 is the single source of pose. Odometry contributing a second pose would
  // let two subsystems disagree about where the robot was.
  OdometryTranslator t;
  nav_msgs::msg::Odometry msg;
  msg.twist.twist.linear.x = 3.0;
  msg.twist.twist.linear.y = 4.0;
  msg.twist.twist.angular.z = 0.75;
  t.Translate(Serialize(msg), Ctx("/odom", 1000.0), *runner_);
  Flush();
  EXPECT_TRUE(transport_->BodyContains("odom.speed"));
  EXPECT_TRUE(transport_->BodyContains("5.0")) << "sqrt(3^2+4^2) should be 5";
  EXPECT_TRUE(transport_->BodyContains("odom.yaw_rate"));
}

// ---------------------------------------------------------------------------
// Imu
// ---------------------------------------------------------------------------

TEST_F(TranslatorTest, ImuDerivesAttitudeAndRatesInDegrees) {
  ImuTranslator t;
  sensor_msgs::msg::Imu msg;
  msg.orientation = RollPitch(20.0, -10.0);
  msg.angular_velocity.z = 0.5;      // radians per second
  msg.linear_acceleration.x = 3.0;
  msg.linear_acceleration.z = 4.0;   // magnitude 5
  t.Translate(Serialize(msg), Ctx("/imu", 1000.0), *runner_);
  Flush();

  const auto series = SensorSeriesIn(*transport_);
  ASSERT_EQ(series.count("imu.roll_degrees"), 1u);
  ASSERT_EQ(series.count("imu.pitch_degrees"), 1u);
  EXPECT_NEAR(series.at("imu.roll_degrees").front(), 20.0, 1e-3);
  EXPECT_NEAR(series.at("imu.pitch_degrees").front(), -10.0, 1e-3);
  // Degrees per second, as the name says. A channel labelled degrees carrying
  // radians reads as a robot barely turning, and reads plausibly.
  ASSERT_EQ(series.count("imu.yaw_rate_degrees_per_second"), 1u);
  EXPECT_NEAR(series.at("imu.yaw_rate_degrees_per_second").front(), 28.6479, 1e-3);
  ASSERT_EQ(series.count("imu.acceleration_meters_per_second_squared"), 1u);
  EXPECT_NEAR(series.at("imu.acceleration_meters_per_second_squared").front(), 5.0, 1e-3);
}

TEST_F(TranslatorTest, ImuWithoutAnOrientationEstimateRecordsNoAttitude) {
  // REP-145 signals an absent estimate with covariance[0] == -1 and leaves the
  // data fields zeroed, so a driver with no orientation publishes a perfectly
  // formed identity quaternion. Taking it records a robot that is permanently
  // level, which is indistinguishable from a correct reading on flat ground.
  ImuTranslator t;
  sensor_msgs::msg::Imu msg;
  msg.orientation_covariance[0] = -1.0;
  msg.orientation.w = 1.0;
  msg.angular_velocity.z = 0.5;
  msg.linear_acceleration.z = 9.81;
  t.Translate(Serialize(msg), Ctx("/imu", 1000.0), *runner_);
  Flush();

  const auto series = SensorSeriesIn(*transport_);
  EXPECT_EQ(series.count("imu.roll_degrees"), 0u);
  EXPECT_EQ(series.count("imu.pitch_degrees"), 0u);
  // Only the orientation was disclaimed. The other two estimates carry their
  // own covariance and are still real measurements.
  EXPECT_EQ(series.count("imu.yaw_rate_degrees_per_second"), 1u);
  EXPECT_EQ(series.count("imu.acceleration_meters_per_second_squared"), 1u);
}

TEST_F(TranslatorTest, ImuDropsAnyEstimateThatDeclaresItselfAbsent) {
  ImuTranslator t;
  sensor_msgs::msg::Imu msg;
  msg.orientation = RollPitch(5.0, 5.0);
  msg.angular_velocity_covariance[0] = -1.0;
  msg.linear_acceleration_covariance[0] = -1.0;
  t.Translate(Serialize(msg), Ctx("/imu", 1000.0), *runner_);
  Flush();

  const auto series = SensorSeriesIn(*transport_);
  EXPECT_EQ(series.count("imu.roll_degrees"), 1u);
  EXPECT_EQ(series.count("imu.yaw_rate_degrees_per_second"), 0u)
      << "a gyro that says it has no estimate was recorded as a robot holding still";
  EXPECT_EQ(series.count("imu.acceleration_meters_per_second_squared"), 0u);
}

// ---------------------------------------------------------------------------
// LaserScan
// ---------------------------------------------------------------------------

TEST_F(TranslatorTest, LaserScanDerivesDistancesFromValidReturnsOnly) {
  LaserScanTranslator t;
  sensor_msgs::msg::LaserScan msg;
  msg.range_min = 0.2f;
  msg.range_max = 10.0f;
  msg.ranges = {std::numeric_limits<float>::infinity(),
                1.0f,
                2.0f,
                0.05f,  // below range_min: the driver saying "too close to tell"
                std::numeric_limits<float>::quiet_NaN(),
                3.0f};
  t.Translate(Serialize(msg), Ctx("/scan", 1000.0), *runner_);
  Flush();

  const auto series = SensorSeriesIn(*transport_);
  ASSERT_EQ(series.count("scan.range_min_meters"), 1u);
  EXPECT_NEAR(series.at("scan.range_min_meters").front(), 1.0, 1e-4);
  EXPECT_NEAR(series.at("scan.range_mean_meters").front(), 2.0, 1e-4);
  EXPECT_NEAR(series.at("scan.valid_fraction").front(), 0.5, 1e-4);
  // Three series and no more: a scan is hundreds of floats, and a per-beam
  // channel would spend a whole session's budget on data with nowhere to go.
  EXPECT_EQ(series.size(), 3u);
}

TEST_F(TranslatorTest, AScanThatSeesNothingRecordsTheZeroRatherThanADistance) {
  // A lidar facing glass, an open doorway or nothing at all returns inf.
  // Reporting a nearest range there invents an obstacle; reporting nothing at
  // all hides a dead sensor. The valid fraction is what separates the two.
  LaserScanTranslator t;
  sensor_msgs::msg::LaserScan msg;
  msg.range_min = 0.2f;
  msg.range_max = 10.0f;
  msg.ranges.assign(8, std::numeric_limits<float>::infinity());
  t.Translate(Serialize(msg), Ctx("/scan", 1000.0), *runner_);
  Flush();

  const auto series = SensorSeriesIn(*transport_);
  EXPECT_EQ(series.count("scan.range_min_meters"), 0u);
  EXPECT_EQ(series.count("scan.range_mean_meters"), 0u);
  ASSERT_EQ(series.count("scan.valid_fraction"), 1u);
  EXPECT_NEAR(series.at("scan.valid_fraction").front(), 0.0, 1e-9);
}

TEST_F(TranslatorTest, ImuAndScanEachDecimateToTheConfiguredRate) {
  // An IMU publishes at 100 to 400 Hz and a scan at up to 40, and each
  // translator carries its own decimation, so each needs its own proof of it.
  ImuTranslator imu;
  LaserScanTranslator scan;
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.linear_acceleration.z = 9.81;
  sensor_msgs::msg::LaserScan scan_msg;
  scan_msg.range_min = 0.2f;
  scan_msg.range_max = 10.0f;
  scan_msg.ranges = {1.0f};
  for (int i = 0; i < 20; ++i) {
    const double t = 1000.0 + i * 0.05;  // 20 Hz in, 2 Hz out
    imu.Translate(Serialize(imu_msg), Ctx("/imu", t, 2.0), *runner_);
    scan.Translate(Serialize(scan_msg), Ctx("/scan", t, 2.0), *runner_);
  }
  Flush();

  const auto series = SensorSeriesIn(*transport_);
  ASSERT_EQ(series.count("imu.acceleration_meters_per_second_squared"), 1u);
  EXPECT_LE(series.at("imu.acceleration_meters_per_second_squared").size(), 3u);
  ASSERT_EQ(series.count("scan.valid_fraction"), 1u);
  EXPECT_LE(series.at("scan.valid_fraction").size(), 3u);
}

// A source at the gate's own rate is delivered at that rate, jitter and all, so the message type
// defaults the reference file states -- odometry and joint states at 30 Hz -- are what a session
// carries. Scheduling each deadline from the accepted time would deliver about two thirds.
TEST_F(TranslatorTest, ASourceAtTheGateRateIsDeliveredAtThatRate) {
  OdometryTranslator odometry;
  JointStateTranslator joints;
  for (int i = 0; i < 600; ++i) {  // twenty seconds at 30 Hz, arriving up to 2 ms either side
    nav_msgs::msg::Odometry message;
    message.twist.twist.linear.x = 0.01 * i;  // a new value each time: sensors are change-gated
    const double jitter = (i % 2 == 0 ? 1.0 : -1.0) * 0.0005 * (i % 5);
    odometry.Translate(Serialize(message), Ctx("/odom", 1000.0 + i / 30.0 + jitter, 30.0),
                       *runner_);
  }
  for (int i = 0; i < 600; ++i) {
    sensor_msgs::msg::JointState message;
    message.name = {"j1"};
    message.effort = {1.0 + i};
    const double jitter = (i % 2 == 0 ? -1.0 : 1.0) * 0.0005 * (i % 5);
    joints.Translate(Serialize(message),
                     Ctx("/joint_states", 1000.0 + i / 30.0 + jitter, 30.0), *runner_);
  }
  Flush();
  const auto series = SensorSeriesIn(*transport_);
  ASSERT_EQ(series.count("odom.speed"), 1u);
  EXPECT_GE(series.at("odom.speed").size(), 585u) << "of 600 at 30 Hz into a 30 Hz gate";
  ASSERT_EQ(series.count("joints.effort_rms"), 1u);
  EXPECT_GE(series.at("joints.effort_rms").size(), 585u) << "of 600 at 30 Hz into a 30 Hz gate";
}

// A translator's gate reads the arrival clock the node hands it, never the time a record
// carries: arrivals a second apart pass a 1 Hz gate however close their wire times are.
TEST_F(TranslatorTest, ATranslatorGatesOnArrivalNotOnTheWireTime) {
  OdometryTranslator odometry;
  for (int i = 0; i < 10; ++i) {
    nav_msgs::msg::Odometry message;
    message.twist.twist.linear.x = 0.1 * i;
    TranslationContext ctx = Ctx("/odom", 1000.0 + 0.01 * i, 1.0);
    ctx.gate_time = 5000.0 + i;
    odometry.Translate(Serialize(message), ctx, *runner_);
  }
  Flush();
  const auto series = SensorSeriesIn(*transport_);
  ASSERT_EQ(series.count("odom.speed"), 1u);
  EXPECT_EQ(series.at("odom.speed").size(), 10u);
}

// ---------------------------------------------------------------------------
// Bare std_msgs scalars, and the series names they land on
//
// These assert on the SERIES NAME rather than on the value, because the name is
// the whole contract with anything downstream: a consumer matches on the name
// declared in the session properties, and a name that silently falls back is a
// visualisation that is simply absent while the samples all arrive and every
// count reports success. `SeriesNameFor` is the one function that decides it.
// ---------------------------------------------------------------------------

TEST_F(TranslatorTest, Float32SeriesFallsBackToTheDottedTopic) {
  Float32Translator t;
  std_msgs::msg::Float32 msg;
  msg.data = 42.0;
  t.Translate(Serialize(msg), Ctx("/ultrasonic", 1000.0), *runner_);
  msg.data = 7.5;
  t.Translate(Serialize(msg), Ctx("/robot1/range", 1000.0), *runner_);
  Flush();

  const auto series = SensorSeriesIn(*transport_);
  ASSERT_TRUE(series.count("ultrasonic.value"))
      << "with no configured series the topic names the channel, and the suffix is what "
         "keeps a dot in it";
  EXPECT_DOUBLE_EQ(series.at("ultrasonic.value").front(), 42.0);
  ASSERT_TRUE(series.count("robot1.range.value")) << "a slash becomes a dot";
  EXPECT_DOUBLE_EQ(series.at("robot1.range.value").front(), 7.5);
  EXPECT_FALSE(series.count("ultrasonic"))
      << "a name with no dot lands in the viewer's undifferentiated bucket";
}

TEST_F(TranslatorTest, AConfiguredFloat32SeriesIsUsedVerbatim) {
  // The point of setting a name is to STATE what the channel is -- the unit
  // included, since a bare Float32 carries none. Appending the translator's
  // suffix to it would undo that, so the configured name is final.
  Float32Translator t;
  std_msgs::msg::Float32 msg;
  msg.data = 42.0;
  TranslationContext ctx = Ctx("/ultrasonic", 1000.0);
  ctx.series = "ultrasonic.distance_centimeters";
  t.Translate(Serialize(msg), ctx, *runner_);
  Flush();

  const auto series = SensorSeriesIn(*transport_);
  ASSERT_TRUE(series.count("ultrasonic.distance_centimeters"));
  // The value is recorded UNCONVERTED: the name says centimetres, and nothing
  // in this code scales anything to agree with it.
  EXPECT_DOUBLE_EQ(series.at("ultrasonic.distance_centimeters").front(), 42.0);
  EXPECT_FALSE(series.count("ultrasonic.distance_centimeters.value"))
      << "the suffix must not be appended to a name the operator wrote";
  EXPECT_FALSE(series.count("ultrasonic.value"));
}

TEST_F(TranslatorTest, AnEmptySeriesIsTheSameAsNoSeries) {
  // `topic_config.<topic>.series` is declared with an empty default, so a series
  // never set arrives here as "". It must fall back rather than record a nameless
  // series.
  Float32Translator t;
  std_msgs::msg::Float32 msg;
  msg.data = 3.0;
  TranslationContext ctx = Ctx("/ultrasonic", 1000.0);
  ctx.series = "";
  t.Translate(Serialize(msg), ctx, *runner_);
  Flush();

  const auto series = SensorSeriesIn(*transport_);
  EXPECT_TRUE(series.count("ultrasonic.value"));
  EXPECT_FALSE(series.count("")) << "an empty name must never reach the wire";
}

TEST_F(TranslatorTest, Int32MultiArrayTreatsAConfiguredSeriesAsABase) {
  // A translator recording more than one number per message appends to the
  // configured name rather than replacing it, so the four bits of a line
  // follower are four selectable channels under one heading.
  Int32MultiArrayTranslator t;
  std_msgs::msg::Int32MultiArray msg;
  msg.data = {1, 0, 1};
  TranslationContext ctx = Ctx("/line_sensor", 1000.0);
  ctx.series = "line_sensor.ir";
  t.Translate(Serialize(msg), ctx, *runner_);
  Flush();

  const auto series = SensorSeriesIn(*transport_);
  ASSERT_TRUE(series.count("line_sensor.ir.0"));
  ASSERT_TRUE(series.count("line_sensor.ir.1"));
  ASSERT_TRUE(series.count("line_sensor.ir.2"));
  EXPECT_DOUBLE_EQ(series.at("line_sensor.ir.0").front(), 1.0);
  EXPECT_DOUBLE_EQ(series.at("line_sensor.ir.1").front(), 0.0);
  ASSERT_TRUE(series.count("line_sensor.ir.count"))
      << "the element count is what makes a variable-length array's length visible in the data";
  EXPECT_DOUBLE_EQ(series.at("line_sensor.ir.count").front(), 3.0);
  EXPECT_FALSE(series.count("line_sensor.0")) << "the configured base replaced the topic name";
}

TEST_F(TranslatorTest, Int32MultiArrayFallsBackToTheDottedTopicAsItsBase) {
  Int32MultiArrayTranslator t;
  std_msgs::msg::Int32MultiArray msg;
  msg.data = {1, 0};
  t.Translate(Serialize(msg), Ctx("/line_sensor", 1000.0), *runner_);
  Flush();

  const auto series = SensorSeriesIn(*transport_);
  EXPECT_TRUE(series.count("line_sensor.0"));
  EXPECT_TRUE(series.count("line_sensor.1"));
  ASSERT_TRUE(series.count("line_sensor.count"));
  EXPECT_DOUBLE_EQ(series.at("line_sensor.count").front(), 2.0);
}

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------

TEST_F(TranslatorTest, DiagnosticsEmitOnTransitionOnly) {
  // /diagnostics publishes every status in every message at 1 Hz. Forwarding it
  // verbatim buries the three events that mattered under thousands that did not.
  DiagnosticsTranslator t;
  diagnostic_msgs::msg::DiagnosticArray msg;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "motor_driver";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "nominal";
  msg.status = {status};

  for (int i = 0; i < 20; ++i) {
    t.Translate(Serialize(msg), Ctx("/diagnostics", 1000.0 + i), *runner_);
  }
  Flush();
  EXPECT_EQ(transport_->CountIn("robot.diagnostic.degraded"), 0);
  EXPECT_EQ(transport_->CountIn("robot.diagnostic.recovered"), 0)
      << "twenty identical OK messages must produce zero events";

  msg.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  msg.status[0].message = "overcurrent";
  t.Translate(Serialize(msg), Ctx("/diagnostics", 1030.0), *runner_);
  Flush();
  EXPECT_EQ(transport_->CountIn("robot.diagnostic.degraded"), 1);
  EXPECT_TRUE(transport_->BodyContains("overcurrent"));
  EXPECT_TRUE(transport_->BodyContains("motor_driver"));

  msg.status[0].level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  msg.status[0].message = "nominal";
  t.Translate(Serialize(msg), Ctx("/diagnostics", 1040.0), *runner_);
  Flush();
  EXPECT_EQ(transport_->CountIn("robot.diagnostic.recovered"), 1);
}

TEST_F(TranslatorTest, DiagnosticsReportAFaultThatPredatesTheSession) {
  // First sight of an already-broken component is worth an event, otherwise a
  // fault that started before we did is invisible for the whole run.
  DiagnosticsTranslator t;
  diagnostic_msgs::msg::DiagnosticArray msg;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "lidar";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  status.message = "not responding";
  msg.status = {status};
  t.Translate(Serialize(msg), Ctx("/diagnostics", 1000.0), *runner_);
  Flush();
  EXPECT_EQ(transport_->CountIn("robot.diagnostic.degraded"), 1);
}

namespace {

diagnostic_msgs::msg::DiagnosticArray OneStatus(const std::string& name, unsigned char level,
                                                const std::string& message = "") {
  diagnostic_msgs::msg::DiagnosticArray array;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = name;
  status.level = level;
  status.message = message;
  array.status.push_back(status);
  return array;
}

// Events of one name, by the `component` property they carry.
int CountFor(const std::vector<WireEvent>& events, const std::string& name,
             const std::string& component) {
  int found = 0;
  for (const auto& event : events) {
    if (event.name == name && event.properties.value("component", std::string()) == component) {
      ++found;
    }
  }
  return found;
}

}  // namespace

TEST_F(TranslatorTest, TwoDiagnosticTopicsSharingAComponentNameDoNotFlap) {
  // ONE instance serves every DiagnosticArray topic. A component name is unique
  // only within its own publisher, so `motor_driver` steady at OK on one topic and
  // steady at WARN on another is two facts -- and a level remembered by name alone
  // reads them as a transition each way, every message, forever.
  using D = diagnostic_msgs::msg::DiagnosticStatus;
  DiagnosticsTranslator t;
  for (int i = 0; i < 5; ++i) {
    t.Translate(Serialize(OneStatus("motor_driver", D::OK, "nominal")),
                Ctx("/diagnostics", 1000.0 + i), *runner_);
    t.Translate(Serialize(OneStatus("motor_driver", D::WARN, "brush wear")),
                Ctx("/robot1/diagnostics", 1000.0 + i), *runner_);
  }
  Flush();

  // The second topic's status is unhealthy the first time it is seen, which is
  // worth exactly ONE event. Everything after that is steady state on both.
  EXPECT_EQ(transport_->CountIn("robot.diagnostic.degraded"), 1);
  EXPECT_EQ(transport_->CountIn("robot.diagnostic.recovered"), 0)
      << "nothing recovered; only a level shared between two publishers moved";
  const auto events = EventsIn(*transport_);
  const WireEvent* degraded = FindEvent(events, "robot.diagnostic.degraded");
  ASSERT_NE(degraded, nullptr);
  EXPECT_EQ(degraded->properties.value("topic", std::string()), "/robot1/diagnostics");
}

TEST_F(TranslatorTest, ADegradationTheCapDiscardedIsNotFollowedByARecoveryFromNothing) {
  // The cap discards events; a state machine advanced past a discarded one
  // describes a transition the session never saw. Here the `degraded` is
  // suppressed, so remembering ERROR would lose the fault outright and leave the
  // later `recovered` as a recovery from a degradation nothing recorded.
  using D = diagnostic_msgs::msg::DiagnosticStatus;
  EventRateLimiter limiter;
  limiter.set_limit_hz(1.0);
  DiagnosticsTranslator t;
  auto at = [this, &limiter](double when) {
    TranslationContext ctx = Ctx("/diagnostics", when);
    ctx.event_limiter = &limiter;
    return ctx;
  };

  // Spends the one token this source has, at the same instant so nothing refills.
  t.Translate(Serialize(OneStatus("lidar", D::ERROR, "not responding")), at(1000.0), *runner_);
  t.Translate(Serialize(OneStatus("motor_driver", D::ERROR, "overcurrent")), at(1000.0),
              *runner_);
  // Still faulted, and now there is budget for it.
  t.Translate(Serialize(OneStatus("motor_driver", D::ERROR, "overcurrent")), at(1002.0),
              *runner_);
  t.Translate(Serialize(OneStatus("motor_driver", D::OK, "nominal")), at(1004.0), *runner_);
  Flush();

  ASSERT_EQ(limiter.suppressed_for("/diagnostics"), 1u)
      << "the case under test did not happen: nothing was suppressed";
  const auto events = EventsIn(*transport_);
  EXPECT_EQ(CountFor(events, "robot.diagnostic.degraded", "motor_driver"), 1)
      << "the suppressed fault was never re-offered, so the session lost it";
  EXPECT_EQ(CountFor(events, "robot.diagnostic.recovered", "motor_driver"), 1);
}

TEST_F(TranslatorTest, DiagnosticsBoundTheKeyValuePairsTaken) {
  // Values are arbitrary strings from a driver. One chatty component must not
  // be able to dominate the payload.
  DiagnosticsTranslator t;
  diagnostic_msgs::msg::DiagnosticArray msg;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "chatty";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  for (int i = 0; i < 100; ++i) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "k" + std::to_string(i);
    kv.value = "v";
    status.values.push_back(kv);
  }
  msg.status = {status};
  t.Translate(Serialize(msg), Ctx("/diagnostics", 1000.0), *runner_);
  Flush();
  EXPECT_TRUE(transport_->BodyContains("kv.k0"));
  EXPECT_FALSE(transport_->BodyContains("kv.k50")) << "key/value pairs were not bounded";
}

// ---------------------------------------------------------------------------
// Objectives. This is where the interesting bugs live.
// ---------------------------------------------------------------------------

namespace {

action_msgs::msg::GoalStatusArray MakeStatusArray(
    const std::vector<std::pair<std::uint8_t, std::int8_t>>& goals) {
  action_msgs::msg::GoalStatusArray array;
  for (const auto& g : goals) {
    action_msgs::msg::GoalStatus status;
    status.goal_info.goal_id.uuid.fill(0);
    status.goal_info.goal_id.uuid[0] = g.first;
    status.status = g.second;
    array.status_list.push_back(status);
  }
  return array;
}

using GS = action_msgs::msg::GoalStatus;

}  // namespace

TEST_F(TranslatorTest, ObjectiveTracksAcceptedToSucceeded) {
  ObjectiveTracker t;
  const std::string topic = "/navigate_to_pose/_action/status";
  t.Translate(Serialize(MakeStatusArray({{1, GS::STATUS_ACCEPTED}})), Ctx(topic, 1000.0),
              *runner_);
  t.Translate(Serialize(MakeStatusArray({{1, GS::STATUS_EXECUTING}})), Ctx(topic, 1001.0),
              *runner_);
  t.Translate(Serialize(MakeStatusArray({{1, GS::STATUS_SUCCEEDED}})), Ctx(topic, 1047.0),
              *runner_);
  Flush();
  EXPECT_EQ(transport_->CountIn("navigate_to_pose.started"), 1);
  EXPECT_EQ(transport_->CountIn("navigate_to_pose.executing"), 1);
  EXPECT_EQ(transport_->CountIn("navigate_to_pose.succeeded"), 1);
  // Duration measured from first sight to terminal: 47 seconds.
  EXPECT_TRUE(transport_->BodyContains("47.0"));
}

TEST_F(TranslatorTest, ObjectiveReportsAbortedAndCanceled) {
  // The outcomes a simulator will almost never hand you, and the ones that
  // matter most in failure analysis.
  ObjectiveTracker t;
  const std::string topic = "/navigate_to_pose/_action/status";
  t.Translate(Serialize(MakeStatusArray({{1, GS::STATUS_EXECUTING},
                                         {2, GS::STATUS_EXECUTING}})),
              Ctx(topic, 1000.0), *runner_);
  t.Translate(Serialize(MakeStatusArray({{1, GS::STATUS_ABORTED},
                                         {2, GS::STATUS_CANCELED}})),
              Ctx(topic, 1010.0), *runner_);
  Flush();
  EXPECT_EQ(transport_->CountIn("navigate_to_pose.aborted"), 1);
  EXPECT_EQ(transport_->CountIn("navigate_to_pose.canceled"), 1);
}

TEST_F(TranslatorTest, ObjectiveDoesNotDuplicateAnUnchangedStatus) {
  // The status topic republishes the whole array. Emitting per publication
  // rather than per transition would produce one event per second per goal.
  ObjectiveTracker t;
  const std::string topic = "/navigate_to_pose/_action/status";
  for (int i = 0; i < 30; ++i) {
    t.Translate(Serialize(MakeStatusArray({{1, GS::STATUS_EXECUTING}})),
                Ctx(topic, 1000.0 + i), *runner_);
  }
  Flush();
  EXPECT_EQ(transport_->CountIn("navigate_to_pose.executing"), 1);
}

TEST_F(TranslatorTest, ObjectiveReportsAGoalThatVanishesWithoutTerminating) {
  // The status QoS is KEEP_LAST depth 1 and a server drops a goal from the array
  // once result_timeout elapses. So a goal can simply disappear mid-execution.
  // Without this sweep it stays "executing" forever in the session record.
  ObjectiveTracker t;
  t.vanished_goal_timeout_s = 5.0;
  const std::string topic = "/navigate_to_pose/_action/status";
  t.Translate(Serialize(MakeStatusArray({{1, GS::STATUS_EXECUTING}})), Ctx(topic, 1000.0),
              *runner_);
  // Goal 1 is gone from every subsequent array.
  t.Translate(Serialize(MakeStatusArray({{2, GS::STATUS_ACCEPTED}})), Ctx(topic, 1010.0),
              *runner_);
  Flush();
  EXPECT_EQ(transport_->CountIn("navigate_to_pose.outcome_unknown"), 1);
  EXPECT_TRUE(transport_->BodyContains("executing"));
}

TEST_F(TranslatorTest, ObjectiveDoesNotReportVanishedGoalsThatAlreadyTerminated) {
  // The goal is watched through its WHOLE life here -- accepted, then succeeded,
  // then aged out -- so what is pinned is the transition path recording the
  // outcome, not the first-sighting seeding that
  // AGoalAlreadyTerminalOnItsFirstSightingIsNotThisSessionsObjective covers. Seen
  // terminal only on arrival, this test passes with the transition path's
  // bookkeeping deleted, and the sweep would then report a succeeded navigation
  // as an outcome nobody knows.
  ObjectiveTracker t;
  t.vanished_goal_timeout_s = 5.0;
  const std::string topic = "/navigate_to_pose/_action/status";
  t.Translate(Serialize(MakeStatusArray({{1, GS::STATUS_ACCEPTED}})), Ctx(topic, 1000.0),
              *runner_);
  t.Translate(Serialize(MakeStatusArray({{1, GS::STATUS_SUCCEEDED}})), Ctx(topic, 1001.0),
              *runner_);
  // Goal 1 is gone from every later array, and more than the timeout has passed.
  t.Translate(Serialize(MakeStatusArray({{2, GS::STATUS_ACCEPTED}})), Ctx(topic, 1011.0),
              *runner_);
  Flush();
  EXPECT_EQ(transport_->CountIn("navigate_to_pose.succeeded"), 1);
  EXPECT_EQ(transport_->CountIn("navigate_to_pose.outcome_unknown"), 0)
      << "a goal that succeeded then aged out must not be reported as unknown";
}

TEST_F(TranslatorTest, ObjectiveHandlesConcurrentGoalsIndependently) {
  ObjectiveTracker t;
  const std::string topic = "/navigate_to_pose/_action/status";
  t.Translate(Serialize(MakeStatusArray({{1, GS::STATUS_EXECUTING},
                                         {2, GS::STATUS_ACCEPTED},
                                         {3, GS::STATUS_EXECUTING}})),
              Ctx(topic, 1000.0), *runner_);
  t.Translate(Serialize(MakeStatusArray({{1, GS::STATUS_SUCCEEDED},
                                         {2, GS::STATUS_EXECUTING},
                                         {3, GS::STATUS_ABORTED}})),
              Ctx(topic, 1005.0), *runner_);
  Flush();
  EXPECT_EQ(transport_->CountIn("navigate_to_pose.succeeded"), 1);
  EXPECT_EQ(transport_->CountIn("navigate_to_pose.aborted"), 1);
  EXPECT_EQ(t.goals().size(), 3u);
}

TEST_F(TranslatorTest, ObjectiveUuidIsFormattedStably) {
  std::array<std::uint8_t, 16> uuid{};
  uuid[0] = 0xAB;
  uuid[15] = 0xCD;
  const std::string text = ObjectiveTracker::FormatUuid(uuid);
  EXPECT_EQ(text.size(), 36u);
  EXPECT_EQ(text.substr(0, 2), "ab");
  EXPECT_EQ(text.substr(text.size() - 2), "cd");
}

TEST_F(TranslatorTest, RegistryCoversTheCoreTypes) {
  TranslatorRegistry registry;
  RegisterCoreTranslators(registry);
  EXPECT_TRUE(registry.Handles("sensor_msgs/msg/BatteryState"));
  EXPECT_TRUE(registry.Handles("sensor_msgs/msg/JointState"));
  EXPECT_TRUE(registry.Handles("nav_msgs/msg/Odometry"));
  EXPECT_TRUE(registry.Handles("sensor_msgs/msg/Imu"));
  EXPECT_TRUE(registry.Handles("sensor_msgs/msg/LaserScan"));
  EXPECT_TRUE(registry.Handles("diagnostic_msgs/msg/DiagnosticArray"));
  EXPECT_TRUE(registry.Handles("action_msgs/msg/GoalStatusArray"));
  // Nav2 lives in its own optional package, so a robot running neither Nav2 nor
  // MoveIt does not have to install both.
  EXPECT_FALSE(registry.Handles("nav2_msgs/msg/BehaviorTreeLog"));
}

// ---------------------------------------------------------------------------
// Where a translator's events land
// ---------------------------------------------------------------------------

TEST_F(TranslatorTest, ATranslatorEventIsStampedWhereTheRobotWas) {
  // An event at the origin is indistinguishable from an event at the origin, and
  // "where did this happen" is most of what this platform is for.
  DiagnosticsTranslator t;
  diagnostic_msgs::msg::DiagnosticArray msg;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "motor";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  status.message = "stalled";
  msg.status.push_back(status);
  t.Translate(Serialize(msg), CtxAt("/diagnostics", 1000.0, c3d::Vec3{3.0, -4.0, 0.5}),
              *runner_);
  Flush();

  const auto events = EventsIn(*transport_);
  const WireEvent* event = FindEvent(events, "robot.diagnostic.degraded");
  ASSERT_NE(event, nullptr);
  // rep103 on this runner, so the ROS position reaches the wire unchanged.
  EXPECT_DOUBLE_EQ(event->x, 3.0);
  EXPECT_DOUBLE_EQ(event->y, -4.0);
  EXPECT_DOUBLE_EQ(event->z, 0.5);
  EXPECT_FALSE(event->properties.contains("c3d.position_unknown"));
}

TEST_F(TranslatorTest, ATranslatorEventTakesThePoseConvention) {
  // The point an event carries is a position like any other, so it takes the same
  // axis map the pose channel does. Under rep103 -- the fixture's own runner --
  // that map is the identity and proves nothing, which is why this one is unity.
  auto rig = MakeRig(PoseConvention::kUnity);
  DiagnosticsTranslator t;
  diagnostic_msgs::msg::DiagnosticArray msg;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "motor";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  msg.status.push_back(status);
  t.Translate(Serialize(msg), CtxAt("/diagnostics", 1000.0, c3d::Vec3{1.0, 2.0, 3.0}),
              *rig->runner);
  rig->Flush();

  const auto events = EventsIn(*rig->transport);
  const WireEvent* event = FindEvent(events, "robot.diagnostic.degraded");
  ASSERT_NE(event, nullptr);
  // Unity takes ROS (x, y, z) to (-y, z, x). Written out rather than called,
  // so a change to the SDK's own map cannot silently agree with itself.
  EXPECT_DOUBLE_EQ(event->x, -2.0);
  EXPECT_DOUBLE_EQ(event->y, 3.0);
  EXPECT_DOUBLE_EQ(event->z, 1.0);
}

TEST_F(TranslatorTest, AnEventBeforeTheFirstPoseSaysItsPointIsUnknown) {
  // The origin is a real place. An event stamped there before any pose has landed
  // would be a claim, so it carries the property that says it is not one.
  BatteryTranslator t;
  sensor_msgs::msg::BatteryState msg;
  msg.percentage = 0.9f;
  msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  t.Translate(Serialize(msg), Ctx("/battery_state", 1000.0), *runner_);
  msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  t.Translate(Serialize(msg), Ctx("/battery_state", 1001.0), *runner_);
  Flush();

  const auto events = EventsIn(*transport_);
  const WireEvent* event = FindEvent(events, "robot.power.status_changed");
  ASSERT_NE(event, nullptr);
  EXPECT_DOUBLE_EQ(event->x, 0.0);
  EXPECT_DOUBLE_EQ(event->y, 0.0);
  EXPECT_DOUBLE_EQ(event->z, 0.0);
  ASSERT_TRUE(event->properties.contains("c3d.position_unknown"));
  EXPECT_TRUE(event->properties["c3d.position_unknown"].get<bool>());
}

TEST_F(TranslatorTest, TheEventCapIsPerSourceAndCountsWhatItRefuses) {
  EventRateLimiter limiter;
  limiter.set_limit_hz(1.0);
  DiagnosticsTranslator t;
  for (int i = 0; i < 10; ++i) {
    diagnostic_msgs::msg::DiagnosticArray msg;
    diagnostic_msgs::msg::DiagnosticStatus status;
    // A component nobody has seen before, already unhealthy: an event every time,
    // and one that does not depend on what the translator remembers. A single
    // flapping component would NOT do here -- a transition whose event the cap
    // discarded is re-offered rather than replaced, so half the messages would
    // reach the cap and half would find nothing to say.
    status.name = "motor" + std::to_string(i);
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    msg.status.push_back(status);
    TranslationContext ctx = CtxAt("/diagnostics", 1000.0, c3d::Vec3{1.0, 0.0, 0.0});
    ctx.event_limiter = &limiter;
    t.Translate(Serialize(msg), ctx, *runner_);
  }
  Flush();
  EXPECT_EQ(limiter.suppressed_for("/diagnostics"), 9u);
  // Counted by name: the session's own start event shares the stream.
  int recorded = 0;
  for (const auto& event : EventsIn(*transport_)) {
    if (event.name.rfind("robot.diagnostic.", 0) == 0) ++recorded;
  }
  EXPECT_EQ(recorded, 1);
}

// ---------------------------------------------------------------------------
// Objectives across several actions
// ---------------------------------------------------------------------------

TEST_F(TranslatorTest, ObjectiveNamesAreDerivedFromTheActionIgnoringNamespace) {
  // The dashboard matches objectives on the event NAME, so a robot that navigates
  // and docks must produce two objectives rather than one that merges them.
  ObjectiveTracker t;
  t.Translate(Serialize(MakeStatusArray({{1, GS::STATUS_ACCEPTED}})),
              Ctx("/robot1/dock/_action/status", 1000.0), *runner_);
  Flush();
  EXPECT_EQ(transport_->CountIn("\"dock.started\""), 1);

  const auto events = EventsIn(*transport_);
  const WireEvent* event = FindEvent(events, "dock.started");
  ASSERT_NE(event, nullptr);
  EXPECT_EQ(event->properties.value("action", std::string()), "/robot1/dock/_action/status");
  EXPECT_EQ(event->properties.value("action_name", std::string()), "dock");
}

TEST_F(TranslatorTest, ObjectiveAttributesInterleavedGoalsToTheirOwnAction) {
  // ONE tracker instance serves every action on the robot, which is only correct
  // if each goal remembers which topic it arrived on.
  ObjectiveTracker t;
  const std::string nav = "/navigate_to_pose/_action/status";
  const std::string dock = "/dock/_action/status";
  t.Translate(Serialize(MakeStatusArray({{1, GS::STATUS_EXECUTING}})), Ctx(nav, 1000.0),
              *runner_);
  t.Translate(Serialize(MakeStatusArray({{2, GS::STATUS_EXECUTING}})), Ctx(dock, 1001.0),
              *runner_);
  t.Translate(Serialize(MakeStatusArray({{1, GS::STATUS_ABORTED}})), Ctx(nav, 1002.0),
              *runner_);
  t.Translate(Serialize(MakeStatusArray({{2, GS::STATUS_SUCCEEDED}})), Ctx(dock, 1003.0),
              *runner_);
  Flush();

  const auto events = EventsIn(*transport_);
  const WireEvent* aborted = FindEvent(events, "navigate_to_pose.aborted");
  const WireEvent* succeeded = FindEvent(events, "dock.succeeded");
  ASSERT_NE(aborted, nullptr);
  ASSERT_NE(succeeded, nullptr);
  EXPECT_EQ(aborted->properties.value("action", std::string()), nav);
  EXPECT_EQ(succeeded->properties.value("action", std::string()), dock);
  EXPECT_EQ(transport_->CountIn("\"dock.aborted\""), 0);
  EXPECT_EQ(transport_->CountIn("\"navigate_to_pose.succeeded\""), 0);
}

TEST_F(TranslatorTest, ObjectiveAttributesAVanishedGoalToItsOwnAction) {
  // The sweep runs on whichever topic published LAST, which is unrelated to the
  // goal it is reporting. A docking goal that vanishes while navigation is the
  // chatty topic must still be reported as a docking goal.
  ObjectiveTracker t;
  t.vanished_goal_timeout_s = 5.0;
  t.Translate(Serialize(MakeStatusArray({{1, GS::STATUS_EXECUTING}})),
              Ctx("/dock/_action/status", 1000.0), *runner_);
  // Goal 1 is gone from docking's own next array, which is what makes it absent.
  t.Translate(Serialize(MakeStatusArray({{3, GS::STATUS_ACCEPTED}})),
              Ctx("/dock/_action/status", 1001.0), *runner_);
  // The sweep then fires on the navigation topic.
  t.Translate(Serialize(MakeStatusArray({{2, GS::STATUS_ACCEPTED}})),
              Ctx("/navigate_to_pose/_action/status", 1010.0), *runner_);
  Flush();

  const auto events = EventsIn(*transport_);
  const WireEvent* unknown = FindEvent(events, "dock.outcome_unknown");
  ASSERT_NE(unknown, nullptr) << "the vanished goal was attributed to the wrong action";
  EXPECT_EQ(unknown->properties.value("action", std::string()), "/dock/_action/status");
  EXPECT_EQ(unknown->properties.value("action_name", std::string()), "dock");
  EXPECT_EQ(transport_->CountIn("\"navigate_to_pose.outcome_unknown\""), 0);
}

TEST_F(TranslatorTest, AQuietStatusTopicDoesNotMakeItsGoalsVanish) {
  // A status topic publishes when one of ITS goals changes state and is silent
  // otherwise, so a navigation goal that is merely still running leaves its
  // topic quiet for the whole drive. Every recovery the tree runs meanwhile --
  // spin, wait, backup -- publishes on its own. Aged against the clock, the
  // navigation goal is declared vanished on the first of those publications,
  // reported `outcome_unknown` and erased; its real ABORTED then arrives as a
  // first sighting that is already terminal and seeds silently. The outcome is
  // lost twice over, and a five-minute Nav2 goal reaches the platform with no
  // terminal event at all.
  ObjectiveTracker t;
  t.vanished_goal_timeout_s = 5.0;
  const std::string nav = "/navigate_to_pose/_action/status";
  t.Translate(Serialize(MakeStatusArray({{1, GS::STATUS_EXECUTING}})), Ctx(nav, 1000.0),
              *runner_);
  for (int i = 1; i <= 12; ++i) {
    t.Translate(Serialize(MakeStatusArray({{2, GS::STATUS_EXECUTING}})),
                Ctx("/spin/_action/status", 1000.0 + i), *runner_);
  }
  t.Translate(Serialize(MakeStatusArray({{1, GS::STATUS_ABORTED}})), Ctx(nav, 1013.0), *runner_);
  Flush();

  EXPECT_EQ(transport_->CountIn("navigate_to_pose.outcome_unknown"), 0)
      << "a goal still in its own topic's array was reported as vanished";
  EXPECT_EQ(transport_->CountIn("navigate_to_pose.aborted"), 1)
      << "the goal's real outcome was swallowed by the sweep that erased it";
}

TEST_F(TranslatorTest, AVanishedGoalSpendsItsOwnActionsBudgetRatherThanTheSweepsTrigger) {
  // The cap is keyed on the source topic, and the sweep runs on whichever topic
  // published last. Charged to the trigger, a busy action silences another
  // action's outcome -- and the outcome of a goal that vanished is the one event
  // in this translator that nothing else in the run can reconstruct.
  EventRateLimiter limiter;
  limiter.set_limit_hz(1.0);
  ObjectiveTracker t;
  t.vanished_goal_timeout_s = 5.0;
  const std::string nav = "/navigate_to_pose/_action/status";
  const std::string dock = "/dock/_action/status";
  auto at = [this, &limiter](const std::string& topic, double when) {
    TranslationContext ctx = Ctx(topic, when);
    ctx.event_limiter = &limiter;
    return ctx;
  };

  t.Translate(Serialize(MakeStatusArray({{1, GS::STATUS_EXECUTING}})), at(dock, 1000.0),
              *runner_);
  // Goal 1 is gone from docking's own next array, which is what makes it absent.
  t.Translate(Serialize(MakeStatusArray({{3, GS::STATUS_ACCEPTED}})), at(dock, 1001.0), *runner_);
  // Navigation spends its own token here; the sweep it triggers reports the
  // docking goal and must not find that budget already gone.
  t.Translate(Serialize(MakeStatusArray({{2, GS::STATUS_ACCEPTED}})), at(nav, 1010.0), *runner_);
  Flush();

  const auto events = EventsIn(*transport_);
  EXPECT_NE(FindEvent(events, "dock.outcome_unknown"), nullptr)
      << "the sweep spent the triggering action's budget";
  EXPECT_EQ(limiter.suppressed_for(nav), 0u);
}

TEST_F(TranslatorTest, ObjectiveEventsFromATopicWithNoActionNameStillHaveOne) {
  // A status array reaching the tracker off an unconventional topic must not
  // produce an event whose name begins with a dot.
  ObjectiveTracker t;
  t.Translate(Serialize(MakeStatusArray({{1, GS::STATUS_ACCEPTED}})), Ctx("/goal_states", 1000.0),
              *runner_);
  t.Translate(Serialize(MakeStatusArray({{1, GS::STATUS_SUCCEEDED}})), Ctx("/goal_states", 1001.0),
              *runner_);
  Flush();
  EXPECT_EQ(transport_->CountIn("\"action.started\""), 1);
  EXPECT_EQ(transport_->CountIn("\"action.succeeded\""), 1);
}

TEST_F(TranslatorTest, AGoalAlreadyTerminalOnItsFirstSightingIsNotThisSessionsObjective) {
  // The status topic is transient_local, so the first message can be the last goal
  // of a run that ended before this session existed. Emitted, it puts that run's
  // outcome in this one, where a dashboard objective counts it.
  ObjectiveTracker t;
  const std::string topic = "/navigate_to_pose/_action/status";
  t.Translate(Serialize(MakeStatusArray({{1, GS::STATUS_SUCCEEDED}})), Ctx(topic, 1000.0),
              *runner_);
  Flush();
  EXPECT_EQ(transport_->CountIn("navigate_to_pose.succeeded"), 0)
      << "a goal replayed from before the session was recorded as having happened in it";
  EXPECT_EQ(t.goals().size(), 1u) << "the replayed goal was not tracked at all";

  // A goal this session really did see, on the same topic, still reports normally.
  t.Translate(Serialize(MakeStatusArray({{2, GS::STATUS_ACCEPTED}})), Ctx(topic, 1001.0),
              *runner_);
  t.Translate(Serialize(MakeStatusArray({{2, GS::STATUS_SUCCEEDED}})), Ctx(topic, 1009.0),
              *runner_);
  Flush();
  EXPECT_EQ(transport_->CountIn("navigate_to_pose.started"), 1);
  EXPECT_EQ(transport_->CountIn("navigate_to_pose.succeeded"), 1);
}

TEST_F(TranslatorTest, AGoalFirstSeenInProgressIsThisSessionsObjective) {
  // The other half: a replayed goal that has NOT finished is one the session goes
  // on to watch, so it is reported from the state it was found in.
  ObjectiveTracker t;
  const std::string topic = "/navigate_to_pose/_action/status";
  t.Translate(Serialize(MakeStatusArray({{1, GS::STATUS_EXECUTING}})), Ctx(topic, 1000.0),
              *runner_);
  Flush();
  EXPECT_EQ(transport_->CountIn("navigate_to_pose.executing"), 1);

  t.Translate(Serialize(MakeStatusArray({{1, GS::STATUS_SUCCEEDED}})), Ctx(topic, 1012.0),
              *runner_);
  Flush();
  EXPECT_EQ(transport_->CountIn("navigate_to_pose.succeeded"), 1);
}

// One instance per message type: these four name their series by type, so a second topic of one
// of them shares its series, and the per-topic translators never do.
TEST(SharedSeries, ASecondTopicOfAFixedSeriesTypeIsNamedWithTheFirst) {
  TranslatorRegistry registry;
  RegisterCoreTranslators(registry);
  for (const auto& [type, series] : std::vector<std::pair<std::string, std::string>>{
           {"nav_msgs/msg/Odometry", "odom.*"},
           {"sensor_msgs/msg/Imu", "imu.*"},
           {"sensor_msgs/msg/JointState", "joint.* and joints.*"},
           {"sensor_msgs/msg/LaserScan", "scan.*"}}) {
    const std::string warning = SharedSeriesWarning(registry, type, "/first", "/second");
    EXPECT_NE(warning.find("/second and /first are both " + type), std::string::npos) << warning;
    EXPECT_NE(warning.find("(" + series + ")"), std::string::npos) << warning;
    EXPECT_NE(warning.find("Allowlist only one topic of this type"), std::string::npos) << warning;
    EXPECT_EQ(SharedSeriesWarning(registry, type, "/first", "/first"), "");
  }
  for (const std::string type :
       {"std_msgs/msg/Float32", "std_msgs/msg/Int32MultiArray", "sensor_msgs/msg/BatteryState",
        "diagnostic_msgs/msg/DiagnosticArray", "action_msgs/msg/GoalStatusArray",
        "sensor_msgs/msg/Temperature"}) {
    EXPECT_EQ(SharedSeriesWarning(registry, type, "/first", "/second"), "") << type;
  }
}
