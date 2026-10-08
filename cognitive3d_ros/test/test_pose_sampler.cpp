// PoseSampler tests, driven by populating a tf2 Buffer directly. No graph, no
// simulator, fully deterministic, and it lets us construct the cases a
// simulator produces only by luck.

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

#include "c3d/spool.hpp"
#include "cognitive3d_ros/pose_sampler.hpp"
#include "cognitive3d_ros/session_runner.hpp"

using namespace cognitive3d_ros;
namespace fs = std::filesystem;

namespace {

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
  std::vector<std::string> urls, bodies;
  int CountIn(const std::string& needle) const {
    int n = 0;
    for (const auto& b : bodies) {
      std::size_t pos = 0;
      while ((pos = b.find(needle, pos)) != std::string::npos) { ++n; pos += needle.size(); }
    }
    return n;
  }
};

geometry_msgs::msg::TransformStamped MakeTf(const std::string& parent,
                                            const std::string& child, double x,
                                            double y, double z, double stamp_s) {
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = parent;
  t.child_frame_id = child;
  t.header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  t.header.stamp.nanosec =
      static_cast<std::uint32_t>((stamp_s - static_cast<std::int32_t>(stamp_s)) * 1e9);
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = z;
  t.transform.rotation.w = 1.0;
  return t;
}

// `MakeTf` takes no rotation, so a test built on it alone poses everything at
// the identity orientation, where a 90 degree error in the ray is unreachable.
// The helpers below build the rotated transforms the gaze tests need.

geometry_msgs::msg::Quaternion Qxyzw(double x, double y, double z, double w) {
  geometry_msgs::msg::Quaternion q;
  q.x = x; q.y = y; q.z = z; q.w = w;
  return q;
}

geometry_msgs::msg::Quaternion AxisAngle(double ax, double ay, double az, double degrees) {
  const double n = std::sqrt(ax * ax + ay * ay + az * az);
  const double half = degrees * M_PI / 360.0, s = std::sin(half) / n;
  return Qxyzw(ax * s, ay * s, az * s, std::cos(half));
}

geometry_msgs::msg::Quaternion QMul(const geometry_msgs::msg::Quaternion& a,
                                    const geometry_msgs::msg::Quaternion& b) {
  return Qxyzw(a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
               a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
               a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
               a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z);
}

// The fixed body -> optical rotation every ROS camera driver publishes between
// `*_camera_frame` and `*_optical_frame`. Written out rather than derived, so a
// change to the SDK's own constant cannot silently agree with a matching change
// here.
const geometry_msgs::msg::Quaternion kBodyToOptical = Qxyzw(-0.5, 0.5, -0.5, 0.5);

geometry_msgs::msg::TransformStamped MakeTfQ(const std::string& parent,
                                             const std::string& child, double x, double y,
                                             double z,
                                             const geometry_msgs::msg::Quaternion& rotation,
                                             double stamp_s) {
  auto t = MakeTf(parent, child, x, y, z, stamp_s);
  t.transform.rotation = rotation;
  return t;
}

struct V3 { double x, y, z; };

// How far a correct payload's rotation can still read from the direction it should name, from the
// wire's own 5-decimal rounding and nothing else. Four orders of magnitude below the errors this
// exists to catch (a wrong rotation rule is off by 15, 90 or 180 degrees), and tight enough that a
// genuine rotation bug cannot hide under it.
constexpr double kGazeRotationToleranceDegrees = 1e-3;

V3 Rotate(const std::array<double, 4>& q, const V3& v) {
  const double x = q[0], y = q[1], z = q[2], w = q[3];
  const double tx = 2 * (y * v.z - z * v.y);
  const double ty = 2 * (z * v.x - x * v.z);
  const double tz = 2 * (x * v.y - y * v.x);
  return V3{v.x + w * tx + y * tz - z * ty,
            v.y + w * ty + z * tx - x * tz,
            v.z + w * tz + x * ty - y * tx};
}

V3 Rotate(const geometry_msgs::msg::Quaternion& q, const V3& v) {
  return Rotate(std::array<double, 4>{q.x, q.y, q.z, q.w}, v);
}

// The Unity wire's position map, written out rather than taken from the SDK, so a test that
// derives an expected direction with it is independent of the code it checks.
V3 RosToUnityAxes(const V3& ros) { return V3{-ros.y, ros.z, ros.x}; }

// HOW THE PLATFORM READS A GAZE ROTATION, asserted against the bytes that were actually
// serialized rather than against what the sampler was asked to do: as a headset's, so
//
//     R(r) . forward_local
//
// is the direction the recorded frame faces. Session Replay turns the robot it draws from the
// pose timeline by exactly that. `forward_local` is (0,0,1) on the Unity wire and (1,0,0) on a
// rep103 wire, where no change of basis has been applied and REP-103 forward is still +X.
// `expected` is the direction the test's own transforms say the frame faces, in wire axes.
//
// Every record examined must carry no gaze point: the gaze channel sends a position and a
// rotation and nothing else. Returns the worst angular error in degrees, and fails outright on
// zero samples: a check that examined nothing is the failure mode this exists to prevent, not a
// pass.
double WorstGazeRotationErrorDegrees(const CapturingTransport& transport, V3 forward_local,
                                     V3 expected) {
  double worst = 0.0;
  int checked = 0;
  const double expected_length =
      std::sqrt(expected.x * expected.x + expected.y * expected.y + expected.z * expected.z);
  for (std::size_t i = 0; i < transport.urls.size(); ++i) {
    if (transport.urls[i].find("/gaze/") == std::string::npos) continue;
    const auto payload = nlohmann::json::parse(transport.bodies[i]);
    for (const auto& record : payload.value("data", nlohmann::json::array())) {
      EXPECT_FALSE(record.contains("g")) << "a gaze record carries a gaze point: " << record.dump();
      const std::array<double, 4> r{record["r"][0], record["r"][1], record["r"][2],
                                    record["r"][3]};
      // The wire rounds to 5 decimals, so the stored `r` is never exactly unit.
      // BOTH vectors have to be normalised before the dot product: `acos` near 1
      // amplifies a magnitude error into a phantom angle -- acos(1 - e) is
      // sqrt(2e), so a 1e-6 error in |e| reads as 0.08 degrees of rotation that
      // is not there.
      const double norm = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2] + r[3] * r[3]);
      EXPECT_NEAR(norm, 1.0, 1e-3) << "stored rotation is not a unit quaternion, so it "
                                      "names no rotation at all";
      const V3 e = Rotate(r, forward_local);
      const double elen = std::sqrt(e.x * e.x + e.y * e.y + e.z * e.z);
      const double dot =
          (expected.x * e.x + expected.y * e.y + expected.z * e.z) / (expected_length * elen);
      worst = std::max(worst, std::acos(std::max(-1.0, std::min(1.0, dot))) * 180.0 / M_PI);
      ++checked;
    }
  }
  EXPECT_GT(checked, 0) << "no gaze sample was examined, so nothing was verified";
  return worst;
}

// The `properties` object of every `robot.localisation_jump` the transport was
// handed. Read back from the serialized bytes rather than from the sampler,
// because a magnitude the event does not CARRY is a magnitude no consumer has --
// and asserted on parsed numbers rather than by counting a formatted double,
// which binds the test to how nlohmann prints 90 degrees.
std::vector<nlohmann::json> LocalisationJumpProperties(const CapturingTransport& transport) {
  std::vector<nlohmann::json> out;
  for (std::size_t i = 0; i < transport.urls.size(); ++i) {
    if (transport.urls[i].find("/events/") == std::string::npos) continue;
    const auto payload = nlohmann::json::parse(transport.bodies[i]);
    for (const auto& record : payload.value("data", nlohmann::json::array())) {
      if (record.value("name", std::string()) != "robot.localisation_jump") continue;
      out.push_back(record.value("properties", nlohmann::json::object()));
    }
  }
  return out;
}

// Every gaze POSITION the transport was handed, in the order it was recorded.
// Which FRAME a sample came from is not on the wire, so the only way to ask
// whether a pose in the wrong frame was recorded is to read the numbers back.
std::vector<V3> GazePositions(const CapturingTransport& transport) {
  std::vector<V3> out;
  for (std::size_t i = 0; i < transport.urls.size(); ++i) {
    if (transport.urls[i].find("/gaze/") == std::string::npos) continue;
    const auto payload = nlohmann::json::parse(transport.bodies[i]);
    for (const auto& record : payload.value("data", nlohmann::json::array())) {
      if (!record.contains("p")) continue;
      out.push_back(V3{record["p"][0], record["p"][1], record["p"][2]});
    }
  }
  return out;
}

class PoseSamplerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() /
           ("c3d_ps_" + std::to_string(::getpid()) + "_" + std::to_string(counter_++));
    fs::create_directories(dir_);
    c3d::Config cfg;
    cfg.application_key = "K"; cfg.scene_id = "S"; cfg.scene_version = "1";
    cfg.device_id = "robot"; cfg.spool_dir = dir_.string();
    cfg.auto_send_interval_s = 3600.0; cfg.worker_tick_s = 0.01;
    cfg.retry_min_s = 0.02; cfg.retry_max_s = 0.05;
    transport_ = std::make_shared<CapturingTransport>();
    spool_ = std::make_shared<c3d::DiskSpool>(dir_.string(), 64ull << 20,
                                              std::make_shared<c3d::NullLogger>());
    runner_ = std::make_unique<SessionRunner>(cfg, PoseConvention::kRep103, transport_,
                                              spool_, std::make_shared<c3d::NullLogger>());
    // The dynamic object is OFF by default. Every test below that reads the dynamics channel
    // asks about the switched-on behaviour, so the fixture switches it on; the off path has its
    // own tests and builds its own runner.
    runner_->set_robot_dynamic_object(true);
    ASSERT_TRUE(runner_->Begin());
    buffer_ = std::make_shared<tf2_ros::Buffer>(
        std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
    // As the node's buffer is: its TransformListener spins its own thread and
    // marks the buffer so. Without the mark, Humble's tf2_ros refuses every
    // canTransform that takes a timeout argument, zero included, so the body
    // lookup fails and nearly every test here fails with it -- on Humble only,
    // while Jazzy accepts a zero timeout either way.
    buffer_->setUsingDedicatedThread(true);
  }
  void TearDown() override {
    if (runner_) runner_->End("test");
    runner_.reset();
    std::error_code ec;
    fs::remove_all(dir_, ec);
  }
  void Flush() { runner_->session()->Flush(); runner_->session()->PumpUploads(); }

  // A stationary robot one metre along odom's +X, at the given orientation, with
  // BOTH links of the chain stamped at the same time.
  //
  // Restamping the whole chain is not tidiness. tf2's latest common time is the
  // OLDEST of the latest stamps along it, so a `map -> odom` left behind at the
  // first stamp pins every later lookup to the first sample -- the pose never
  // changes, the assertion passes, and nothing was tested.
  void SetChain(double stamp_s, const geometry_msgs::msg::Quaternion& rotation) {
    buffer_->setTransform(MakeTf("map", "odom", 0, 0, 0, stamp_s), "test");
    buffer_->setTransform(MakeTfQ("odom", "base_link", 1.0, 0.0, 0.0, rotation, stamp_s), "test");
  }

  // A spool of its own for a runner a test builds beside the fixture's, in a directory of its
  // own that it names in `cfg`. Every session drains its spool on a thread of its own from the
  // moment it is created, so on the fixture's spool the fixture's session sends some of the
  // runner's chunks through `transport_`, and the transport the test reads never sees them.
  std::shared_ptr<c3d::DiskSpool> OwnSpool(c3d::Config* cfg) {
    cfg->spool_dir = (dir_ / ("own_" + std::to_string(own_spools_++))).string();
    return std::make_shared<c3d::DiskSpool>(cfg->spool_dir, 64ull << 20,
                                            std::make_shared<c3d::NullLogger>());
  }

  std::unique_ptr<SessionRunner> MakeRunner(PoseConvention convention,
                                            std::shared_ptr<CapturingTransport>* transport_out) {
    c3d::Config cfg;
    cfg.application_key = "K"; cfg.scene_id = "S"; cfg.scene_version = "1";
    cfg.device_id = "robot";
    cfg.auto_send_interval_s = 3600.0; cfg.worker_tick_s = 0.01;
    auto spool = OwnSpool(&cfg);
    auto transport = std::make_shared<CapturingTransport>();
    auto runner = std::make_unique<SessionRunner>(cfg, convention, transport, spool,
                                                  std::make_shared<c3d::NullLogger>());
    *transport_out = std::move(transport);
    return runner;
  }

  // kUnity is the production default and the platform's gaze contract is a
  // statement about the Unity wire, so the contract tests run against one of
  // these rather than against the fixture's rep103 runner.
  std::unique_ptr<SessionRunner> MakeUnityRunner(
      std::shared_ptr<CapturingTransport>* transport_out) {
    return MakeRunner(PoseConvention::kUnity, transport_out);
  }

  fs::path dir_;
  std::shared_ptr<CapturingTransport> transport_;
  std::shared_ptr<c3d::DiskSpool> spool_;
  std::unique_ptr<SessionRunner> runner_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  int own_spools_ = 0;
  static int counter_;
};
int PoseSamplerTest::counter_ = 0;

}  // namespace

TEST_F(PoseSamplerTest, ProducesGazeAndADynamicObjectFromTheTfTree) {
  buffer_->setTransform(MakeTf("map", "odom", 0, 0, 0, 100.0), "test");
  buffer_->setTransform(MakeTf("odom", "base_link", 2.0, 3.0, 0.0, 100.0), "test");

  PoseSampler sampler(buffer_, PoseSamplerConfig{});
  std::string status;
  ASSERT_TRUE(sampler.Sample(1000.0, *runner_, &status));
  EXPECT_EQ(status, "map");
  Flush();

  // The gaze stream is the spatial timeline and the only carrier of session
  // properties. Without it the session opens empty.
  bool saw_gaze = false, saw_dynamics = false;
  for (const auto& url : transport_->urls) {
    if (url.find("/gaze/") != std::string::npos) saw_gaze = true;
    if (url.find("/dynamics/") != std::string::npos) saw_dynamics = true;
  }
  EXPECT_TRUE(saw_gaze);
  EXPECT_TRUE(saw_dynamics);
  EXPECT_EQ(transport_->CountIn("robot_placeholder"), 1) << "manifest entry missing";
  EXPECT_EQ(transport_->CountIn("robot.topology"), 1);
}

TEST_F(PoseSamplerTest, TheDynamicObjectIsOffByDefaultAndTheTimelineIsNot) {
  // The default the shipped parameter carries, asserted on the WIRE rather than on the flag: the
  // robot is drawn from the pose timeline, so a run that registers no object still has to produce
  // the gaze stream and the events. A session with no gaze opens empty, which is a worse outcome
  // than the missing robot mesh the off default avoids.
  c3d::Config cfg;
  cfg.application_key = "K"; cfg.scene_id = "S"; cfg.scene_version = "1";
  cfg.device_id = "robot";
  cfg.auto_send_interval_s = 3600.0; cfg.worker_tick_s = 0.01;
  auto spool = OwnSpool(&cfg);
  auto transport = std::make_shared<CapturingTransport>();
  SessionRunner runner(cfg, PoseConvention::kRep103, transport, spool,
                       std::make_shared<c3d::NullLogger>());
  EXPECT_FALSE(runner.robot_dynamic_object()) << "the runner's default must be the node's default";
  ASSERT_TRUE(runner.Begin());

  buffer_->setTransform(MakeTf("map", "odom", 0, 0, 0, 100.0), "test");
  buffer_->setTransform(MakeTf("odom", "base_link", 2.0, 3.0, 0.0, 100.0), "test");
  PoseSampler sampler(buffer_, PoseSamplerConfig{});
  ASSERT_TRUE(sampler.Sample(1000.0, runner, nullptr));
  runner.session()->Flush();
  runner.session()->PumpUploads();

  bool saw_gaze = false, saw_dynamics = false;
  for (const auto& url : transport->urls) {
    if (url.find("/gaze/") != std::string::npos) saw_gaze = true;
    if (url.find("/dynamics/") != std::string::npos) saw_dynamics = true;
  }
  EXPECT_TRUE(saw_gaze) << "the pose timeline is what draws the robot now; it must still be sent";
  EXPECT_FALSE(saw_dynamics) << "no dynamics payload may be sent when no object is registered";
  // The manifest is the registration, and it rides inside the dynamics payload. Its mesh name is
  // asserted separately from the URL because an empty manifest and a missing payload are
  // different failures, and only the mesh name can show a registration that went nowhere else.
  EXPECT_EQ(transport->CountIn("robot_placeholder"), 0) << "the robot was registered anyway";
  EXPECT_EQ(transport->CountIn("robot.topology"), 1) << "events are not part of the switch";
  runner.End("test");
}

TEST_F(PoseSamplerTest, SwitchingTheDynamicObjectOnRestoresIt) {
  // The same sampler, the same tf tree, the same assertions inverted. Without this the test above
  // passes for a sampler that stopped sampling, which is the failure it cannot see on its own.
  buffer_->setTransform(MakeTf("map", "odom", 0, 0, 0, 100.0), "test");
  buffer_->setTransform(MakeTf("odom", "base_link", 2.0, 3.0, 0.0, 100.0), "test");
  ASSERT_TRUE(runner_->robot_dynamic_object());
  PoseSampler sampler(buffer_, PoseSamplerConfig{});
  ASSERT_TRUE(sampler.Sample(1000.0, *runner_, nullptr));
  Flush();

  bool saw_dynamics = false;
  for (const auto& url : transport_->urls) {
    if (url.find("/dynamics/") != std::string::npos) saw_dynamics = true;
  }
  EXPECT_TRUE(saw_dynamics);
  EXPECT_EQ(transport_->CountIn("robot_placeholder"), 1);
}

// Robots have no eyes. A point a fixed distance along the forward axis is not anywhere the robot
// looked, and the platform counts every gaze point it is sent into its gaze analytics, so a gaze
// record is `time`, `p` and `r` and nothing else -- with a gaze frame or without one, under every
// convention. Asserted as the record's whole key set, so any added field fails too.
TEST_F(PoseSamplerTest, AGazeRecordIsTimePositionAndRotationAndNothingElse) {
  for (PoseConvention convention :
       {PoseConvention::kRep103, PoseConvention::kUnity, PoseConvention::kGltfAuthored}) {
    for (const std::string gaze_frame : {"", "head_camera_frame", "cam_optical_frame"}) {
      std::shared_ptr<CapturingTransport> transport;
      auto runner = MakeRunner(convention, &transport);
      ASSERT_TRUE(runner->Begin());
      SetChain(100.0, AxisAngle(0, 0, 1, 30));
      buffer_->setTransform(MakeTfQ("base_link", "head_camera_frame", 0.2, 0.0, 0.3,
                                    AxisAngle(0, 1, 0, 15), 100.0), "test");
      buffer_->setTransform(MakeTfQ("base_link", "cam_optical_frame", 0.2, 0.0, 0.3,
                                    kBodyToOptical, 100.0), "test");
      PoseSamplerConfig cfg;
      cfg.gaze_frame = gaze_frame;
      PoseSampler sampler(buffer_, cfg);
      ASSERT_TRUE(sampler.Sample(1000.0, *runner, nullptr));
      ASSERT_TRUE(sampler.Sample(1000.1, *runner, nullptr));
      EXPECT_EQ(sampler.gaze_frame_lookup_failures(), 0u) << "the frame must resolve, or the "
                                                             "body is what was recorded";
      runner->session()->Flush();
      runner->session()->PumpUploads();

      int checked = 0;
      for (std::size_t i = 0; i < transport->urls.size(); ++i) {
        if (transport->urls[i].find("/gaze/") == std::string::npos) continue;
        const auto payload = nlohmann::json::parse(transport->bodies[i]);
        for (const auto& record : payload.value("data", nlohmann::json::array())) {
          std::vector<std::string> keys;
          for (const auto& item : record.items()) keys.push_back(item.key());
          EXPECT_EQ(keys, (std::vector<std::string>{"p", "r", "time"}))
              << PoseConventionName(convention) << " gaze_frame '" << gaze_frame
              << "': " << record.dump();
          ++checked;
        }
      }
      EXPECT_EQ(checked, 2) << PoseConventionName(convention) << " gaze_frame '" << gaze_frame
                            << "': no gaze record was examined, so nothing was verified";
      runner->End("test");
    }
  }
}

TEST_F(PoseSamplerTest, DetectsARelocalisationJump) {
  // A map-frame pose jumps discretely on every localisation update. A naive
  // path-length integral over map->base_link is inflated by every one of them,
  // so the jump has to be annotated rather than counted as travel.
  buffer_->setTransform(MakeTf("map", "odom", 0, 0, 0, 100.0), "test");
  buffer_->setTransform(MakeTf("odom", "base_link", 1.0, 0.0, 0.0, 100.0), "test");

  PoseSamplerConfig cfg;
  cfg.relocalisation_jump_m = 0.5;
  PoseSampler sampler(buffer_, cfg);
  ASSERT_TRUE(sampler.Sample(1000.0, *runner_, nullptr));

  // Small motion: not a jump.
  buffer_->setTransform(MakeTf("odom", "base_link", 1.2, 0.0, 0.0, 101.0), "test");
  ASSERT_TRUE(sampler.Sample(1001.0, *runner_, nullptr));
  EXPECT_EQ(sampler.relocalisation_jumps(), 0u);

  // The localiser corrects by 3 metres in one step.
  buffer_->setTransform(MakeTf("map", "odom", 3.0, 0.0, 0.0, 102.0), "test");
  ASSERT_TRUE(sampler.Sample(1002.0, *runner_, nullptr));
  EXPECT_EQ(sampler.relocalisation_jumps(), 1u);

  Flush();
  EXPECT_EQ(transport_->CountIn("robot.localisation_jump"), 1);
}

// A localiser can turn the robot without moving it. A Cartographer 2D stack that
// re-registers against the room can re-estimate its own yaw by a right angle
// while the robot stands still, and a detector reading `dx, dy, dz` alone sees
// the 90 degree step as millimetres of translation and no event at all. A
// recorded orientation the robot never took is worse than an inflated odometer:
// the gaze ray is built from the orientation, so it is Session Replay's spatial
// timeline that goes wrong.
TEST_F(PoseSamplerTest, DetectsAJumpThatIsPureRotation) {
  // Both links are restamped on every step. tf2's latest common time is the
  // OLDEST of the latest stamps along the chain, so leaving `map -> odom` behind
  // pins the lookup to the first sample and the test silently verifies nothing.
  SetChain(100.0, AxisAngle(0, 0, 1, 0.0));

  PoseSampler sampler(buffer_, PoseSamplerConfig{});
  ASSERT_TRUE(sampler.Sample(1000.0, *runner_, nullptr));

  // The robot does not move. Only its orientation is re-estimated, by 90 degrees.
  // The translation between these two samples is exactly zero, so no value of
  // `relocalisation_jump_m` whatsoever would catch this one.
  SetChain(101.0, AxisAngle(0, 0, 1, 90.0));
  ASSERT_TRUE(sampler.Sample(1001.0, *runner_, nullptr));
  EXPECT_EQ(sampler.relocalisation_jumps(), 1u);

  Flush();
  const auto jumps = LocalisationJumpProperties(*transport_);
  ASSERT_EQ(jumps.size(), 1u);
  EXPECT_NEAR(jumps[0].value("rotation_deg", 0.0), 90.0, 1e-6)
      << "the event has to carry the magnitude, or a reader cannot tell a "
         "re-estimate from a nudge";
  EXPECT_NEAR(jumps[0].value("distance_m", 1.0), 0.0, 1e-9)
      << "the robot did not move, and the event saying otherwise would send a "
         "reader looking for motion that is not there";
  EXPECT_EQ(jumps[0].value("kind", std::string()), "rotation")
      << "`kind` says which threshold fired; the magnitudes alone cannot, "
         "because a reader does not know what they were compared against";
}

// The other side of the same threshold, and the reason it is not simply small.
// A base spinning in place at 200 deg/s -- faster than a TurtleBot 4's own limit
// of 109 -- covers 20 degrees in the 0.1 s a relocalisation check spans, and every
// one of those samples is the robot doing what it was told. A detector that fired
// here would annotate an honest pirouette as corrupt data six times over.
TEST_F(PoseSamplerTest, ALegitimateFastRotationIsNotAJump) {
  SetChain(100.0, AxisAngle(0, 0, 1, 0.0));

  const PoseSamplerConfig cfg;  // the shipped thresholds, not test-only values
  PoseSampler sampler(buffer_, cfg);
  ASSERT_TRUE(sampler.Sample(1000.0, *runner_, nullptr));

  // Six samples, 20 degrees each: 120 degrees of turn in 0.6 s, more than the
  // 90 of the re-registration above, but never more than 20 in any one step.
  for (int i = 1; i <= 6; ++i) {
    SetChain(100.0 + i, AxisAngle(0, 0, 1, 20.0 * i));
    ASSERT_TRUE(sampler.Sample(1000.0 + 0.1 * i, *runner_, nullptr));
  }
  EXPECT_EQ(sampler.relocalisation_jumps(), 0u)
      << "a rotation the robot actually performed was reported as a localiser fault";

  Flush();
  EXPECT_EQ(transport_->CountIn("robot.localisation_jump"), 0);
}

namespace {

// A stationary robot behind a localiser shaped as nav2_amcl's: `map -> odom` once per scan, every
// 1/7 s, stamped the scan's time plus a 1 s transform tolerance, and `odom -> base_link` at 50 Hz,
// stamped as published. At scan time 5 s the estimate corrects by `jump_m` along x and `jump_deg`
// of yaw. tf2 interpolates `map -> odom` between the two future-dated entries either side of the
// lookup time, so the correction reaches a latest-time lookup as a ramp one scan period long,
// about a second late -- the shape no per-sample threshold sees at a high rate. Sampled at
// `sample_hz` of wire time from 2 s to 8 s, publishing everything due before each sample.
// Returns the number of samples recorded.
int SampleAnAmclCorrection(tf2_ros::Buffer& buffer, PoseSampler& sampler, SessionRunner& runner,
                           double sample_hz, double jump_m, double jump_deg) {
  struct Pending {
    double published;
    geometry_msgs::msg::TransformStamped transform;
  };
  std::vector<Pending> pending;
  for (int k = 0; k <= 400; ++k) {
    pending.push_back({0.02 * k, MakeTf("odom", "base_link", 0.0, 0.0, 0.0, 0.02 * k)});
  }
  for (int k = 0; k <= 56; ++k) {
    const double scan = k / 7.0;
    const bool corrected = k >= 35;  // scan time 5 s
    pending.push_back({scan, MakeTfQ("map", "odom", corrected ? jump_m : 0.0, 0.0, 0.0,
                                     AxisAngle(0, 0, 1, corrected ? jump_deg : 0.0),
                                     scan + 1.0)});
  }
  std::stable_sort(pending.begin(), pending.end(),
                   [](const Pending& a, const Pending& b) { return a.published < b.published; });
  std::size_t next = 0;
  int recorded = 0;
  for (int n = 0;; ++n) {
    const double now = 2.0 + n / sample_hz;
    if (now > 8.0 + 1e-9) break;
    while (next < pending.size() && pending[next].published <= now + 1e-9) {
      buffer.setTransform(pending[next++].transform, "test");
    }
    if (sampler.Sample(1000.0 + now, runner, nullptr)) ++recorded;
  }
  return recorded;
}

struct AmclCase {
  double sample_hz;
  double jump_m;
  double jump_deg;
  const char* kind;
};

}  // namespace

// One correction is one event, of the kind that crossed its threshold, at the default rate and
// at 10 Hz alike. Compared sample to sample at 30 Hz, each step is about a quarter of the
// correction -- 21 degrees, 0.28 m -- and nothing is recorded at all.
class AmclCorrectionTest : public PoseSamplerTest,
                           public ::testing::WithParamInterface<AmclCase> {};

TEST_P(AmclCorrectionTest, ACorrectionTfSpreadsOverSeveralSamplesIsOneJump) {
  const AmclCase c = GetParam();
  PoseSamplerConfig config;  // the shipped thresholds
  config.rate_hz = c.sample_hz;
  PoseSampler sampler(buffer_, config);
  const int recorded = SampleAnAmclCorrection(*buffer_, sampler, *runner_, c.sample_hz, c.jump_m,
                                              c.jump_deg);
  ASSERT_GT(recorded, static_cast<int>(5.5 * c.sample_hz))
      << "the lookups failed, so no correction reached the pose";
  c3d::Vec3 last{};
  ASSERT_TRUE(sampler.last_body_position(&last));
  ASSERT_NEAR(last.x, c.jump_m, 1e-6) << "the correction never reached the pose";

  EXPECT_EQ(sampler.relocalisation_jumps(), 1u);
  Flush();
  const auto jumps = LocalisationJumpProperties(*transport_);
  ASSERT_EQ(jumps.size(), 1u);
  EXPECT_EQ(jumps[0].value("kind", std::string()), c.kind);
}

INSTANTIATE_TEST_SUITE_P(
    RotationPositionAndBoth, AmclCorrectionTest,
    ::testing::Values(AmclCase{30.0, 0.0, 90.0, "rotation"}, AmclCase{30.0, 1.2, 0.0, "position"},
                      AmclCase{30.0, 1.2, 90.0, "both"}, AmclCase{10.0, 0.0, 90.0, "rotation"},
                      AmclCase{10.0, 1.2, 0.0, "position"}),
    [](const ::testing::TestParamInfo<AmclCase>& info) {
      return std::string(info.param.kind) + "_at_" +
             std::to_string(static_cast<int>(info.param.sample_hz)) + "_hz";
    });

// A discrete correction at 30 Hz is ONE event. Comparing every sample with the pose a tenth of a
// second back would see the same step from each of the three samples after it.
TEST_F(PoseSamplerTest, ADiscreteJumpAtThirtyHertzIsOneEvent) {
  PoseSamplerConfig config;
  config.rate_hz = 30.0;
  PoseSampler sampler(buffer_, config);
  for (int n = 0; n <= 30; ++n) {
    SetChain(100.0 + n / 30.0, AxisAngle(0, 0, 1, n < 10 ? 0.0 : 90.0));
    ASSERT_TRUE(sampler.Sample(1000.0 + n / 30.0, *runner_, nullptr)) << n;
  }
  EXPECT_EQ(sampler.relocalisation_jumps(), 1u);
  Flush();
  EXPECT_EQ(transport_->CountIn("robot.localisation_jump"), 1);
}

// A correction spread over longer than one check is still one event: 90 degrees over 1/7 s,
// sampled at 30 Hz, crosses the threshold in one check and leaves less than it to the next.
TEST_F(PoseSamplerTest, ACorrectionLongerThanOneCheckIsOneEvent) {
  PoseSamplerConfig config;
  config.rate_hz = 30.0;
  PoseSampler sampler(buffer_, config);
  for (int n = 0; n <= 30; ++n) {
    const double t = n / 30.0;
    const double ramp = std::clamp((t - 0.3) * 7.0, 0.0, 1.0);
    SetChain(100.0 + t, AxisAngle(0, 0, 1, 90.0 * ramp));
    ASSERT_TRUE(sampler.Sample(1000.0 + t, *runner_, nullptr)) << n;
  }
  EXPECT_EQ(sampler.relocalisation_jumps(), 1u);
  Flush();
  const auto jumps = LocalisationJumpProperties(*transport_);
  ASSERT_EQ(jumps.size(), 1u);
  EXPECT_GT(jumps[0].value("rotation_deg", 0.0), 45.0);
  EXPECT_LT(jumps[0].value("rotation_deg", 0.0), 90.0)
      << "measured over one check, which this correction outlasts";
}

// The check compares poses a tenth of a second apart however fast the samples come, and starts
// again from each check: 200 deg/s for a whole second is 20 degrees per check at 30 Hz and at
// 1000 Hz, and 200 degrees to a comparison that never moved its starting pose.
TEST_F(PoseSamplerTest, ALegitimateFastRotationIsNotAJumpAtAnyRate) {
  for (const double hz : {30.0, 1000.0}) {
    buffer_->clear();
    PoseSamplerConfig config;
    config.rate_hz = hz;
    PoseSampler sampler(buffer_, config);
    const int samples = static_cast<int>(hz);
    for (int n = 0; n <= samples; ++n) {
      const double t = n / hz;
      SetChain(100.0 + t, AxisAngle(0, 0, 1, 200.0 * t));
      ASSERT_TRUE(sampler.Sample(1000.0 + t, *runner_, nullptr)) << hz << " Hz, " << n;
    }
    EXPECT_EQ(sampler.relocalisation_jumps(), 0u) << hz << " Hz";
  }
  Flush();
  EXPECT_EQ(transport_->CountIn("robot.localisation_jump"), 0);
}

// Below 10 Hz every sample is compared with the previous one: a 0.6 m step is a jump on the very
// next sample, and 0.3 m per sample never adds up to one.
TEST_F(PoseSamplerTest, BelowTenHertzEverySampleIsCompared) {
  PoseSamplerConfig config;
  config.rate_hz = 2.0;
  PoseSampler sampler(buffer_, config);
  const auto at = [&](int n, double x) {
    buffer_->setTransform(MakeTf("map", "odom", 0, 0, 0, 100.0 + 0.5 * n), "test");
    buffer_->setTransform(MakeTf("odom", "base_link", x, 0.0, 0.0, 100.0 + 0.5 * n), "test");
    return sampler.Sample(1000.0 + 0.5 * n, *runner_, nullptr);
  };
  for (int n = 0; n <= 3; ++n) ASSERT_TRUE(at(n, 0.3 * n));
  EXPECT_EQ(sampler.relocalisation_jumps(), 0u) << "0.3 m per sample is motion";
  ASSERT_TRUE(at(4, 1.5));
  EXPECT_EQ(sampler.relocalisation_jumps(), 1u) << "a 0.6 m step at 2 Hz";
}

// Nothing is compared across a stale stretch, a change of reference frame, a clock that went
// back or a session restart, even inside one check interval at 30 Hz: each would report the
// difference between two unrelated poses as a jump.
TEST_F(PoseSamplerTest, AStaleStretchInsideACheckIsNotAJump) {
  PoseSamplerConfig config;
  config.rate_hz = 30.0;
  config.stale_transform_after_s = 0.05;
  PoseSampler sampler(buffer_, config);
  SetChain(100.0, Qxyzw(0, 0, 0, 1));
  ASSERT_TRUE(sampler.Sample(1000.0, *runner_, nullptr));
  ASSERT_TRUE(sampler.Sample(1000.0 + 1.0 / 30.0, *runner_, nullptr));
  ASSERT_FALSE(sampler.Sample(1000.0 + 2.0 / 30.0, *runner_, nullptr));
  ASSERT_TRUE(sampler.last_sample_stale());

  buffer_->setTransform(MakeTf("map", "odom", 0, 0, 0, 100.1), "test");
  buffer_->setTransform(MakeTf("odom", "base_link", 5.0, 0.0, 0.0, 100.1), "test");
  ASSERT_TRUE(sampler.Sample(1000.0 + 3.0 / 30.0, *runner_, nullptr));
  buffer_->setTransform(MakeTf("map", "odom", 0, 0, 0, 100.2), "test");
  buffer_->setTransform(MakeTf("odom", "base_link", 5.0, 0.0, 0.0, 100.2), "test");
  ASSERT_TRUE(sampler.Sample(1000.0 + 6.0 / 30.0, *runner_, nullptr));
  EXPECT_EQ(sampler.relocalisation_jumps(), 0u);
}

TEST_F(PoseSamplerTest, AChangeOfReferenceFrameInsideACheckIsNotAJump) {
  PoseSamplerConfig config;
  config.rate_hz = 30.0;
  config.reference_frame_grace_s = 0.0;
  PoseSampler sampler(buffer_, config);
  std::string status;
  SetChain(100.0, Qxyzw(0, 0, 0, 1));
  ASSERT_TRUE(sampler.Sample(1000.0, *runner_, &status)) << status;
  ASSERT_EQ(sampler.active_reference_frame(), "map");

  // `map` goes, and the run falls back to `odom` ...
  buffer_->clear();
  buffer_->setTransform(MakeTf("odom", "base_link", 1.0, 0.0, 0.0, 100.0 + 1.0 / 30.0), "test");
  ASSERT_TRUE(sampler.Sample(1000.0 + 1.0 / 30.0, *runner_, &status)) << status;
  ASSERT_EQ(sampler.active_reference_frame(), "odom");

  // ... and comes back four metres away, which is a different frame's origin, not a jump.
  for (int n = 2; n <= 6; ++n) {
    const double t = n / 30.0;
    buffer_->setTransform(MakeTf("map", "odom", 4.0, 0.0, 0.0, 100.0 + t), "test");
    buffer_->setTransform(MakeTf("odom", "base_link", 1.0, 0.0, 0.0, 100.0 + t), "test");
    ASSERT_TRUE(sampler.Sample(1000.0 + t, *runner_, &status)) << status;
  }
  ASSERT_EQ(sampler.active_reference_frame(), "map");
  EXPECT_EQ(sampler.relocalisation_jumps(), 0u);
}

TEST_F(PoseSamplerTest, AClockThatWentBackStartsTheCheckAgain) {
  PoseSamplerConfig config;
  config.rate_hz = 30.0;
  PoseSampler sampler(buffer_, config);
  SetChain(100.0, Qxyzw(0, 0, 0, 1));
  ASSERT_TRUE(sampler.Sample(1000.0, *runner_, nullptr));

  buffer_->setTransform(MakeTf("map", "odom", 0, 0, 0, 100.05), "test");
  buffer_->setTransform(MakeTf("odom", "base_link", 5.0, 0.0, 0.0, 100.05), "test");
  ASSERT_TRUE(sampler.Sample(999.95, *runner_, nullptr));
  buffer_->setTransform(MakeTf("map", "odom", 0, 0, 0, 100.1), "test");
  buffer_->setTransform(MakeTf("odom", "base_link", 5.0, 0.0, 0.0, 100.1), "test");
  ASSERT_TRUE(sampler.Sample(1000.1, *runner_, nullptr));
  EXPECT_EQ(sampler.relocalisation_jumps(), 0u);
}

TEST_F(PoseSamplerTest, ASessionRestartStartsTheCheckAgain) {
  PoseSamplerConfig config;
  config.rate_hz = 30.0;
  PoseSampler sampler(buffer_, config);
  SetChain(100.0, Qxyzw(0, 0, 0, 1));
  ASSERT_TRUE(sampler.Sample(1000.0, *runner_, nullptr));
  sampler.Reset();

  buffer_->setTransform(MakeTf("map", "odom", 0, 0, 0, 100.1), "test");
  buffer_->setTransform(MakeTf("odom", "base_link", 5.0, 0.0, 0.0, 100.1), "test");
  ASSERT_TRUE(sampler.Sample(1000.1, *runner_, nullptr));
  EXPECT_EQ(sampler.relocalisation_jumps(), 0u);
  Flush();
  EXPECT_EQ(transport_->CountIn("robot.localisation_jump"), 0);
}

TEST_F(PoseSamplerTest, FallsBackToOdomOnceTheGraceForMapHasRunOut) {
  // A robot with no localiser at all has no map frame ever, and it still has to
  // end up recording on odom. What changes is when: not on the first tick, when
  // a localiser merely slower than wheel odometry is indistinguishable from an
  // absent one.
  buffer_->setTransform(MakeTf("odom", "base_link", 1.0, 2.0, 0.0, 100.0), "test");

  const PoseSamplerConfig config;  // the shipped grace, not a test-only value
  PoseSampler sampler(buffer_, config);
  std::string status;
  EXPECT_FALSE(sampler.Sample(1000.0, *runner_, &status));
  EXPECT_TRUE(sampler.holding_for_preferred_frame());
  EXPECT_NE(status.find("map"), std::string::npos)
      << "a hold that does not say what it is waiting for reads as a broken robot";
  EXPECT_EQ(sampler.lookup_failures(), 0u) << "a deliberate hold is not a lookup failure";

  ASSERT_TRUE(sampler.Sample(1000.0 + config.reference_frame_grace_s, *runner_, &status));
  EXPECT_EQ(status, "odom");
  EXPECT_EQ(sampler.active_reference_frame(), "odom");
  EXPECT_FALSE(sampler.holding_for_preferred_frame());
}

// The race every stock Nav2 robot starts with. `odom -> base_link` is published from the first
// wheel tick; `map -> odom` follows once amcl has a scan, about half a second later. Settling on
// the fallback inside that window would put the first gaze sample at the odom origin -- metres
// from where the robot actually is -- and write `robot.reference_frame_unavailable` into a session
// that goes on to be fully localised for its whole length.
TEST_F(PoseSamplerTest, APreferredFrameArrivingJustAfterTheFallbackIsNotAFallbackAtAll) {
  buffer_->setTransform(MakeTf("odom", "base_link", 1.0, 0.0, 0.0, 100.0), "test");
  PoseSampler sampler(buffer_, PoseSamplerConfig{});
  EXPECT_FALSE(sampler.Sample(1000.0, *runner_, nullptr));

  // 500 ms later: one period of a 2 Hz localiser.
  buffer_->setTransform(MakeTf("map", "odom", 5.0, 0.0, 0.0, 101.0), "test");
  buffer_->setTransform(MakeTf("odom", "base_link", 1.0, 0.0, 0.0, 101.0), "test");
  ASSERT_TRUE(sampler.Sample(1000.5, *runner_, nullptr));
  EXPECT_EQ(sampler.active_reference_frame(), "map");

  Flush();
  EXPECT_EQ(transport_->CountIn("robot.reference_frame_unavailable"), 0)
      << "a localiser one tick slower than odometry was reported as absent";
  EXPECT_EQ(transport_->CountIn("robot.reference_frame_changed"), 0)
      << "the run never left a frame, because it never adopted the wrong one";
  // map -> odom is 5 m and odom -> base_link is 1 m, so the map pose is at 6 and
  // an odom pose recorded first would be at 1.
  const auto positions = GazePositions(*transport_);
  ASSERT_FALSE(positions.empty()) << "no gaze sample was examined, so nothing was verified";
  for (const auto& p : positions) {
    EXPECT_NEAR(p.x, 6.0, 1e-3) << "a pose in the odom frame reached the payload";
  }
}

// The grace is configuration, and zero is a reachable value: a robot with no localiser and an
// operator who would rather have the first second of the timeline than the right frame.
TEST_F(PoseSamplerTest, AZeroGraceTakesTheFallbackImmediately) {
  buffer_->setTransform(MakeTf("odom", "base_link", 1.0, 2.0, 0.0, 100.0), "test");
  PoseSamplerConfig config;
  config.reference_frame_grace_s = 0.0;
  PoseSampler sampler(buffer_, config);
  std::string status;
  ASSERT_TRUE(sampler.Sample(1000.0, *runner_, &status));
  EXPECT_EQ(status, "odom");
  EXPECT_FALSE(sampler.holding_for_preferred_frame());
}

// The binary streams pose every frame and scan through LookupSensorPose, and it
// answers the same question the gaze channel asks -- including while the answer
// is still being waited for. `odom -> camera_link` resolves perfectly well
// during the grace, and taking it would put the first seconds of a run's pictures
// in a frame whose origin is wherever the robot booted, beside a pose timeline in
// `map`, with nothing on the row to say which of the two it is.
TEST_F(PoseSamplerTest, ASensorPoseWaitsOutTheGraceTheGazeChannelIsWaitingOut) {
  buffer_->setTransform(MakeTf("odom", "base_link", 1.0, 0.0, 0.0, 100.0), "test");
  buffer_->setTransform(MakeTf("base_link", "camera_link", 0.25, 0.0, 0.0, 100.0), "test");
  const rclcpp::Time stamp(100, 0, RCL_SYSTEM_TIME);

  const PoseSamplerConfig config;  // the shipped grace, not a test-only value
  PoseSampler sampler(buffer_, config);
  c3d::Vec3 position{};
  c3d::Quat rotation{};

  EXPECT_FALSE(sampler.Sample(1000.0, *runner_, nullptr));
  ASSERT_TRUE(sampler.holding_for_preferred_frame());
  EXPECT_FALSE(sampler.LookupSensorPose("camera_link", stamp, &position, &rotation))
      << "the frame would have been recorded in odom while gaze recorded nothing";

  ASSERT_TRUE(sampler.Sample(1000.0 + config.reference_frame_grace_s, *runner_, nullptr));
  ASSERT_EQ(sampler.active_reference_frame(), "odom");
  ASSERT_TRUE(sampler.LookupSensorPose("camera_link", stamp, &position, &rotation))
      << "once the gaze channel has settled there is a frame to pose against";
  EXPECT_NEAR(position.x, 1.25, 1e-6);
}

// The other half: a localised robot loses nothing to the rule above, because the
// gaze channel settles on its first sample and the sensor follows it there.
TEST_F(PoseSamplerTest, ASensorPoseIsInMapAsSoonAsTheGazeChannelIs) {
  buffer_->setTransform(MakeTf("map", "odom", 5.0, 0.0, 0.0, 100.0), "test");
  buffer_->setTransform(MakeTf("odom", "base_link", 1.0, 0.0, 0.0, 100.0), "test");
  buffer_->setTransform(MakeTf("base_link", "camera_link", 0.25, 0.0, 0.0, 100.0), "test");
  const rclcpp::Time stamp(100, 0, RCL_SYSTEM_TIME);

  PoseSampler sampler(buffer_, PoseSamplerConfig{});
  ASSERT_TRUE(sampler.Sample(1000.0, *runner_, nullptr));
  ASSERT_EQ(sampler.active_reference_frame(), "map");

  c3d::Vec3 position{};
  c3d::Quat rotation{};
  ASSERT_TRUE(sampler.LookupSensorPose("camera_link", stamp, &position, &rotation));
  // map -> odom is 5 m, odom -> base_link 1 m, base_link -> camera_link 0.25 m,
  // so an odom-framed answer would read 1.25 rather than 6.25.
  EXPECT_NEAR(position.x, 6.25, 1e-6);
}

TEST_F(PoseSamplerTest, EmitsAnEventWhenTheReferenceFrameChanges) {
  // Switching from odom to map changes the meaning of every subsequent
  // position, so it must not happen silently.
  buffer_->setTransform(MakeTf("odom", "base_link", 1.0, 0.0, 0.0, 100.0), "test");
  const PoseSamplerConfig config;
  PoseSampler sampler(buffer_, config);
  // The grace has to run out first: until it does there is no frame to change FROM.
  const double settled = 1000.0 + config.reference_frame_grace_s;
  EXPECT_FALSE(sampler.Sample(1000.0, *runner_, nullptr));
  ASSERT_TRUE(sampler.Sample(settled, *runner_, nullptr));
  EXPECT_EQ(sampler.active_reference_frame(), "odom");

  buffer_->setTransform(MakeTf("map", "odom", 5.0, 0.0, 0.0, 101.0), "test");
  buffer_->setTransform(MakeTf("odom", "base_link", 1.0, 0.0, 0.0, 101.0), "test");
  ASSERT_TRUE(sampler.Sample(settled + 1.0, *runner_, nullptr));
  EXPECT_EQ(sampler.active_reference_frame(), "map");

  Flush();
  EXPECT_EQ(transport_->CountIn("robot.reference_frame_changed"), 1);
  // This run STARTED on the fallback and later recovered, so both events are correct: it says
  // "you never had map" at the start and "you have it now" when localisation converged.
  EXPECT_EQ(transport_->CountIn("robot.reference_frame_unavailable"), 1);
}

// STARTING on the fallback is the case the transition event cannot see, and it is the worse one:
// the session is anchored to a frame whose origin is wherever the robot booted, so the whole
// trajectory sits at the spawn offset from the scene it is drawn in. `map` does not exist at all
// on a stock robot without amcl or SLAM, so this is the DEFAULT outcome, not an edge case.
TEST_F(PoseSamplerTest, SaysSoWhenThePreferredFrameNeverResolvedAtAll) {
  buffer_->setTransform(MakeTf("odom", "base_link", 1.0, 0.0, 0.0, 100.0), "test");
  const PoseSamplerConfig config;
  PoseSampler sampler(buffer_, config);
  EXPECT_FALSE(sampler.Sample(1000.0, *runner_, nullptr));
  ASSERT_TRUE(sampler.Sample(1000.0 + config.reference_frame_grace_s, *runner_, nullptr));
  EXPECT_EQ(sampler.active_reference_frame(), "odom");

  Flush();
  EXPECT_EQ(transport_->CountIn("robot.reference_frame_unavailable"), 1)
      << "a run that never reached its preferred frame said nothing about it";
  EXPECT_EQ(transport_->CountIn("robot.reference_frame_changed"), 0)
      << "never having been in a frame is not a transition out of it";
}

// The mirror: a run that starts in the frame it asked for has nothing to report.
TEST_F(PoseSamplerTest, SaysNothingWhenThePreferredFrameResolvedImmediately) {
  buffer_->setTransform(MakeTf("map", "odom", 5.0, 0.0, 0.0, 100.0), "test");
  buffer_->setTransform(MakeTf("odom", "base_link", 1.0, 0.0, 0.0, 100.0), "test");
  PoseSampler sampler(buffer_, PoseSamplerConfig{});
  ASSERT_TRUE(sampler.Sample(1000.0, *runner_, nullptr));
  EXPECT_EQ(sampler.active_reference_frame(), "map");

  Flush();
  EXPECT_EQ(transport_->CountIn("robot.reference_frame_unavailable"), 0);
  EXPECT_EQ(transport_->CountIn("robot.reference_frame_changed"), 0);
}

// An empty `reference_frame` is a reachable "fallback only" configuration -- `try_lookup` treats
// it as unconfigured -- so it is a deliberate choice, not a frame the run failed to reach.
TEST_F(PoseSamplerTest, SaysNothingWhenNoPreferredFrameWasAskedFor) {
  buffer_->setTransform(MakeTf("odom", "base_link", 1.0, 0.0, 0.0, 100.0), "test");
  PoseSamplerConfig config;
  config.preferred_reference_frame = "";
  PoseSampler sampler(buffer_, config);
  ASSERT_TRUE(sampler.Sample(1000.0, *runner_, nullptr));
  EXPECT_EQ(sampler.active_reference_frame(), "odom");

  Flush();
  EXPECT_EQ(transport_->CountIn("robot.reference_frame_unavailable"), 0)
      << "reported a frame the operator never asked for";
}

TEST_F(PoseSamplerTest, AFlappingReferenceFrameCannotSpendMoreThanTheEventCap) {
  // The pose channel is a source like any other. A localiser dropping in and out
  // produces an event per sample, and uncapped it would crowd out -- and past the
  // session's record bound, DELETE -- whatever else the run had to say.
  EventRateLimiter limiter;
  limiter.set_limit_hz(1.0);
  PoseSampler sampler(buffer_, PoseSamplerConfig{});
  sampler.set_event_limiter(&limiter);

  double stamp = 100.0;
  buffer_->setTransform(MakeTf("map", "odom", 5.0, 0.0, 0.0, stamp), "test");
  buffer_->setTransform(MakeTf("odom", "base_link", 1.0, 0.0, 0.0, stamp), "test");
  // Starts in the frame it asked for, so this sample reports no frame decision; it
  // spends the first token on `robot.topology`, which is what the bucket refills
  // from over the second that follows.
  ASSERT_TRUE(sampler.Sample(1000.0, *runner_, nullptr));

  // Twelve frame changes in one instant, so nothing refills between them.
  for (int i = 0; i < 6; ++i) {
    buffer_->clear();
    buffer_->setTransform(MakeTf("odom", "base_link", 1.0, 0.0, 0.0, ++stamp), "test");
    ASSERT_TRUE(sampler.Sample(1002.0, *runner_, nullptr));
    EXPECT_EQ(sampler.active_reference_frame(), "odom");
    buffer_->setTransform(MakeTf("map", "odom", 5.0, 0.0, 0.0, ++stamp), "test");
    buffer_->setTransform(MakeTf("odom", "base_link", 1.0, 0.0, 0.0, stamp), "test");
    ASSERT_TRUE(sampler.Sample(1002.0, *runner_, nullptr));
    EXPECT_EQ(sampler.active_reference_frame(), "map");
  }

  Flush();
  EXPECT_EQ(transport_->CountIn("robot.reference_frame_changed"), 1)
      << "the pose channel's events are not going through the per-source cap";
  EXPECT_EQ(limiter.suppressed_for(kPoseSamplerEventSource), 11u);
  EXPECT_EQ(limiter.suppressed(), 11u);
  // The cap takes events and nothing else: every sample is still on the timeline.
  EXPECT_EQ(GazePositions(*transport_).size(), 13u);
}

TEST_F(PoseSamplerTest, TopologyIsRetriedWhenTheFrameReportSpentTheOnlyToken) {
  // The frame decision and the topology fall on the SAME sample, so at a cap of
  // 1 Hz they compete for one token and the topology loses. It is emitted once
  // per session and nothing re-offers it, so treating the attempt as the emission
  // costs the session the only copy it will ever have -- and a run that starts on
  // the fallback, which is what makes the two coincide, is the default outcome on
  // a robot without amcl or SLAM rather than an edge case.
  EventRateLimiter limiter;
  limiter.set_limit_hz(1.0);
  const PoseSamplerConfig config;
  PoseSampler sampler(buffer_, config);
  sampler.set_event_limiter(&limiter);

  buffer_->setTransform(MakeTf("odom", "base_link", 1.0, 0.0, 0.0, 100.0), "test");
  const double settled = 1000.0 + config.reference_frame_grace_s;
  EXPECT_FALSE(sampler.Sample(1000.0, *runner_, nullptr));
  ASSERT_TRUE(sampler.Sample(settled, *runner_, nullptr));
  Flush();
  EXPECT_EQ(transport_->CountIn("robot.reference_frame_unavailable"), 1);
  ASSERT_EQ(transport_->CountIn("robot.topology"), 0)
      << "the premise of this test is that the token went to the frame report";

  // One second on, the bucket has refilled and the retry lands.
  ASSERT_TRUE(sampler.Sample(settled + 1.0, *runner_, nullptr));
  Flush();
  EXPECT_EQ(transport_->CountIn("robot.topology"), 1)
      << "a suppressed topology was never offered again";
  EXPECT_EQ(transport_->CountIn("robot.reference_frame_unavailable"), 1)
      << "the frame report is a decision, not a per-sample event";
}

TEST_F(PoseSamplerTest, ReportsFailureRatherThanCrashingWithNoTransforms) {
  PoseSampler sampler(buffer_, PoseSamplerConfig{});
  std::string status;
  EXPECT_FALSE(sampler.Sample(1000.0, *runner_, &status));
  EXPECT_FALSE(status.empty()) << "the reason must be reportable to the operator";
  EXPECT_EQ(sampler.lookup_failures(), 1u);
}

TEST_F(PoseSamplerTest, UnityConventionIsAppliedToWhatIsRecorded) {
  buffer_->setTransform(MakeTf("map", "odom", 0, 0, 0, 100.0), "test");
  buffer_->setTransform(MakeTf("odom", "base_link", 1.0, 2.0, 3.0, 100.0), "test");

  c3d::Config cfg;
  cfg.application_key = "K"; cfg.scene_id = "S"; cfg.scene_version = "1";
  cfg.device_id = "robot";
  cfg.auto_send_interval_s = 3600.0; cfg.worker_tick_s = 0.01;
  auto spool = OwnSpool(&cfg);
  auto transport = std::make_shared<CapturingTransport>();
  SessionRunner unity_runner(cfg, PoseConvention::kUnity, transport, spool,
                             std::make_shared<c3d::NullLogger>());
  ASSERT_TRUE(unity_runner.Begin());

  PoseSampler sampler(buffer_, PoseSamplerConfig{});
  ASSERT_TRUE(sampler.Sample(1000.0, unity_runner, nullptr));
  unity_runner.session()->Flush();
  unity_runner.session()->PumpUploads();

  // ROS (1, 2, 3) becomes Unity (-2, 3, 1).
  bool found = false;
  for (const auto& b : transport->bodies) {
    if (b.find("[-2.0,3.0,1.0]") != std::string::npos) found = true;
  }
  EXPECT_TRUE(found) << "the Unity conversion was not applied at the recording boundary";
  unity_runner.End("test");
}

TEST_F(PoseSamplerTest, MeshYawOffsetRotatesTheModelButNotTheGazeRotation) {
  // An artist-authored or URDF mesh often faces sideways relative to base_link.
  // A model whose wheels sit at x = +/-33.5 and z = +/-27.1, each disc thin
  // along Z, is one: its axles run along Z and its
  // forward axis is X, not the +X-forward robot convention... which means the
  // model renders driving crabwise.
  //
  // The fix has to apply to the dynamic object ONLY. Rotating the pose instead
  // turns the gaze viewpoint off the path, which just relocates the bug.
  buffer_->setTransform(MakeTf("map", "odom", 0, 0, 0, 100.0), "test");
  buffer_->setTransform(MakeTf("odom", "base_link", 0.0, 0.0, 0.0, 100.0), "test");

  PoseSamplerConfig cfg;
  cfg.mesh_yaw_offset_deg = 90.0;
  PoseSampler sampler(buffer_, cfg);
  ASSERT_TRUE(sampler.Sample(1000.0, *runner_, nullptr));
  Flush();

  // Identity orientation, so the robot faces ROS +X and the gaze rotation must
  // still be the identity, unaffected by the mesh offset.
  bool gaze_ok = false, dynamic_rotated = false;
  for (std::size_t i = 0; i < transport_->urls.size(); ++i) {
    const std::string& url = transport_->urls[i];
    const std::string& body = transport_->bodies[i];
    if (url.find("/gaze/") != std::string::npos) {
      const auto payload = nlohmann::json::parse(body);
      for (const auto& record : payload.value("data", nlohmann::json::array())) {
        if (record["r"] == nlohmann::json::array({0.0, 0.0, 0.0, 1.0})) gaze_ok = true;
      }
    }
    // A 90 degree yaw about ROS Z is (0, 0, sin45, cos45) ~ (0,0,0.70711,0.70711).
    if (url.find("/dynamics/") != std::string::npos &&
        body.find("0.70711") != std::string::npos) {
      dynamic_rotated = true;
    }
  }
  EXPECT_TRUE(gaze_ok) << "the mesh offset must not turn the gaze rotation";
  EXPECT_TRUE(dynamic_rotated) << "the dynamic object should carry the mesh offset";
}

TEST_F(PoseSamplerTest, ZeroMeshOffsetLeavesTheRotationUntouched) {
  buffer_->setTransform(MakeTf("map", "odom", 0, 0, 0, 100.0), "test");
  buffer_->setTransform(MakeTf("odom", "base_link", 1.0, 0.0, 0.0, 100.0), "test");
  PoseSampler sampler(buffer_, PoseSamplerConfig{});
  ASSERT_TRUE(sampler.Sample(1000.0, *runner_, nullptr));
  Flush();
  for (std::size_t i = 0; i < transport_->urls.size(); ++i) {
    if (transport_->urls[i].find("/dynamics/") == std::string::npos) continue;
    EXPECT_EQ(transport_->bodies[i].find("0.70711"), std::string::npos)
        << "no offset was configured";
  }
}

TEST_F(PoseSamplerTest, TheUpOffsetLiftsTheDynamicObjectAndNothingElse) {
  // A vertical trim for a mesh whose origin is not where the consumer wants the
  // model to sit. Three things have to hold at once and only the first is
  // obvious: it moves the dynamic object, it moves it along UP, and it moves
  // NOTHING else -- lifting the gaze origin off the robot would break the gaze
  // contract, and lifting an event would float it above the floor.
  buffer_->setTransform(MakeTf("map", "odom", 0, 0, 0, 100.0), "test");
  buffer_->setTransform(MakeTf("odom", "base_link", 1.0, 2.0, 3.0, 100.0), "test");

  c3d::Config cfg;
  cfg.application_key = "K"; cfg.scene_id = "S"; cfg.scene_version = "1";
  cfg.device_id = "robot";
  cfg.auto_send_interval_s = 3600.0; cfg.worker_tick_s = 0.01;
  auto spool = OwnSpool(&cfg);
  auto transport = std::make_shared<CapturingTransport>();
  SessionRunner runner(cfg, PoseConvention::kGltfAuthored, transport, spool,
                       std::make_shared<c3d::NullLogger>());
  runner.set_robot_dynamic_object(true);
  ASSERT_TRUE(runner.Begin());
  runner.set_dynamic_up_offset_meters(0.25);
  ASSERT_EQ(runner.dynamic_up_offset_meters(), 0.25);

  PoseSampler sampler(buffer_, PoseSamplerConfig{});
  ASSERT_TRUE(sampler.Sample(1000.0, runner, nullptr));
  runner.RecordEventAt(1000.0, "on.the.floor", c3d::Vec3{1.0, 2.0, 3.0}, {}, "");
  runner.session()->Flush();
  runner.session()->PumpUploads();

  // gltf_authored sends ROS (x, y, z) as (-x, z, -y), so ROS (1, 2, 3) is
  // (-1, 3, -2) and the viewer's up axis is the middle component. The dynamic
  // object must read 3.25 there; gaze and the event must read 3.0.
  bool dynamic_lifted = false, gaze_unmoved = false, event_unmoved = false;
  for (std::size_t i = 0; i < transport->urls.size(); ++i) {
    const std::string& url = transport->urls[i];
    const std::string& body = transport->bodies[i];
    if (url.find("/dynamics/") != std::string::npos &&
        body.find("[-1.0,3.25,-2.0]") != std::string::npos) {
      dynamic_lifted = true;
    }
    if (url.find("/gaze/") != std::string::npos &&
        body.find("[-1.0,3.0,-2.0]") != std::string::npos) {
      gaze_unmoved = true;
    }
    if (url.find("/events/") != std::string::npos &&
        body.find("[-1.0,3.0,-2.0]") != std::string::npos) {
      event_unmoved = true;
    }
  }
  EXPECT_TRUE(dynamic_lifted) << "the offset never reached the dynamic channel";
  EXPECT_TRUE(gaze_unmoved) << "the offset lifted the gaze origin off the robot";
  EXPECT_TRUE(event_unmoved) << "the offset floated an event off the floor";
  runner.End("test");
}

TEST_F(PoseSamplerTest, TheUpOffsetIsUpUnderRep103Too) {
  // Applied AFTER the conversion the offset has to name each target frame's up
  // axis, and rep103 does not convert at all -- so the component a Y-up
  // convention calls up is ROS LEFT there, and a vertical trim would shift the
  // model sideways. Adding it along ROS +Z before the conversion is what makes
  // this test possible to write.
  buffer_->setTransform(MakeTf("map", "odom", 0, 0, 0, 100.0), "test");
  buffer_->setTransform(MakeTf("odom", "base_link", 1.0, 2.0, 3.0, 100.0), "test");

  runner_->set_dynamic_up_offset_meters(0.5);
  PoseSampler sampler(buffer_, PoseSamplerConfig{});
  ASSERT_TRUE(sampler.Sample(1000.0, *runner_, nullptr));
  Flush();

  bool lifted = false;
  for (std::size_t i = 0; i < transport_->urls.size(); ++i) {
    if (transport_->urls[i].find("/dynamics/") == std::string::npos) continue;
    if (transport_->bodies[i].find("[1.0,2.0,3.5]") != std::string::npos) lifted = true;
    EXPECT_EQ(transport_->bodies[i].find("[1.0,2.5,3.0]"), std::string::npos)
        << "the offset went along ROS LEFT instead of ROS UP";
  }
  EXPECT_TRUE(lifted) << "rep103 up is +Z, so the dynamic object must read z = 3.5";
}

TEST_F(PoseSamplerTest, NoUpOffsetLeavesTheDynamicObjectWhereTheRobotIs) {
  // The default is 0.0 and must be exactly that: a consumer whose mesh origin is
  // already right never configures this, and a silent default height is a
  // defect nothing downstream would reveal.
  buffer_->setTransform(MakeTf("map", "odom", 0, 0, 0, 100.0), "test");
  buffer_->setTransform(MakeTf("odom", "base_link", 1.0, 2.0, 3.0, 100.0), "test");

  EXPECT_EQ(runner_->dynamic_up_offset_meters(), 0.0);
  PoseSampler sampler(buffer_, PoseSamplerConfig{});
  ASSERT_TRUE(sampler.Sample(1000.0, *runner_, nullptr));
  Flush();

  bool at_the_robot = false;
  for (std::size_t i = 0; i < transport_->urls.size(); ++i) {
    if (transport_->urls[i].find("/dynamics/") == std::string::npos) continue;
    if (transport_->bodies[i].find("[1.0,2.0,3.0]") != std::string::npos) at_the_robot = true;
  }
  EXPECT_TRUE(at_the_robot) << "with no offset the dynamic object is the robot's own pose";
}

// ---------------------------------------------------------------------------
// The gaze rotation. Every test above this line uses an identity orientation and
// no gaze frame, so none of them can see a 90 degree gaze error that
// configuration alone produces. The tests below can.
// ---------------------------------------------------------------------------

TEST_F(PoseSamplerTest, ConventionParsingAndAutoResolution) {
  FrameConvention c;
  EXPECT_TRUE(ParseFrameConvention("auto", &c));
  EXPECT_EQ(c, FrameConvention::kAuto);
  EXPECT_TRUE(ParseFrameConvention("", &c));
  EXPECT_EQ(c, FrameConvention::kAuto) << "an unset parameter means auto";
  EXPECT_TRUE(ParseFrameConvention("optical_forward_z", &c));
  EXPECT_EQ(c, FrameConvention::kOpticalForwardZ);
  EXPECT_TRUE(ParseFrameConvention("body", &c));
  EXPECT_EQ(c, FrameConvention::kBodyForwardX);
  EXPECT_FALSE(ParseFrameConvention("forward", &c)) << "a typo must be rejected loudly";

  // A TurtleBot 4 publishes both, and the optical one is the one that means
  // "where the camera looks".
  EXPECT_TRUE(FrameNameLooksOptical("oakd_rgb_camera_optical_frame"));
  EXPECT_TRUE(FrameNameLooksOptical("front_camera_optical"));
  // The names a remapped multi-robot graph actually produces. A suffix-only match
  // misses all three, and a miss is silent: the contract still holds, and the
  // startup warning is gated on exactly the condition a miss makes false.
  EXPECT_TRUE(FrameNameLooksOptical("camera_optical_frame_2"));
  EXPECT_TRUE(FrameNameLooksOptical("oakd/RGB_CAMERA_OPTICAL_FRAME"));
  EXPECT_TRUE(FrameNameLooksOptical("robot1/oakd_rgb_camera_optical_frame"));
  EXPECT_TRUE(FrameNameLooksOptical("optical"));
  EXPECT_FALSE(FrameNameLooksOptical("oakd_rgb_camera_frame"));
  EXPECT_FALSE(FrameNameLooksOptical("base_link"));
  EXPECT_FALSE(FrameNameLooksOptical(""));
  // "optical" must be a whole token, or a frame merely mentioning it in another
  // word would be silently re-interpreted.
  EXPECT_FALSE(FrameNameLooksOptical("opticalish_frame"));
  EXPECT_FALSE(FrameNameLooksOptical("fibreoptical"));
  // A PX4/mavros optical FLOW sensor is body-style and carries the token, so
  // the token match has to exclude it or a working frame becomes a silent
  // 90 degree error.
  EXPECT_FALSE(FrameNameLooksOptical("optical_flow_link"));
  EXPECT_FALSE(FrameNameLooksOptical("optical_flow_frame"));
  EXPECT_FALSE(FrameNameLooksOptical("px4/OPTICAL_FLOW_FRAME"));
  // ...but the exclusion is the ADJACENCY, not a `flow` token anywhere. Scoped
  // to the whole string it would defeat exactly the namespaced frames the token
  // match exists for, and the guard warning could not fire because it is
  // gated on this same predicate. Every one of these is a real camera.
  EXPECT_TRUE(FrameNameLooksOptical("robot_flow/oakd_rgb_camera_optical_frame"));
  EXPECT_TRUE(FrameNameLooksOptical("flow_1/camera_optical_frame"));
  EXPECT_TRUE(FrameNameLooksOptical("flow_cam_optical_frame"));
  EXPECT_TRUE(FrameNameLooksOptical("optical_flow_optical_frame"));
  // px4flow_link tokenises to "px4flow", never to "flow", so the exclusion
  // never touches it -- pinned so a tokeniser change shows up.
  EXPECT_FALSE(FrameNameLooksOptical("px4flow_link"));

  EXPECT_EQ(ResolveFrameConvention("oakd_rgb_camera_optical_frame", FrameConvention::kAuto),
            FrameConvention::kOpticalForwardZ);
  EXPECT_EQ(ResolveFrameConvention("oakd_rgb_camera_frame", FrameConvention::kAuto),
            FrameConvention::kBodyForwardX);
  // An explicit choice always wins, including a wrong-looking one: the operator
  // may know something the naming convention does not.
  EXPECT_EQ(ResolveFrameConvention("oakd_rgb_camera_optical_frame", FrameConvention::kBodyForwardX),
            FrameConvention::kBodyForwardX);
}

TEST_F(PoseSamplerTest, TheGazeRotationFacesTheRobotsForwardForANonPlanarRobotPose) {
  // A wheeled robot on a flat floor only ever produces yaw, and several wrong
  // rotation rules are exact under pure yaw. So the pose that pins the rule has
  // to have pitch and roll in it -- without them a 90 degree error passes.
  std::shared_ptr<CapturingTransport> transport;
  auto runner = MakeUnityRunner(&transport);
  ASSERT_TRUE(runner->Begin());

  const auto rotation = QMul(QMul(AxisAngle(0, 0, 1, 60), AxisAngle(0, 1, 0, 25)),
                             AxisAngle(1, 0, 0, 15));
  buffer_->setTransform(MakeTf("map", "odom", 0, 0, 0, 100.0), "test");
  buffer_->setTransform(MakeTfQ("odom", "base_link", 1.0, 2.0, 0.5, rotation, 100.0), "test");

  PoseSampler sampler(buffer_, PoseSamplerConfig{});
  ASSERT_TRUE(sampler.Sample(1000.0, *runner, nullptr));
  runner->session()->Flush();
  runner->session()->PumpUploads();

  EXPECT_LT(WorstGazeRotationErrorDegrees(*transport, V3{0, 0, 1},
                                          RosToUnityAxes(Rotate(rotation, V3{1, 0, 0}))),
            kGazeRotationToleranceDegrees);
  runner->End("test");
}

TEST_F(PoseSamplerTest, TheGazeRotationFacesTheRobotsForwardInTheRep103BasisToo) {
  // The rep103 wire applies no change of basis, so REP-103 forward is still +X.
  // Asserting both bases keeps the rule a property of the sampler rather than a
  // coincidence of one conversion.
  const auto rotation = QMul(AxisAngle(0, 0, 1, 40), AxisAngle(0, 1, 0, 20));
  buffer_->setTransform(MakeTf("map", "odom", 0, 0, 0, 100.0), "test");
  buffer_->setTransform(MakeTfQ("odom", "base_link", 1.0, 2.0, 0.0, rotation, 100.0), "test");

  PoseSampler sampler(buffer_, PoseSamplerConfig{});
  ASSERT_TRUE(sampler.Sample(1000.0, *runner_, nullptr));
  Flush();
  EXPECT_LT(WorstGazeRotationErrorDegrees(*transport_, V3{1, 0, 0},
                                          Rotate(rotation, V3{1, 0, 0})),
            kGazeRotationToleranceDegrees);
}

TEST_F(PoseSamplerTest, TheGazeFrameFailureNoticeSurvivesAResetAndCannotGoSilent) {
  // The defect this method exists to make impossible. The bar lives here with
  // the counter, and Reset() clears both. A bar that outlived a per-session
  // counter would, after any restart -- a bag loop, a deactivate/activate cycle
  // -- compare `0 > previous total`, false forever, and the operator warning
  // would go permanently silent on a run where every gaze sample comes from the
  // wrong frame. So this test is the cycle: notice, reset, notice again.
  buffer_->setTransform(MakeTf("map", "odom", 0, 0, 0, 100.0), "test");
  buffer_->setTransform(MakeTf("odom", "base_link", 1.0, 2.0, 0.0, 100.0), "test");

  PoseSamplerConfig cfg;
  cfg.gaze_frame = "a_frame_that_does_not_exist";
  PoseSampler sampler(buffer_, cfg);

  EXPECT_FALSE(sampler.ConsumeGazeFrameFailureNotice()) << "nothing has failed yet";
  ASSERT_TRUE(sampler.Sample(1000.0, *runner_, nullptr));
  EXPECT_TRUE(sampler.ConsumeGazeFrameFailureNotice());
  EXPECT_FALSE(sampler.ConsumeGazeFrameFailureNotice()) << "one failure, one notice";
  ASSERT_TRUE(sampler.Sample(1001.0, *runner_, nullptr));
  EXPECT_TRUE(sampler.ConsumeGazeFrameFailureNotice()) << "a further failure notices again";

  // The session restarts. The counter goes to zero and so must the bar, or the
  // next run can never out-count the previous one.
  sampler.Reset();
  EXPECT_FALSE(sampler.ConsumeGazeFrameFailureNotice()) << "nothing has failed in the new session";
  ASSERT_TRUE(sampler.Sample(1002.0, *runner_, nullptr));
  EXPECT_TRUE(sampler.ConsumeGazeFrameFailureNotice())
      << "THE DEFECT: a bar that outlived the counter stayed above it forever";
}

TEST_F(PoseSamplerTest, AResolvableGazeFrameCountsNoFailures) {
  buffer_->setTransform(MakeTf("map", "odom", 0, 0, 0, 100.0), "test");
  buffer_->setTransform(MakeTf("odom", "base_link", 1.0, 2.0, 0.0, 100.0), "test");
  buffer_->setTransform(MakeTfQ("base_link", "cam_optical_frame", 0.2, 0.0, 0.3,
                                kBodyToOptical, 100.0), "test");
  PoseSamplerConfig cfg;
  cfg.gaze_frame = "cam_optical_frame";
  PoseSampler sampler(buffer_, cfg);
  ASSERT_TRUE(sampler.Sample(1000.0, *runner_, nullptr));
  EXPECT_EQ(sampler.gaze_frame_lookup_failures(), 0u)
      << "or the counter fires for a working camera and means nothing";
  EXPECT_FALSE(sampler.ConsumeGazeFrameFailureNotice()) << "and no operator warning either";
}

TEST_F(PoseSamplerTest, TheGazeRotationIsABodyStyleCameraFramesOwn) {
  // A camera that is offset and tilted relative to base_link is the ordinary
  // case. The gaze rotation has to be the camera's: the body's beside the
  // camera's position is wrong by exactly the tilt, 15 degrees of pitch.
  std::shared_ptr<CapturingTransport> transport;
  auto runner = MakeUnityRunner(&transport);
  ASSERT_TRUE(runner->Begin());

  buffer_->setTransform(MakeTf("map", "odom", 0, 0, 0, 100.0), "test");
  buffer_->setTransform(MakeTfQ("odom", "base_link", 1.0, 2.0, 0.0,
                                AxisAngle(0, 0, 1, 30), 100.0), "test");
  buffer_->setTransform(MakeTfQ("base_link", "head_camera_frame", 0.2, 0.0, 0.3,
                                AxisAngle(0, 1, 0, 15), 100.0), "test");

  PoseSamplerConfig cfg;
  cfg.gaze_frame = "head_camera_frame";
  PoseSampler sampler(buffer_, cfg);
  EXPECT_EQ(sampler.resolved_gaze_frame_convention(), FrameConvention::kBodyForwardX)
      << "a frame not named like an optical frame must resolve to body";
  ASSERT_TRUE(sampler.Sample(1000.0, *runner, nullptr));
  runner->session()->Flush();
  runner->session()->PumpUploads();

  const V3 camera_forward =
      Rotate(QMul(AxisAngle(0, 0, 1, 30), AxisAngle(0, 1, 0, 15)), V3{1, 0, 0});
  EXPECT_LT(WorstGazeRotationErrorDegrees(*transport, V3{0, 0, 1}, RosToUnityAxes(camera_forward)),
            kGazeRotationToleranceDegrees);
  runner->End("test");
}

TEST_F(PoseSamplerTest, TheGazeRotationFacesWhereAnOpticalCameraFrameLooks) {
  // An optical frame is +Z forward, +X right, +Y down. Reading its +X as
  // forward, as a body frame's would be, is exactly 90 degrees wrong -- and it
  // is the frame an operator reaches for, because it is the one that means
  // "where the camera looks".
  std::shared_ptr<CapturingTransport> transport;
  auto runner = MakeUnityRunner(&transport);
  ASSERT_TRUE(runner->Begin());

  buffer_->setTransform(MakeTf("map", "odom", 0, 0, 0, 100.0), "test");
  buffer_->setTransform(MakeTfQ("odom", "base_link", 1.0, 2.0, 0.0,
                                AxisAngle(0, 0, 1, 40), 100.0), "test");
  // A pitched-down optical camera: the body-style mount, then the fixed
  // body -> optical rotation on top of it.
  buffer_->setTransform(
      MakeTfQ("base_link", "oakd_rgb_camera_optical_frame", 0.2, 0.0, 0.3,
              QMul(AxisAngle(0, 1, 0, 20), kBodyToOptical), 100.0), "test");

  PoseSamplerConfig cfg;
  cfg.gaze_frame = "oakd_rgb_camera_optical_frame";
  PoseSampler sampler(buffer_, cfg);
  EXPECT_EQ(sampler.resolved_gaze_frame_convention(), FrameConvention::kOpticalForwardZ)
      << "the name ends in _optical_frame, so auto must resolve to optical";
  ASSERT_TRUE(sampler.Sample(1000.0, *runner, nullptr));
  runner->session()->Flush();
  runner->session()->PumpUploads();

  // Where the lens points: the optical frame's own +Z.
  const V3 lens = Rotate(QMul(AxisAngle(0, 0, 1, 40), QMul(AxisAngle(0, 1, 0, 20), kBodyToOptical)),
                         V3{0, 0, 1});
  EXPECT_LT(WorstGazeRotationErrorDegrees(*transport, V3{0, 0, 1}, RosToUnityAxes(lens)),
            kGazeRotationToleranceDegrees);
  runner->End("test");
}

TEST_F(PoseSamplerTest, AnOpticalGazeFrameFacesWhereTheCameraActuallyLooks) {
  // The absolute direction in the reference frame, from a camera turned off the
  // body. The robot sits at the origin facing +X and the optical camera is yawed
  // 90 degrees, so it looks along map +Y, and the gaze rotation must face +Y.
  buffer_->setTransform(MakeTf("map", "odom", 0, 0, 0, 100.0), "test");
  buffer_->setTransform(MakeTf("odom", "base_link", 0.0, 0.0, 0.0, 100.0), "test");
  buffer_->setTransform(MakeTfQ("base_link", "cam_optical_frame", 0.0, 0.0, 0.0,
                                QMul(AxisAngle(0, 0, 1, 90), kBodyToOptical), 100.0), "test");

  PoseSamplerConfig cfg;
  cfg.gaze_frame = "cam_optical_frame";
  PoseSampler sampler(buffer_, cfg);
  ASSERT_TRUE(sampler.Sample(1000.0, *runner_, nullptr));
  Flush();

  // rep103 runner, so the payload is raw ROS and forward is the rotation's +X.
  EXPECT_LT(WorstGazeRotationErrorDegrees(*transport_, V3{1, 0, 0}, V3{0, 1, 0}),
            kGazeRotationToleranceDegrees)
      << "an optical camera yawed 90 degrees must look along map +Y";
}

TEST_F(PoseSamplerTest, TheDynamicObjectStaysOnTheRobotBodyWhenAGazeFrameIsSet) {
  // The gaze channel moving to the camera must not take the robot with it: the
  // dynamic object is the body, and it is what the viewer draws driving around.
  buffer_->setTransform(MakeTf("map", "odom", 0, 0, 0, 100.0), "test");
  buffer_->setTransform(MakeTf("odom", "base_link", 1.0, 2.0, 0.0, 100.0), "test");
  buffer_->setTransform(MakeTfQ("base_link", "cam_optical_frame", 0.25, 0.0, 0.5,
                                kBodyToOptical, 100.0), "test");

  PoseSamplerConfig cfg;
  cfg.gaze_frame = "cam_optical_frame";
  PoseSampler sampler(buffer_, cfg);
  ASSERT_TRUE(sampler.Sample(1000.0, *runner_, nullptr));
  Flush();

  // rep103, so both are raw ROS. base_link is at (1, 2, 0); the camera is at
  // (1.25, 2, 0.5), which is what the gaze position must be.
  bool gaze_on_camera = false, dynamic_on_body = false;
  for (std::size_t i = 0; i < transport_->urls.size(); ++i) {
    const std::string& url = transport_->urls[i];
    const std::string& body = transport_->bodies[i];
    if (url.find("/gaze/") != std::string::npos &&
        body.find("[1.25,2.0,0.5]") != std::string::npos) {
      gaze_on_camera = true;
    }
    if (url.find("/dynamics/") != std::string::npos &&
        body.find("[1.0,2.0,0.0]") != std::string::npos) {
      dynamic_on_body = true;
    }
  }
  EXPECT_TRUE(gaze_on_camera) << "the gaze pose must be the camera's, or the ray contradicts it";
  EXPECT_TRUE(dynamic_on_body) << "the robot body must still be at base_link";
}

TEST_F(PoseSamplerTest, AMissingGazeFrameFallsBackToTheBodysWholePose) {
  // gaze_frame names a frame that is not in the tree. Dropping the sample
  // would lose the whole session to a typo; the fallback records the body's
  // position and the body's rotation together.
  //
  // And that is exactly why the fallback has to be COUNTED. Every other signal
  // says the run is fine -- the sample is recorded, it is well formed, and the
  // node records `c3d.ros.gaze_frame` regardless -- so the count is the only
  // thing that distinguishes a working camera from a dropped character in its
  // name. This test asserts the count, not just the fallback.
  std::shared_ptr<CapturingTransport> transport;
  auto runner = MakeUnityRunner(&transport);
  ASSERT_TRUE(runner->Begin());

  buffer_->setTransform(MakeTf("map", "odom", 0, 0, 0, 100.0), "test");
  buffer_->setTransform(MakeTfQ("odom", "base_link", 1.0, 2.0, 0.0,
                                AxisAngle(0, 1, 0, 25), 100.0), "test");

  PoseSamplerConfig cfg;
  cfg.gaze_frame = "a_frame_that_does_not_exist_optical_frame";
  PoseSampler sampler(buffer_, cfg);
  ASSERT_TRUE(sampler.Sample(1000.0, *runner, nullptr));
  runner->session()->Flush();
  runner->session()->PumpUploads();

  EXPECT_LT(WorstGazeRotationErrorDegrees(*transport, V3{0, 0, 1},
                                          RosToUnityAxes(Rotate(AxisAngle(0, 1, 0, 25),
                                                                V3{1, 0, 0}))),
            kGazeRotationToleranceDegrees);
  EXPECT_EQ(sampler.gaze_frame_lookup_failures(), 1u)
      << "an unresolvable gaze_frame must be counted, or a typo is invisible";
  EXPECT_EQ(sampler.lookup_failures(), 0u)
      << "the sample was recorded, so it is not a pose lookup failure";

  // It keeps counting, because the operator warning is throttled and the
  // sensor series is what a session is read back by.
  ASSERT_TRUE(sampler.Sample(1001.0, *runner, nullptr));
  EXPECT_EQ(sampler.gaze_frame_lookup_failures(), 2u);

  sampler.Reset();
  EXPECT_EQ(sampler.gaze_frame_lookup_failures(), 0u) << "Reset must clear it with the rest";
  runner->End("test");
}

// ---------------------------------------------------------------------------
// The records as sent
// ---------------------------------------------------------------------------

namespace {

// Every record of a stream's top-level array, read back from the bytes sent.
std::vector<nlohmann::json> RecordsOn(const CapturingTransport& transport,
                                      const std::string& url_fragment,
                                      const std::string& array_name) {
  std::vector<nlohmann::json> out;
  for (std::size_t i = 0; i < transport.urls.size(); ++i) {
    if (transport.urls[i].find(url_fragment) == std::string::npos) continue;
    const auto payload = nlohmann::json::parse(transport.bodies[i]);
    for (const auto& record : payload.value(array_name, nlohmann::json::array())) {
      out.push_back(record);
    }
  }
  return out;
}

}  // namespace

// The records pinned, number for number, under every convention: a non-planar body with no gaze
// frame, and a pitched optical camera as the gaze frame. Session Replay places and turns the robot
// by `p` and `r`, so a change to any of these numbers changes every replay, and the rotation tests
// above cannot see one that keeps the forward axis, such as a roll about it.
TEST_F(PoseSamplerTest, TheGazeRecordsArePinnedUnderEveryConvention) {
  using json = nlohmann::json;
  struct Pinned {
    PoseConvention convention;
    json body_gaze;
    json camera_gaze;
  };
  const std::vector<Pinned> pinned = {
      {PoseConvention::kUnity,
       json::parse(R"({"p":[-2.0,0.5,1.0],"r":[-0.24955,0.45951,0.00307,-0.85239],"time":1000.0})"),
       json::parse(R"({"p":[-2.12856,0.3,1.15321],"r":[0.16318,-0.33682,0.05939,0.92542],)"
                   R"("time":1000.0})")},
      {PoseConvention::kGltfAuthored,
       json::parse(R"({"p":[-1.0,0.5,-2.0],"r":[0.17863,-0.92765,0.17429,0.27781],"time":1000.0})"),
       json::parse(R"({"p":[-1.15321,0.3,-2.12856],"r":[-0.07339,0.89254,-0.15738,-0.4162],)"
                   R"("time":1000.0})")},
      {PoseConvention::kRep103,
       json::parse(R"({"p":[1.0,2.0,0.5],"r":[0.00307,0.24955,0.45951,0.85239],"time":1000.0})"),
       json::parse(R"({"p":[1.15321,2.12856,0.3],"r":[0.05939,-0.16318,-0.33682,-0.92542],)"
                   R"("time":1000.0})")},
  };
  const auto fresh_buffer = [] {
    auto buffer = std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
    buffer->setUsingDedicatedThread(true);
    return buffer;
  };
  for (const Pinned& pin : pinned) {
    const char* name = PoseConventionName(pin.convention);
    {
      std::shared_ptr<CapturingTransport> transport;
      auto runner = MakeRunner(pin.convention, &transport);
      ASSERT_TRUE(runner->Begin());
      auto buffer = fresh_buffer();
      buffer->setTransform(MakeTf("map", "odom", 0, 0, 0, 100.0), "test");
      buffer->setTransform(
          MakeTfQ("odom", "base_link", 1.0, 2.0, 0.5,
                  QMul(QMul(AxisAngle(0, 0, 1, 60), AxisAngle(0, 1, 0, 25)), AxisAngle(1, 0, 0, 15)),
                  100.0),
          "test");
      PoseSampler sampler(buffer, PoseSamplerConfig{});
      ASSERT_TRUE(sampler.Sample(1000.0, *runner, nullptr));
      runner->session()->Flush();
      runner->session()->PumpUploads();
      const auto gaze = RecordsOn(*transport, "/gaze/", "data");
      ASSERT_EQ(gaze.size(), 1u) << name;
      EXPECT_EQ(gaze[0], pin.body_gaze) << name << ": " << gaze[0].dump();
      runner->End("test");
    }
    {
      std::shared_ptr<CapturingTransport> transport;
      auto runner = MakeRunner(pin.convention, &transport);
      ASSERT_TRUE(runner->Begin());
      auto buffer = fresh_buffer();
      buffer->setTransform(MakeTf("map", "odom", 0, 0, 0, 200.0), "test");
      buffer->setTransform(MakeTfQ("odom", "base_link", 1.0, 2.0, 0.0, AxisAngle(0, 0, 1, 40), 200.0),
                           "test");
      buffer->setTransform(MakeTfQ("base_link", "oakd_rgb_camera_optical_frame", 0.2, 0.0, 0.3,
                                   QMul(AxisAngle(0, 1, 0, 20), kBodyToOptical), 200.0),
                           "test");
      PoseSamplerConfig cfg;
      cfg.gaze_frame = "oakd_rgb_camera_optical_frame";
      PoseSampler sampler(buffer, cfg);
      ASSERT_TRUE(sampler.Sample(1000.0, *runner, nullptr));
      runner->session()->Flush();
      runner->session()->PumpUploads();
      const auto gaze = RecordsOn(*transport, "/gaze/", "data");
      ASSERT_EQ(gaze.size(), 1u) << name;
      EXPECT_EQ(gaze[0], pin.camera_gaze) << name << ": " << gaze[0].dump();
      runner->End("test");
    }
  }
}

// ---------------------------------------------------------------------------
// One instant per tick: the body chain and a head chain stamped differently
// ---------------------------------------------------------------------------

namespace {

// Yaw in degrees of a pure-yaw quaternion as SERIALIZED, [x, y, z, w].
double SerializedYawDegrees(const nlohmann::json& r) {
  return 2.0 * std::atan2(r[2].get<double>(), r[3].get<double>()) * 180.0 / M_PI;
}

}  // namespace

// The other side of the same rule: a joint whose broadcaster is AHEAD of the body
// is read at the body's stamp, interpolated, and not at the joint's own newest. The
// head pans 0 -> 40 degrees between 100.0 and 100.2, and the body's newest is 100.1.
TEST_F(PoseSamplerTest, AJointAheadOfTheBodyIsReadAtTheBodysStampNotItsOwnLatest) {
  SetChain(100.0, AxisAngle(0, 0, 1, 0));
  SetChain(100.1, AxisAngle(0, 0, 1, 0));
  buffer_->setTransform(MakeTfQ("base_link", "camera_link", 0.0, 0.0, 0.1,
                                AxisAngle(0, 0, 1, 0), 100.0), "test");
  buffer_->setTransform(MakeTfQ("base_link", "camera_link", 0.0, 0.0, 0.1,
                                AxisAngle(0, 0, 1, 40), 100.2), "test");
  PoseSamplerConfig cfg;
  cfg.gaze_frame = "camera_link";
  PoseSampler sampler(buffer_, cfg);
  ASSERT_TRUE(sampler.Sample(1000.0, *runner_, nullptr));
  EXPECT_EQ(sampler.gaze_frame_lookup_failures(), 0u);
  Flush();

  const auto gaze = RecordsOn(*transport_, "/gaze/", "data");
  ASSERT_EQ(gaze.size(), 1u);
  EXPECT_NEAR(SerializedYawDegrees(gaze[0]["r"]), 20.0, 1e-3)
      << "the joint must be read at the body's stamp, 100.1, halfway through its pan";
}

// The body chain (map -> odom -> base_link) is newest at 100.1 and turning; the
// head joint (base_link -> head_camera_frame) comes from a slower broadcaster whose
// newest stamp is 100.05. A "latest" lookup of the head's whole chain resolves to
// 100.05 -- the oldest newest stamp along it -- and poses the camera where the body
// was half a turn step ago: 10 degrees of yaw here, while the body is at 20. The
// camera must be posed at the BODY's instant instead, so the gaze viewpoint and the
// body it is mounted on are one instant of the robot.
TEST_F(PoseSamplerTest, AGazeFrameOnASlowerChainIsPosedAtTheBodysInstant) {
  SetChain(100.0, AxisAngle(0, 0, 1, 0));
  SetChain(100.1, AxisAngle(0, 0, 1, 20));
  for (double stamp : {100.0, 100.05}) {
    buffer_->setTransform(MakeTfQ("base_link", "head_camera_frame", 0.1, 0.0, 0.1,
                                  AxisAngle(0, 0, 1, 0), stamp), "test");
  }
  PoseSamplerConfig cfg;
  cfg.gaze_frame = "head_camera_frame";
  PoseSampler sampler(buffer_, cfg);
  ASSERT_TRUE(sampler.Sample(1000.0, *runner_, nullptr));
  EXPECT_EQ(sampler.gaze_frame_lookup_failures(), 0u)
      << "a camera behind the body is not a failed lookup, and must not fall back to the body";
  Flush();

  const auto gaze = RecordsOn(*transport_, "/gaze/", "data");
  ASSERT_EQ(gaze.size(), 1u);
  EXPECT_NEAR(SerializedYawDegrees(gaze[0]["r"]), 20.0, 1e-3)
      << "the camera viewpoint trails the body it is mounted on";
  const double yaw = 20.0 * M_PI / 180.0;
  EXPECT_NEAR(gaze[0]["p"][0].get<double>(), 1.0 + 0.1 * std::cos(yaw), 1e-5);
  EXPECT_NEAR(gaze[0]["p"][1].get<double>(), 0.1 * std::sin(yaw), 1e-5);
  EXPECT_NEAR(gaze[0]["p"][2].get<double>(), 0.1, 1e-5);
  EXPECT_LT(WorstGazeRotationErrorDegrees(*transport_, V3{1, 0, 0},
                                          V3{std::cos(yaw), std::sin(yaw), 0.0}),
            kGazeRotationToleranceDegrees);
}

// A transform whose publisher stopped stays in the buffer, and tf keeps answering with it. Once its
// stamp has not advanced for longer than the bound, the sample is skipped and counted instead of
// recording the robot standing still.
TEST_F(PoseSamplerTest, AStaleTransformIsSkippedAndCountedNotRecordedAsFresh) {
  SetChain(100.0, Qxyzw(0, 0, 0, 1));
  PoseSamplerConfig config;
  config.stale_transform_after_s = 2.0;
  PoseSampler sampler(buffer_, config);
  std::string status;
  ASSERT_TRUE(sampler.Sample(1000.0, *runner_, &status)) << status;
  EXPECT_TRUE(sampler.Sample(1001.5, *runner_, &status)) << "within the bound: " << status;
  EXPECT_FALSE(sampler.Sample(1002.5, *runner_, &status));
  EXPECT_TRUE(sampler.last_sample_stale());
  EXPECT_EQ(sampler.stale_transforms(), 1u);
  EXPECT_NE(status.find("has not advanced"), std::string::npos) << status;
  EXPECT_EQ(sampler.lookup_failures(), 0u) << "a stale transform is not a missing one";

  // The publisher resumes: the next sample is fresh again.
  SetChain(101.0, Qxyzw(0, 0, 0, 1));
  EXPECT_TRUE(sampler.Sample(1002.6, *runner_, &status)) << status;
  EXPECT_FALSE(sampler.last_sample_stale());
  EXPECT_EQ(sampler.stale_transforms(), 1u);
}

// A static chain is stamped zero and never advances, which is not staleness.
TEST_F(PoseSamplerTest, AStaticChainIsNeverStale) {
  buffer_->setTransform(MakeTf("map", "base_link", 1.0, 2.0, 0.0, 0.0), "test",
                        /*is_static=*/true);
  PoseSamplerConfig config;
  config.stale_transform_after_s = 2.0;
  PoseSampler sampler(buffer_, config);
  std::string status;
  for (double t : {1000.0, 1005.0, 1010.0, 1060.0}) {
    EXPECT_TRUE(sampler.Sample(t, *runner_, &status)) << t << ": " << status;
  }
  EXPECT_EQ(sampler.stale_transforms(), 0u);
}

// A robot that moved while its transform was stale did not jump: the first fresh pose after the
// stretch is compared with nothing, not with the pose that froze.
TEST_F(PoseSamplerTest, AMoveAcrossAStaleStretchIsNotARelocalisationJump) {
  SetChain(100.0, Qxyzw(0, 0, 0, 1));
  PoseSamplerConfig config;
  config.stale_transform_after_s = 2.0;
  config.relocalisation_jump_m = 0.5;
  PoseSampler sampler(buffer_, config);
  std::string status;
  ASSERT_TRUE(sampler.Sample(1000.0, *runner_, &status)) << status;
  ASSERT_FALSE(sampler.Sample(1002.5, *runner_, &status));
  ASSERT_TRUE(sampler.last_sample_stale());

  // The publisher comes back with the robot four metres on.
  buffer_->setTransform(MakeTf("map", "odom", 0, 0, 0, 101.0), "test");
  buffer_->setTransform(MakeTfQ("odom", "base_link", 5.0, 0.0, 0.0, Qxyzw(0, 0, 0, 1), 101.0),
                        "test");
  ASSERT_TRUE(sampler.Sample(1002.6, *runner_, &status)) << status;
  EXPECT_EQ(sampler.relocalisation_jumps(), 0u)
      << "the move made while the transform was stale was recorded as a localisation jump";
}

// A change of reference frame is a different transform, whose stamp is measured from the change:
// the fallback's stamp having stood still is no evidence against the preferred frame's.
TEST_F(PoseSamplerTest, AChangeOfReferenceFrameRestartsTheStaleClock) {
  buffer_->setTransform(MakeTf("odom", "base_link", 1.0, 0.0, 0.0, 100.0), "test");
  PoseSamplerConfig config;
  config.stale_transform_after_s = 2.0;
  config.reference_frame_grace_s = 0.0;
  PoseSampler sampler(buffer_, config);
  std::string status;
  ASSERT_TRUE(sampler.Sample(1000.0, *runner_, &status)) << status;
  ASSERT_EQ(sampler.active_reference_frame(), "odom");
  ASSERT_TRUE(sampler.Sample(1001.5, *runner_, &status)) << status;

  // `map` arrives, at the same stamp the fallback has been standing at.
  buffer_->setTransform(MakeTf("map", "odom", 0, 0, 0, 100.0), "test");
  EXPECT_TRUE(sampler.Sample(1002.5, *runner_, &status)) << status;
  EXPECT_EQ(sampler.active_reference_frame(), "map");
  EXPECT_FALSE(sampler.last_sample_stale());
  EXPECT_EQ(sampler.stale_transforms(), 0u);
}

TEST(StaleTransformBound, TenSamplePeriodsAndNeverUnderFiveSeconds) {
  EXPECT_DOUBLE_EQ(StaleTransformAfterSeconds(10.0), 5.0);
  EXPECT_DOUBLE_EQ(StaleTransformAfterSeconds(1.0), 10.0);
  EXPECT_DOUBLE_EQ(StaleTransformAfterSeconds(0.5), 20.0);
}

// The first sample a check is due at, for samples 1/hz apart, spans between a tenth of a second,
// less the jitter allowance, and two tenths: every sample at 10 Hz and below, every third at
// 30 Hz. A sample a little early is still the one checked.
TEST(RelocalisationCheck, DueATenthOfASecondApartAndEverySampleBelowTenHertz) {
  for (const double hz : {10.0, 11.0, 15.0, 25.0, 30.0, 1000.0}) {
    int k = 1;
    while (!RelocalisationCheckDue(k / hz, hz) && k < 1000) ++k;
    const double spanned = k / hz;
    EXPECT_GE(spanned, 0.1 - 0.1 * std::min(0.1, 1.0 / hz) - 1e-12) << hz << " Hz";
    EXPECT_LT(spanned, 0.2) << hz << " Hz";
  }
  for (const double hz : {9.0, 2.0, 0.5}) {
    EXPECT_TRUE(RelocalisationCheckDue(1.0 / hz, hz)) << hz << " Hz";
  }
  EXPECT_EQ(RelocalisationCheckDue(1.0 / 30.0, 30.0), false);
  EXPECT_EQ(RelocalisationCheckDue(2.0 / 30.0, 30.0), false);
  EXPECT_EQ(RelocalisationCheckDue(3.0 / 30.0 - 0.002, 30.0), true) << "2 ms early";
}
