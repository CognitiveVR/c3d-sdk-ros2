// The pose stream end to end on a real ROS graph: a robot publishes a small moving tf tree, a tf2
// listener fills a buffer from it, the gaze channel's sampler settles the reference frame, and the
// rig sampler sends the tree into a real session. What the session was given is captured on the
// way in, and the session's own counters and its upload to /v0/pose say it took it.
//
// The wire itself is the capture tier's: the Cap'n Proto readers are private to core/.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "c3d/spool.hpp"
#include "cognitive3d_ros/pose_sampler.hpp"
#include "cognitive3d_ros/tf_rig_sampler.hpp"

using namespace cognitive3d_ros;
namespace fs = std::filesystem;
using namespace std::chrono_literals;

namespace {

double WallSeconds() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch())
      .count();
}

class CapturingTransport : public c3d::ITransport {
 public:
  c3d::HttpResponse Post(const std::string& url, const std::string&,
                         const std::vector<std::string>&) override {
    std::lock_guard<std::mutex> lock(mu);
    urls.push_back(url);
    c3d::HttpResponse ok;
    ok.status = 200;
    ok.saw_request_time_header = true;
    return ok;
  }
  std::mutex mu;
  std::vector<std::string> urls;
};

// Records what the rig sampler hands the session, then hands it on.
class Tap : public PoseRigSink {
 public:
  explicit Tap(PoseRigSink& session) : session_(session) {}
  bool SendManifest(const c3d::pose::Manifest& manifest, std::string* error) override {
    const bool ok = session_.SendManifest(manifest, error);
    if (ok) manifests.push_back(manifest);
    return ok;
  }
  bool RecordSample(const std::string& id, std::int64_t t_us,
                    const std::vector<RosJointPose>& joints, std::optional<bool> enabled,
                    std::string* error) override {
    const bool ok = session_.RecordSample(id, t_us, joints, enabled, error);
    if (ok) samples.push_back({t_us, joints});
    return ok;
  }
  void RecordEvent(double t, const std::string& name, const c3d::Vec3& p, bool known,
                   c3d::Properties properties) override {
    session_.RecordEvent(t, name, p, known, std::move(properties));
  }
  struct Sample {
    std::int64_t t_us;
    std::vector<RosJointPose> joints;
  };
  std::vector<c3d::pose::Manifest> manifests;
  std::vector<Sample> samples;

 private:
  PoseRigSink& session_;
};

geometry_msgs::msg::TransformStamped Tf(const std::string& parent, const std::string& child,
                                        const rclcpp::Time& stamp, double x, double yaw) {
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = stamp;
  t.header.frame_id = parent;
  t.child_frame_id = child;
  t.transform.translation.x = x;
  t.transform.rotation.z = std::sin(yaw / 2);
  t.transform.rotation.w = std::cos(yaw / 2);
  return t;
}

// The Raspbot's shape: map -> odom -> base_link driving forward, a lidar and a camera under the
// body, the camera on a gimbal panning at 20 Hz, its optical frame fixed under it.
class Robot : public rclcpp::Node {
 public:
  Robot() : rclcpp::Node("pose_stream_robot") {
    dynamic_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    const rclcpp::Time now = get_clock()->now();
    auto laser = Tf("base_link", "laser", now, 0.05, 0.0);
    auto optical = Tf("camera_link", "camera_optical_frame", now, 0.0, 0.0);
    optical.transform.rotation.x = -0.5;
    optical.transform.rotation.y = 0.5;
    optical.transform.rotation.z = -0.5;
    optical.transform.rotation.w = 0.5;
    static_->sendTransform({Tf("map", "odom", now, 0.0, 0.0), laser, optical});
    start_ = now;
    body_timer_ = create_wall_timer(20ms, [this]() {
      const rclcpp::Time now = get_clock()->now();
      dynamic_->sendTransform(Tf("odom", "base_link", now, 0.5 * (now - start_).seconds(), 0.0));
    });
    gimbal_timer_ = create_wall_timer(50ms, [this]() {
      const rclcpp::Time now = get_clock()->now();
      dynamic_->sendTransform(
          Tf("base_link", "camera_link", now, 0.08, 0.6 * std::sin((now - start_).seconds())));
    });
  }

 private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_;
  rclcpp::Time start_;
  rclcpp::TimerBase::SharedPtr body_timer_, gimbal_timer_;
};

std::string Property(const c3d::Properties& properties, const std::string& key) {
  for (const auto& [k, v] : properties) {
    if (k == key) return v.as_string();
  }
  return "(absent)";
}

}  // namespace

TEST(PoseStream, AMovingTfTreeReachesTheSessionAsOneRigWithItsParentsAndWallClockTimes) {
  const fs::path dir = fs::temp_directory_path() / ("c3d_pose_stream_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  c3d::Config cfg;
  cfg.application_key = "K";
  cfg.scene_id = "S";
  cfg.scene_version = "1";
  cfg.device_id = "raspbot-01";
  cfg.spool_dir = dir.string();
  cfg.auto_send_interval_s = 3600.0;
  cfg.worker_tick_s = 0.01;
  cfg.pose_block_interval_s = 1.0;
  auto transport = std::make_shared<CapturingTransport>();
  auto spool = std::make_shared<c3d::DiskSpool>(dir.string(), 64ull << 20,
                                                std::make_shared<c3d::NullLogger>());
  SessionRunner runner(cfg, PoseConvention::kUnity, transport, spool,
                       std::make_shared<c3d::NullLogger>());
  ASSERT_TRUE(runner.Begin());

  auto robot = std::make_shared<Robot>();
  auto listener_node = std::make_shared<rclcpp::Node>("pose_stream_listener");
  auto buffer = std::make_shared<tf2_ros::Buffer>(listener_node->get_clock());
  auto listener = std::make_shared<tf2_ros::TransformListener>(*buffer, listener_node, true);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(robot);
  std::atomic<bool> stop{false};
  std::thread spin([&]() {
    while (!stop) executor.spin_some(10ms);
  });

  PoseSamplerConfig gaze;
  gaze.reference_frame_grace_s = 0.0;
  PoseSampler pose_sampler(buffer, gaze);
  TfRigSamplerConfig rig;
  rig.manifest.manifest_id = DefaultPoseManifestId(cfg.device_id);
  rig.manifest.robot_name = cfg.device_id;
  rig.manifest.kinds = RigFrameKinds({{"camera_link", "camera"}, {"laser", "lidar"}});
  TfRigSampler sampler(std::make_shared<Tf2RigTransforms>(buffer), rig);
  SessionRigSink session_sink(runner, nullptr);
  Tap tap(session_sink);

  const double started = WallSeconds();
  std::string status;
  while (WallSeconds() - started < 4.0) {
    const double wire = WallSeconds();
    pose_sampler.Sample(wire, runner, &status);
    sampler.Tick(wire, pose_sampler.reference_frame_in_force(), tap);
    std::this_thread::sleep_for(100ms);
  }
  const double finished = WallSeconds();
  stop = true;
  spin.join();

  ASSERT_FALSE(tap.manifests.empty());
  const c3d::pose::Manifest& m = tap.manifests.back();
  std::vector<std::string> names;
  for (const auto& joint : m.joints) names.push_back(joint.mesh_name);
  ASSERT_EQ(names, (std::vector<std::string>{"base_link", "camera_link", "camera_optical_frame",
                                             "laser"}));
  EXPECT_EQ(Property(m.joints[1].properties, "ros.parent"), "base_link");
  EXPECT_EQ(Property(m.joints[2].properties, "ros.parent"), "camera_link");
  EXPECT_EQ(Property(m.joints[3].properties, "ros.parent"), "base_link");
  EXPECT_EQ(Property(m.joints[1].properties, "c3d.kind"), "camera");
  EXPECT_EQ(Property(m.joints[3].properties, "c3d.kind"), "lidar");
  EXPECT_EQ(Property(m.properties, "c3d.profile"), "ros2/1");
  EXPECT_EQ(m.manifest_id, DefaultPoseManifestId("raspbot-01"));

  // About ten a second for four seconds, less the start while tf fills.
  ASSERT_GE(tap.samples.size(), 25u);
  const auto& last = tap.samples.back();
  for (std::size_t i = 1; i < tap.samples.size(); ++i) {
    EXPECT_GT(tap.samples[i].t_us, tap.samples[i - 1].t_us) << "sample " << i;
  }
  // Wall-clock epoch microseconds, inside the run: never a tf stamp.
  EXPECT_GE(tap.samples.front().t_us, std::llround(started * 1e6));
  EXPECT_LE(last.t_us, std::llround(finished * 1e6));
  ASSERT_EQ(last.joints.size(), 4u);
  // The body drives forward in map; the laser sits still on it; the gimbal pans.
  EXPECT_GT(last.joints[0].position.x, tap.samples.front().joints.size() == 4
                                           ? tap.samples.front().joints[0].position.x
                                           : -1.0);
  EXPECT_NEAR(last.joints[3].position.x, 0.05, 1e-9);
  double low = 1.0, high = -1.0;
  for (const auto& sample : tap.samples) {
    if (sample.joints.size() != 4) continue;
    low = std::min(low, sample.joints[1].rotation.z);
    high = std::max(high, sample.joints[1].rotation.z);
  }
  EXPECT_GT(high - low, 0.2) << "camera_link's pan reaches the stream";
  EXPECT_EQ(sampler.held_frames(), 0u);

  runner.session()->PumpPose();
  const c3d::Stats stats = runner.session()->stats();
  EXPECT_EQ(stats.pose_manifest_sends, tap.manifests.size());
  EXPECT_EQ(stats.pose_samples, tap.samples.size());
  EXPECT_EQ(stats.pose_samples_refused, 0u);
  runner.End("test");
  runner.session()->PumpUploads();
  bool posed = false;
  {
    std::lock_guard<std::mutex> lock(transport->mu);
    for (const auto& url : transport->urls) posed = posed || url.find("/v0/pose") != std::string::npos;
  }
  EXPECT_TRUE(posed) << "the session uploaded pose parts";
  std::error_code ec;
  fs::remove_all(dir, ec);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
