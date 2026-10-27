// The pose stream's ROS tier: the tf-only manifest, the conversion every joint takes, and the rig
// sampler's rules for frames that join, fail and leave. The sampler is driven from a tf tree the
// test writes tick by tick (FakeTf), so every rule is reached on purpose rather than by timing;
// the tf2 adapter is checked separately against a real tf2 buffer.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <unistd.h>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "c3d/pose_manifest.hpp"
#include "c3d/spool.hpp"
#include "cognitive3d_ros/pose_rig.hpp"
#include "cognitive3d_ros/tf_rig_sampler.hpp"

using namespace cognitive3d_ros;
namespace fs = std::filesystem;

namespace {

// ---- Small helpers, written out rather than taken from the SDK ------------------------------

c3d::Quat AxisAngle(c3d::Vec3 k, double degrees) {
  const double n = std::sqrt(k.x * k.x + k.y * k.y + k.z * k.z);
  const double half = degrees * M_PI / 360.0, s = std::sin(half) / n;
  return c3d::Quat{k.x * s, k.y * s, k.z * s, std::cos(half)};
}

c3d::Quat Mul(const c3d::Quat& a, const c3d::Quat& b) {
  return c3d::Quat{a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                   a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                   a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                   a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

c3d::Vec3 Rotate(const c3d::Quat& q, const c3d::Vec3& v) {
  const double tx = 2 * (q.y * v.z - q.z * v.y);
  const double ty = 2 * (q.z * v.x - q.x * v.z);
  const double tz = 2 * (q.x * v.y - q.y * v.x);
  return c3d::Vec3{v.x + q.w * tx + q.y * tz - q.z * ty, v.y + q.w * ty + q.z * tx - q.x * tz,
                   v.z + q.w * tz + q.x * ty - q.y * tx};
}

c3d::Vec3 Add(const c3d::Vec3& a, const c3d::Vec3& b) {
  return c3d::Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
}

void ExpectVec(const c3d::Vec3& got, const c3d::Vec3& want, const std::string& what) {
  EXPECT_NEAR(got.x, want.x, 1e-9) << what;
  EXPECT_NEAR(got.y, want.y, 1e-9) << what;
  EXPECT_NEAR(got.z, want.z, 1e-9) << what;
}

// The same rotation: q and -q are one.
void ExpectSameRotation(const c3d::Quat& got, const c3d::Quat& want, const std::string& what) {
  const double sign = got.x * want.x + got.y * want.y + got.z * want.z + got.w * want.w < 0 ? -1 : 1;
  EXPECT_NEAR(sign * got.x, want.x, 1e-9) << what;
  EXPECT_NEAR(sign * got.y, want.y, 1e-9) << what;
  EXPECT_NEAR(sign * got.z, want.z, 1e-9) << what;
  EXPECT_NEAR(sign * got.w, want.w, 1e-9) << what;
}

// Each wire convention's position map M, and the replay viewer's: it negates X on every position
// it reads and conjugates every rotation by that mirror, (x, y, z, w) -> (x, -y, -z, w). Its
// frame is therefore V M, which under both conventions is a PROPER rotation of ROS axes.
struct Convention {
  PoseConvention value;
  const char* name;
  c3d::Vec3 (*wire)(const c3d::Vec3&);    // M
  c3d::Vec3 (*viewer)(const c3d::Vec3&);  // V M
};
const Convention kReflecting[] = {
    {PoseConvention::kUnity, "unity",
     [](const c3d::Vec3& r) { return c3d::Vec3{-r.y, r.z, r.x}; },
     [](const c3d::Vec3& r) { return c3d::Vec3{r.y, r.z, r.x}; }},
    {PoseConvention::kGltfAuthored, "gltf_authored",
     [](const c3d::Vec3& r) { return c3d::Vec3{-r.x, r.z, -r.y}; },
     [](const c3d::Vec3& r) { return c3d::Vec3{r.x, r.z, -r.y}; }},
};

c3d::Quat ViewerMirror(const c3d::Quat& q) { return c3d::Quat{q.x, -q.y, -q.z, q.w}; }

// ---- A tf tree the test writes --------------------------------------------------------------

class FakeTf : public RigTransformSource {
 public:
  struct Edge {
    std::string parent;
    RigTransform transform;
    bool fails = false;
  };
  std::map<std::string, Edge> edges;  // child -> its edge
  // The body as tf resolves it in `reference`, and whether it resolves.
  std::string reference = "map";
  RigTransform body;
  bool body_resolves = true;

  void Set(const std::string& parent, const std::string& child, c3d::Vec3 t, c3d::Quat r,
           std::int64_t stamp_ns) {
    Edge& edge = edges[child];
    edge.parent = parent;
    edge.transform = RigTransform{t, r, stamp_ns};
    edge.fails = false;
  }

  std::vector<RigFrame> Frames() const override {
    std::vector<RigFrame> frames;
    std::set<std::string> named;
    for (const auto& [child, edge] : edges) {
      frames.push_back(RigFrame{child, edge.parent});
      named.insert(child);
    }
    for (const auto& [child, edge] : edges) {
      if (named.insert(edge.parent).second) frames.push_back(RigFrame{edge.parent, ""});
    }
    return frames;
  }
  bool Latest(const std::string& parent, const std::string& child,
              RigTransform* out) const override {
    if (parent == reference && child == "base_link") {
      if (!body_resolves) return false;
      *out = body;
      return true;
    }
    return AtStamp(parent, child, 0, out);
  }
  bool AtStamp(const std::string& parent, const std::string& child, std::int64_t,
               RigTransform* out) const override {
    const auto found = edges.find(child);
    if (found == edges.end() || found->second.parent != parent || found->second.fails) {
      return false;
    }
    *out = found->second.transform;
    return true;
  }
};

class CapturingSink : public PoseRigSink {
 public:
  struct Sample {
    std::string id;
    std::int64_t t_us;
    std::vector<RosJointPose> joints;
    std::optional<bool> enabled;
  };
  struct Event {
    double t;
    std::string name;
    c3d::Properties properties;
  };
  std::vector<c3d::pose::Manifest> manifests;
  std::vector<Sample> samples;
  std::vector<Event> events;
  bool refuse_manifests = false;

  bool SendManifest(const c3d::pose::Manifest& manifest, std::string* error) override {
    if (refuse_manifests) {
      if (error) *error = "refused by the test";
      return false;
    }
    manifests.push_back(manifest);
    return true;
  }
  bool RecordSample(const std::string& id, std::int64_t t_us,
                    const std::vector<RosJointPose>& joints, std::optional<bool> enabled,
                    std::string*) override {
    samples.push_back(Sample{id, t_us, joints, enabled});
    return true;
  }
  void RecordEvent(double t, const std::string& name, const c3d::Vec3&, bool,
                   c3d::Properties properties) override {
    events.push_back(Event{t, name, std::move(properties)});
  }
};

std::vector<std::string> JointNames(const c3d::pose::Manifest& manifest) {
  std::vector<std::string> names;
  for (const auto& joint : manifest.joints) names.push_back(joint.mesh_name);
  return names;
}

std::string Property(const c3d::Properties& properties, const std::string& key) {
  for (const auto& [k, v] : properties) {
    if (k == key) return v.kind() == c3d::PropertyValue::Kind::kString ? v.as_string() : "(non-string)";
  }
  return "(absent)";
}

bool Has(const c3d::Properties& properties, const std::string& key) {
  for (const auto& kv : properties) {
    if (kv.first == key) return true;
  }
  return false;
}

// The Raspbot's tree as tf has it: no robot_description, the body under odom under map, and
// three frames under the body, the camera on a moving gimbal.
std::vector<RigFrame> RaspbotTree() {
  return {{"odom", "map"},          {"base_link", "odom"},
          {"laser", "base_link"},   {"camera_link", "base_link"},
          {"camera_optical_frame", "camera_link"}, {"map", ""}};
}

}  // namespace

// ---- The tf-only manifest -------------------------------------------------------------------

TEST(RigManifest, TheRaspbotIsOneRigOfItsFourFramesUnderTheBody) {
  const auto rig = RigFramesUnder(RaspbotTree(), "base_link", {});
  ASSERT_EQ(rig.size(), 4u);
  RigManifestOptions options;
  options.manifest_id = DefaultPoseManifestId("raspbot-01");
  options.robot_name = "raspbot-01";
  // As the node builds it from `camera.<name>.frame` and `lidar.<name>.frame`.
  options.kinds = RigFrameKinds({{"camera_link", "camera"}, {"laser", "lidar"}});
  const c3d::pose::Manifest m = BuildRigManifest(rig, options);

  EXPECT_EQ(m.manifest_id, "ros2-64a722d4f3a20f48");
  EXPECT_EQ(m.name, "raspbot-01");
  EXPECT_EQ(m.mesh, "");
  EXPECT_EQ(m.file_type, "");
  EXPECT_EQ(m.rigging_mapping_type, "meshName");
  EXPECT_EQ(m.root, "");
  EXPECT_EQ(Property(m.properties, "c3d.profile"), "ros2/1");
  EXPECT_EQ(Property(m.properties, "ros.robotName"), "raspbot-01");
  EXPECT_EQ(m.properties.size(), 2u) << "tier 1 sends these two object keys and no others";

  // Parents before children, siblings by name: camera_link before laser.
  EXPECT_EQ(JointNames(m), (std::vector<std::string>{"base_link", "camera_link",
                                                     "camera_optical_frame", "laser"}));
  EXPECT_FALSE(Has(m.joints[0].properties, "ros.parent"))
      << "the root's pose is in the reference frame, which is not a joint of the rig";
  EXPECT_EQ(Property(m.joints[1].properties, "ros.parent"), "base_link");
  EXPECT_EQ(Property(m.joints[2].properties, "ros.parent"), "camera_link");
  EXPECT_EQ(Property(m.joints[3].properties, "ros.parent"), "base_link");
  EXPECT_EQ(Property(m.joints[1].properties, "c3d.kind"), "camera");
  EXPECT_EQ(Property(m.joints[3].properties, "c3d.kind"), "lidar");
  EXPECT_FALSE(Has(m.joints[2].properties, "c3d.kind"));
  for (const auto& joint : m.joints) {
    EXPECT_EQ(joint.groups, c3d::pose::kPosition | c3d::pose::kRotation)
        << joint.mesh_name << ": tf has no scale";
  }
  EXPECT_EQ(c3d::pose::ManifestProblem(m), "") << "a tf-only manifest is valid by itself";
}

TEST(RigManifest, AnExcludedFrameTakesItsSubtreeAndNothingAboveTheBodyJoins) {
  std::vector<RigFrame> tree = RaspbotTree();
  tree.push_back({"wheel_left_link", "base_link"});
  tree.push_back({"wheel_left_encoder", "wheel_left_link"});
  tree.push_back({"other_robot/base_link", "map"});
  const auto rig = RigFramesUnder(tree, "base_link", {"wheel_left_link", "camera_link"});
  std::vector<std::string> names;
  for (const auto& frame : rig) names.push_back(frame.name);
  EXPECT_EQ(names, (std::vector<std::string>{"base_link", "laser"}));
  EXPECT_TRUE(RigFramesUnder(tree, "chassis", {}).empty()) << "a body tf does not know";
  // A list that is not a tree terminates.
  EXPECT_EQ(RigFramesUnder({{"a", "base_link"}, {"base_link", "a"}}, "base_link", {}).size(), 2u);
}

TEST(RigManifest, AFrameTwoSensorsNameKeepsTheFirstKindInCameraLidarSensorOrder) {
  const auto kinds = RigFrameKinds({{"rgbd_link", "camera"},
                                    {"", "camera"},
                                    {"rgbd_link", "lidar"},
                                    {"sonar_link", "cone_rangefinder"}});
  EXPECT_EQ(kinds, (std::map<std::string, std::string>{{"rgbd_link", "camera"},
                                                       {"sonar_link", "cone_rangefinder"}}));
}

TEST(RigManifest, TheDefaultManifestIdIsAStableHashOfTheDeviceId) {
  // Pinned against an independent FNV-1a 64: the id must not change between releases, or one
  // robot's sessions stop sharing it.
  EXPECT_EQ(DefaultPoseManifestId("raspbot-01"), "ros2-64a722d4f3a20f48");
  EXPECT_EQ(DefaultPoseManifestId("sim-robot-01"), "ros2-a8ee2656b910c9c7");
  EXPECT_EQ(DefaultPoseManifestId(""), "ros2-cbf29ce484222325");
  EXPECT_EQ(DefaultPoseManifestId("raspbot-01"), DefaultPoseManifestId("raspbot-01"));
  EXPECT_NE(DefaultPoseManifestId("raspbot-01"), DefaultPoseManifestId("raspbot-02"));
}

TEST(RigManifest, AModelIsBothMeshAndFileTypeOrNeither) {
  const auto rig = RigFramesUnder(RaspbotTree(), "base_link", {});
  RigManifestOptions options;
  options.manifest_id = "r";
  options.robot_name = "r";
  options.mesh = "raspbot";
  options.file_type = "gltf";
  EXPECT_EQ(c3d::pose::ManifestProblem(BuildRigManifest(rig, options)), "");
  options.file_type = "";
  EXPECT_NE(c3d::pose::ManifestProblem(BuildRigManifest(rig, options)), "");
}

TEST(RigManifest, TheTierTwoSeamDecoratesTheTfOnlyManifest) {
  const auto rig = RigFramesUnder(RaspbotTree(), "base_link", {});
  RigManifestOptions options;
  options.manifest_id = "r";
  options.robot_name = "r";
  const auto m = BuildRigManifest(rig, options, [](c3d::pose::Manifest* manifest) {
    manifest->properties.emplace_back("c3d.model.format", std::string("urdf"));
  });
  EXPECT_EQ(Property(m.properties, "c3d.model.format"), "urdf");
  EXPECT_EQ(JointNames(m).size(), 4u) << "a decoration adds keys; tf decides the joints";
}

// ---- The conversion pins --------------------------------------------------------------------

// A revolute joint turning by +30 degrees about its axis k, in ROS. Under a convention whose
// position map M is a reflection the axis is a pseudovector: the quaternion on the wire turns by
// +30 degrees about -M k (ConvertAngularVelocity's rule), and once the viewer applies its own
// mirror it turns by +30 degrees about V M k, the axis where the viewer draws the link -- the
// same sense of rotation the robot turned with. A joint taking the gaze rule fails under
// gltf_authored, and one taking any unconjugated basis change fails under both.
TEST(RigConversion, ARevoluteJointTurnsTheSameWayInTheViewerUnderEveryReflectingConvention) {
  const c3d::Vec3 axes[] = {{0, 0, 1}, {0, 1, 0}, {1, 0, 0}, {0.3, -0.5, 0.8}};
  for (const Convention& c : kReflecting) {
    for (const c3d::Vec3& k : axes) {
      for (double degrees : {30.0, 135.0}) {
        const std::string what = std::string(c.name) + " about (" + std::to_string(k.x) + ", " +
                                 std::to_string(k.y) + ", " + std::to_string(k.z) + ") by " +
                                 std::to_string(degrees);
        const c3d::Quat ros = AxisAngle(k, degrees);
        const c3d::pose::JointPose sent = RigJointScenePose({c3d::Vec3{}, ros}, c.value);

        // On the wire: the pseudovector -M k.
        const c3d::Vec3 mk = c.wire(k);
        ExpectSameRotation(sent.rotation, AxisAngle(c3d::Vec3{-mk.x, -mk.y, -mk.z}, degrees),
                           what + ", on the wire");
        ExpectVec(ConvertAngularVelocity(k, c.value), c3d::Vec3{-mk.x, -mk.y, -mk.z},
                  what + ": the SDK's own pseudovector rule");

        // In the viewer: +degrees about V M k, the drawn axis.
        ExpectSameRotation(ViewerMirror(sent.rotation), AxisAngle(c.viewer(k), degrees),
                           what + ", in the viewer");
        // And a point on the link lands where the robot's own point is.
        const c3d::Vec3 p{0.2, -0.1, 0.05};
        ExpectVec(Rotate(ViewerMirror(sent.rotation), c.viewer(p)), c.viewer(Rotate(ros, p)),
                  what + ", a point on the link");
      }
    }
  }
}

// Each joint converted on its own and composed down the chain is the converted world pose of
// the link: the property that makes parent-local joints meaningful on the wire at all.
TEST(RigConversion, PerJointConversionComposesToTheConvertedWorldPose) {
  const RosJointPose root{{1.0, 2.0, 0.1}, AxisAngle({0.1, 0.2, 1.0}, 50)};
  const RosJointPose arm{{0.3, 0.0, 0.2}, AxisAngle({0, 1, 0}, -40)};
  const RosJointPose tip{{0.0, 0.1, 0.05}, AxisAngle({1, 0, 0}, 70)};
  // World pose of the tip in ROS axes.
  c3d::Quat world_r = Mul(Mul(root.rotation, arm.rotation), tip.rotation);
  c3d::Vec3 world_p = Add(root.position,
                          Rotate(root.rotation, Add(arm.position, Rotate(arm.rotation,
                                                                         tip.position))));
  for (PoseConvention convention :
       {PoseConvention::kRep103, PoseConvention::kUnity, PoseConvention::kGltfAuthored}) {
    const std::string what = PoseConventionName(convention);
    const auto r = RigJointScenePose(root, convention);
    const auto a = RigJointScenePose(arm, convention);
    const auto t = RigJointScenePose(tip, convention);
    const c3d::Quat composed_r = Mul(Mul(r.rotation, a.rotation), t.rotation);
    const c3d::Vec3 composed_p =
        Add(r.position, Rotate(r.rotation, Add(a.position, Rotate(a.rotation, t.position))));
    const auto world = RigJointScenePose({world_p, world_r}, convention);
    ExpectVec(composed_p, world.position, what + ": position");
    ExpectSameRotation(composed_r, world.rotation, what + ": rotation");
  }
}

TEST(RigConversion, EveryJointTakesTheDynamicRuleNotTheGazeRule) {
  const RosJointPose yawed{{1, 2, 3}, AxisAngle({0, 0, 1}, 90)};
  for (PoseConvention convention :
       {PoseConvention::kRep103, PoseConvention::kUnity, PoseConvention::kGltfAuthored}) {
    const auto sent = RigJointScenePose(yawed, convention);
    ExpectVec(sent.position, ConvertPosition(yawed.position, convention),
              PoseConventionName(convention));
    ExpectSameRotation(sent.rotation, ConvertDynamicRotation(yawed.rotation, convention),
                       PoseConventionName(convention));
  }
  // The difference the rule makes, where there is one.
  const auto gltf = RigJointScenePose(yawed, PoseConvention::kGltfAuthored);
  const c3d::Quat gaze = ConvertGazeRotation(yawed.rotation, PoseConvention::kGltfAuthored);
  EXPECT_GT(std::fabs(std::fabs(gltf.rotation.x * gaze.x + gltf.rotation.y * gaze.y +
                                gltf.rotation.z * gaze.z + gltf.rotation.w * gaze.w) -
                      1.0),
            0.1);
}

// ---- The rig sampler: first seen, hold, lost ------------------------------------------------

class TfRigSamplerTest : public ::testing::Test {
 protected:
  // Epoch seconds, as the wire's clock gives them; tf stamps are a separate clock (stamp_s below).
  static constexpr double kWire0 = 1.8e9;

  void SetUp() override {
    tf_ = std::make_shared<FakeTf>();
    config_.robot_frame = "base_link";
    config_.manifest.manifest_id = "rig";
    config_.manifest.robot_name = "raspbot-01";
    config_.stale_after_s = 5.0;
    config_.frame_lost_after_s = 10.0;
    config_.disable_after_s = 1.0;
  }

  static std::int64_t Ns(double stamp_s) { return std::llround(stamp_s * 1e9); }

  // The Raspbot at ROS time `stamp_s`, the gimbal at `pan` degrees.
  void Raspbot(double stamp_s, double pan = 0.0) {
    tf_->body = RigTransform{{0.5 + stamp_s, 0.0, 0.0}, AxisAngle({0, 0, 1}, 10), Ns(stamp_s)};
    tf_->Set("map", "odom", {}, c3d::Quat{}, Ns(stamp_s));
    tf_->Set("odom", "base_link", {0.5 + stamp_s, 0, 0}, AxisAngle({0, 0, 1}, 10), Ns(stamp_s));
    tf_->Set("base_link", "laser", {0.05, 0, 0.12}, c3d::Quat{}, 0);
    tf_->Set("base_link", "camera_link", {0.08, 0, 0.1}, AxisAngle({0, 0, 1}, pan), Ns(stamp_s));
    tf_->Set("camera_link", "camera_optical_frame", {}, c3d::Quat{-0.5, 0.5, -0.5, 0.5}, 0);
  }

  TfRigSampler::Outcome Tick(TfRigSampler& sampler, double wire) {
    return sampler.Tick(wire, "map", sink_);
  }

  std::shared_ptr<FakeTf> tf_;
  TfRigSamplerConfig config_;
  CapturingSink sink_;
};

TEST_F(TfRigSamplerTest, OneSampleIsTheBodyInTheWorldAndEveryJointInItsParentAtTheWireTime) {
  TfRigSampler sampler(tf_, config_);
  Raspbot(100.0, 25.0);
  ASSERT_EQ(Tick(sampler, kWire0 + 0.123456), TfRigSampler::Outcome::kRecorded);
  ASSERT_EQ(sink_.manifests.size(), 1u);
  ASSERT_EQ(sink_.samples.size(), 1u);
  const auto& sample = sink_.samples[0];
  EXPECT_EQ(sample.id, "rig");
  // The wire's clock, never a tf stamp.
  EXPECT_EQ(sample.t_us, std::llround((kWire0 + 0.123456) * 1e6));
  EXPECT_FALSE(sample.enabled.has_value()) << "an object counts as visible until it is not";
  ASSERT_EQ(sample.joints.size(), 4u);
  ExpectVec(sample.joints[0].position, c3d::Vec3{100.5, 0, 0}, "root in map");
  ExpectSameRotation(sample.joints[1].rotation, AxisAngle({0, 0, 1}, 25), "camera_link in base");
  ExpectVec(sample.joints[3].position, c3d::Vec3{0.05, 0, 0.12}, "laser in base_link");
  EXPECT_EQ(sampler.samples(), 1u);
  EXPECT_EQ(sampler.held_frames(), 0u);
}

TEST_F(TfRigSamplerTest, NothingIsRecordedBeforeTheGazeChannelHasAReferenceFrame) {
  TfRigSampler sampler(tf_, config_);
  Raspbot(100.0);
  EXPECT_EQ(sampler.Tick(kWire0, "", sink_), TfRigSampler::Outcome::kNoReference);
  tf_->body_resolves = false;
  EXPECT_EQ(Tick(sampler, kWire0 + 0.1), TfRigSampler::Outcome::kNoBody);
  EXPECT_TRUE(sink_.manifests.empty());
  EXPECT_TRUE(sink_.samples.empty());
}

TEST_F(TfRigSamplerTest, AFrameJoinsAfterItsFirstSuccessfulLookupWithAManifestReSend) {
  TfRigSampler sampler(tf_, config_);
  Raspbot(100.0);
  tf_->edges["camera_link"].fails = true;
  ASSERT_EQ(Tick(sampler, kWire0), TfRigSampler::Outcome::kRecorded);
  ASSERT_EQ(sink_.manifests.size(), 1u);
  EXPECT_EQ(JointNames(sink_.manifests[0]), (std::vector<std::string>{"base_link", "laser"}))
      << "camera_link has no value yet, and camera_optical_frame's parent is not in the rig";
  EXPECT_EQ(sink_.samples[0].joints.size(), 2u);
  EXPECT_EQ(sampler.held_frames(), 0u) << "a frame never seen is not held";
  EXPECT_TRUE(sink_.events.empty()) << "a frame never seen was never lost";

  Raspbot(100.1);
  ASSERT_EQ(Tick(sampler, kWire0 + 0.1), TfRigSampler::Outcome::kRecorded);
  ASSERT_EQ(sink_.manifests.size(), 2u);
  EXPECT_EQ(JointNames(sink_.manifests[1]),
            (std::vector<std::string>{"base_link", "camera_link", "camera_optical_frame",
                                      "laser"}));
  EXPECT_EQ(sink_.samples[1].joints.size(), 4u);

  Raspbot(100.2);
  Tick(sampler, kWire0 + 0.2);
  EXPECT_EQ(sink_.manifests.size(), 2u) << "an unchanged rig is not sent again";
  EXPECT_EQ(sampler.manifest_sends(), 2u);
}

TEST_F(TfRigSamplerTest, AFailedLookupHoldsTheLastValueAndCountsItOncePerJointAndSample) {
  TfRigSampler sampler(tf_, config_);
  Raspbot(100.0, 15.0);
  Tick(sampler, kWire0);
  Raspbot(100.1, 99.0);
  tf_->edges["camera_link"].fails = true;
  ASSERT_EQ(Tick(sampler, kWire0 + 0.1), TfRigSampler::Outcome::kRecorded);
  Raspbot(100.2, 99.0);
  tf_->edges["camera_link"].fails = true;
  Tick(sampler, kWire0 + 0.2);

  EXPECT_EQ(sink_.manifests.size(), 1u) << "a hold changes nothing the manifest says";
  ASSERT_EQ(sink_.samples.size(), 3u);
  for (std::size_t i = 1; i < 3; ++i) {
    ASSERT_EQ(sink_.samples[i].joints.size(), 4u);
    ExpectSameRotation(sink_.samples[i].joints[1].rotation, AxisAngle({0, 0, 1}, 15),
                       "camera_link held at its last value");
  }
  EXPECT_EQ(sampler.held_frames(), 2u);
  const auto warnings = sampler.TakeWarnings();
  ASSERT_EQ(warnings.size(), 1u) << "one warning per frame, not per sample";
  EXPECT_NE(warnings[0].find("camera_link"), std::string::npos);
}

TEST_F(TfRigSamplerTest, AJointWhoseBroadcasterStoppedIsHeldThoughTfStillAnswersIt) {
  TfRigSampler sampler(tf_, config_);
  Raspbot(100.0, 15.0);
  Tick(sampler, kWire0);
  // The gimbal's broadcaster stops at ROS time 100; tf answers its last transform for ever.
  for (int i = 1; i <= 70; ++i) {
    Raspbot(100.0 + i * 0.1, 15.0);
    tf_->edges["camera_link"].transform.stamp_ns = Ns(100.0);
    Tick(sampler, kWire0 + i * 0.1);
  }
  // Fresh while within stale_after_s of the body's stamp, held after.
  EXPECT_EQ(sampler.held_frames(), 20u) << "ticks 51..70 are more than 5 s behind the body";
  EXPECT_EQ(sink_.manifests.size(), 1u) << "not yet gone for the 10 s cache window";
}

TEST_F(TfRigSamplerTest, OnAStaticBodyAJointWhoseStampStopsAdvancingIsHeld) {
  TfRigSampler sampler(tf_, config_);
  Raspbot(100.0);
  // A body on a static chain has no stamp to compare a joint's with: the joint's own does.
  tf_->body.stamp_ns = 0;
  // Whole seconds, so no tick sits on a bound by rounding.
  for (int i = 0; i <= 7; ++i) Tick(sampler, kWire0 + i);
  EXPECT_EQ(sampler.held_frames(), 2u) << "camera_link's stamp has not moved for more than 5 s "
                                          "at 6 s and 7 s";
}

TEST_F(TfRigSamplerTest, AFrameGoneLongerThanTheCacheWindowLeavesWithItsSubtreeAndAnEvent) {
  TfRigSampler sampler(tf_, config_);
  Raspbot(100.0);
  Tick(sampler, kWire0);
  const double last_seen = kWire0;
  int ticks = 0;
  for (int i = 1; i <= 102; ++i) {
    Raspbot(100.0 + i * 0.1);
    tf_->edges["camera_link"].fails = true;
    Tick(sampler, kWire0 + i * 0.1);
    ++ticks;
    if (sink_.manifests.size() > 1) break;
  }
  ASSERT_EQ(sink_.manifests.size(), 2u);
  EXPECT_EQ(ticks, 101) << "dropped on the first tick more than 10 s after it was last seen";
  EXPECT_EQ(JointNames(sink_.manifests[1]), (std::vector<std::string>{"base_link", "laser"}));
  ASSERT_EQ(sink_.events.size(), 1u) << "one event for the lost frame, none for its subtree";
  EXPECT_EQ(sink_.events[0].name, "robot.frame_lost");
  EXPECT_EQ(Property(sink_.events[0].properties, "frame"), "camera_link");
  EXPECT_EQ(Property(sink_.events[0].properties, "parent"), "base_link");
  for (const auto& [key, value] : sink_.events[0].properties) {
    if (key == "last_seen_epoch_seconds") EXPECT_DOUBLE_EQ(value.as_double(), last_seen);
  }
  EXPECT_EQ(sampler.frames_lost(), 1u);
  EXPECT_EQ(sampler.held_frames(), 100u) << "held from its first failure until it left";
  EXPECT_EQ(sink_.samples.back().joints.size(), 2u);

  // It comes back: first seen again, joined again.
  Raspbot(111.0);
  Tick(sampler, kWire0 + 11.0);
  ASSERT_EQ(sink_.manifests.size(), 3u);
  EXPECT_EQ(JointNames(sink_.manifests[2]).size(), 4u);
}

TEST_F(TfRigSamplerTest, AStoppedFrameThatLeftDoesNotRejoinOnItsOldTransform) {
  TfRigSampler sampler(tf_, config_);
  Raspbot(100.0);
  Tick(sampler, kWire0);
  for (int i = 1; i <= 200; ++i) {
    Raspbot(100.0 + i * 0.1);
    tf_->edges["camera_link"].transform.stamp_ns = Ns(100.0);
    Tick(sampler, kWire0 + i * 0.1);
  }
  EXPECT_EQ(sampler.frames_lost(), 1u);
  EXPECT_EQ(sink_.manifests.size(), 2u) << "lost once, and not flapping back in";
}

TEST_F(TfRigSamplerTest, AFrameWhoseParentChangesIsSentAgainUnderTheNewOne) {
  TfRigSampler sampler(tf_, config_);
  Raspbot(100.0);
  Tick(sampler, kWire0);
  // Re-parented without moving in the joint order: only `ros.parent` says it changed.
  Raspbot(100.1);
  tf_->Set("base_link", "camera_optical_frame", {0.08, 0, 0.1}, c3d::Quat{}, Ns(100.1));
  Tick(sampler, kWire0 + 0.1);
  ASSERT_EQ(sink_.manifests.size(), 2u);
  const auto& m = sink_.manifests[1];
  ASSERT_EQ(JointNames(m), JointNames(sink_.manifests[0]));
  EXPECT_EQ(Property(m.joints[2].properties, "ros.parent"), "base_link");
}

TEST_F(TfRigSamplerTest, AFrameThatStoppedBeforeTheSessionNeverJoins) {
  TfRigSampler sampler(tf_, config_);
  // tf still holds the gimbal's last transform, from 20 s before the body's.
  Raspbot(100.0);
  tf_->edges["camera_link"].transform.stamp_ns = Ns(80.0);
  Tick(sampler, kWire0);
  ASSERT_EQ(sink_.manifests.size(), 1u);
  EXPECT_EQ(JointNames(sink_.manifests[0]), (std::vector<std::string>{"base_link", "laser"}));
}

TEST_F(TfRigSamplerTest, ExcludedFramesNeverJoin) {
  config_.excluded = {"camera_link"};
  TfRigSampler sampler(tf_, config_);
  Raspbot(100.0);
  Tick(sampler, kWire0);
  EXPECT_EQ(JointNames(sink_.manifests[0]), (std::vector<std::string>{"base_link", "laser"}));
}

TEST_F(TfRigSamplerTest, ABodyMissingPastOneSecondIsOneDisabledSampleAndItsReturnEnablesIt) {
  TfRigSampler sampler(tf_, config_);
  Raspbot(100.0, 20.0);
  Tick(sampler, kWire0);
  tf_->body_resolves = false;
  for (int i = 1; i <= 15; ++i) Tick(sampler, kWire0 + i * 0.1);
  ASSERT_EQ(sink_.samples.size(), 2u) << "one disabled sample, at the first tick a second on";
  const auto& off = sink_.samples[1];
  EXPECT_EQ(off.enabled, std::optional<bool>(false));
  EXPECT_EQ(off.t_us, std::llround((kWire0 + 1.0) * 1e6));
  ExpectVec(off.joints[0].position, sink_.samples[0].joints[0].position, "held root");
  ExpectSameRotation(off.joints[1].rotation, AxisAngle({0, 0, 1}, 20), "held joint");

  tf_->body_resolves = true;
  Raspbot(102.0, 20.0);
  Tick(sampler, kWire0 + 2.0);
  ASSERT_EQ(sink_.samples.size(), 3u);
  EXPECT_EQ(sink_.samples[2].enabled, std::optional<bool>(true));
  Raspbot(102.1, 20.0);
  Tick(sampler, kWire0 + 2.1);
  EXPECT_FALSE(sink_.samples[3].enabled.has_value()) << "said where it changes, not after";
}

TEST_F(TfRigSamplerTest, AStaleBodyRecordsNothingAndAStaticBodyIsNeverStale) {
  TfRigSampler sampler(tf_, config_);
  Raspbot(100.0);
  // Whole seconds, so no tick sits on a bound by rounding.
  for (int i = 0; i <= 12; ++i) Tick(sampler, kWire0 + i);
  ASSERT_EQ(sink_.samples.size(), 7u)
      << "ticks 0 to 5 s are within stale_after_s; 6 s is stale and a second without a body, so "
         "it is the one disabled sample; nothing after";
  EXPECT_EQ(sink_.samples.back().enabled, std::optional<bool>(false));

  TfRigSampler fixed(tf_, config_);
  CapturingSink sink;
  tf_->body.stamp_ns = 0;
  for (int i = 0; i <= 12; ++i) fixed.Tick(kWire0 + i, "map", sink);
  EXPECT_EQ(sink.samples.size(), 13u);
}

TEST_F(TfRigSamplerTest, ARefusedManifestRecordsNoSampleAndIsTriedAgain) {
  TfRigSampler sampler(tf_, config_);
  Raspbot(100.0);
  sink_.refuse_manifests = true;
  EXPECT_EQ(Tick(sampler, kWire0), TfRigSampler::Outcome::kRefused);
  EXPECT_TRUE(sink_.samples.empty());
  EXPECT_EQ(sampler.TakeWarnings().size(), 1u);
  sink_.refuse_manifests = false;
  Raspbot(100.1);
  EXPECT_EQ(Tick(sampler, kWire0 + 0.1), TfRigSampler::Outcome::kRecorded);
  EXPECT_EQ(sink_.manifests.size(), 1u);
}

TEST_F(TfRigSamplerTest, ResetSendsTheManifestAgainForTheNextSession) {
  TfRigSampler sampler(tf_, config_);
  Raspbot(100.0);
  Tick(sampler, kWire0);
  sampler.Reset();
  Raspbot(100.1);
  Tick(sampler, kWire0 + 0.1);
  EXPECT_EQ(sink_.manifests.size(), 2u);
  EXPECT_EQ(sampler.manifest_sends(), 1u);
  EXPECT_EQ(sampler.samples(), 1u);
}

// ---- The tf2 adapter ------------------------------------------------------------------------

namespace {

geometry_msgs::msg::TransformStamped Tf(const std::string& parent, const std::string& child,
                                        double x, double stamp_s) {
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = parent;
  t.child_frame_id = child;
  t.header.stamp.sec = static_cast<std::int32_t>(stamp_s);
  t.header.stamp.nanosec =
      static_cast<std::uint32_t>(std::llround((stamp_s - std::floor(stamp_s)) * 1e9));
  t.transform.translation.x = x;
  t.transform.rotation.w = 1.0;
  return t;
}

}  // namespace

TEST(Tf2RigTransforms, ReadsTheFramesTheirParentsAndAJointAtTheBodysStamp) {
  auto buffer = std::make_shared<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  buffer->setUsingDedicatedThread(true);
  buffer->setTransform(Tf("map", "odom", 0, 10.0), "test");
  buffer->setTransform(Tf("map", "odom", 0, 11.0), "test");
  buffer->setTransform(Tf("odom", "base_link", 1, 10.0), "test");
  buffer->setTransform(Tf("odom", "base_link", 2, 11.0), "test");
  buffer->setTransform(Tf("base_link", "laser", 0.1, 0.0), "test", /*is_static=*/true);
  buffer->setTransform(Tf("base_link", "camera_link", 0.5, 10.0), "test");
  buffer->setTransform(Tf("base_link", "camera_link", 0.7, 10.6), "test");
  const auto source = std::make_shared<Tf2RigTransforms>(buffer);

  std::map<std::string, std::string> parents;
  for (const auto& frame : source->Frames()) parents[frame.name] = frame.parent;
  EXPECT_EQ(parents["odom"], "map");
  EXPECT_EQ(parents["base_link"], "odom");
  EXPECT_EQ(parents["laser"], "base_link");
  EXPECT_EQ(parents["camera_link"], "base_link");
  EXPECT_EQ(parents["map"], "");
  EXPECT_EQ(RigFramesUnder(source->Frames(), "base_link", {}).size(), 3u);

  RigTransform t;
  ASSERT_TRUE(source->Latest("map", "base_link", &t));
  EXPECT_DOUBLE_EQ(t.translation.x, 2.0);
  // At a stamp inside the joint's history: interpolated there.
  ASSERT_TRUE(source->AtStamp("base_link", "camera_link", 10'300'000'000, &t));
  EXPECT_NEAR(t.translation.x, 0.6, 1e-9);
  EXPECT_EQ(t.stamp_ns, 10'300'000'000);
  // Past the joint's newest: its own latest, stamped as such.
  ASSERT_TRUE(source->AtStamp("base_link", "camera_link", 11'000'000'000, &t));
  EXPECT_NEAR(t.translation.x, 0.7, 1e-9);
  EXPECT_EQ(t.stamp_ns, 10'600'000'000);
  ASSERT_TRUE(source->AtStamp("base_link", "laser", 11'000'000'000, &t));
  EXPECT_DOUBLE_EQ(t.translation.x, 0.1);
  EXPECT_FALSE(source->AtStamp("base_link", "nowhere", 11'000'000'000, &t));
}

// ---- The session sink -----------------------------------------------------------------------

namespace {

class NoNetwork : public c3d::ITransport {
 public:
  c3d::HttpResponse Post(const std::string&, const std::string&,
                         const std::vector<std::string>&) override {
    c3d::HttpResponse response;
    response.status = 503;
    return response;
  }
};

}  // namespace

TEST(SessionRigSink, ConvertsEveryJointAndCountsTheFramesInTheSession) {
  const fs::path dir = fs::temp_directory_path() / ("c3d_rig_" + std::to_string(::getpid()));
  fs::create_directories(dir);
  c3d::Config cfg;
  cfg.application_key = "K";
  cfg.scene_id = "S";
  cfg.scene_version = "1";
  cfg.device_id = "robot";
  cfg.spool_dir = dir.string();
  cfg.auto_send_interval_s = 3600.0;
  // Nothing is sent: the samples reach the session and its counters, which is what is asked.
  cfg.uploads_paused = true;
  auto spool = std::make_shared<c3d::DiskSpool>(dir.string(), 64ull << 20,
                                                std::make_shared<c3d::NullLogger>());
  SessionRunner runner(cfg, PoseConvention::kGltfAuthored, std::make_shared<NoNetwork>(), spool,
                       std::make_shared<c3d::NullLogger>());
  ASSERT_TRUE(runner.Begin());
  SessionRigSink sink(runner, nullptr);
  RigManifestOptions options;
  options.manifest_id = "rig";
  options.robot_name = "robot";
  std::string error;
  ASSERT_TRUE(sink.SendManifest(
      BuildRigManifest(RigFramesUnder(RaspbotTree(), "base_link", {}), options), &error))
      << error;
  const double now = runner.session()->session_timestamp() + 1.0;
  const std::vector<RosJointPose> joints(4, RosJointPose{{1, 2, 3}, c3d::Quat{}});
  EXPECT_TRUE(sink.RecordSample("rig", std::llround(now * 1e6), joints, std::nullopt, &error))
      << error;
  EXPECT_FALSE(sink.RecordSample("rig", std::llround(now * 1e6), {joints[0]}, std::nullopt,
                                 &error))
      << "a sample of another joint count than the manifest's is refused";
  runner.session()->PumpPose();
  const c3d::Stats stats = runner.session()->stats();
  EXPECT_EQ(stats.pose_manifest_sends, 1u);
  EXPECT_EQ(stats.pose_samples, 1u);
  runner.End("test");
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
