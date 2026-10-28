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
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
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
#include "cognitive3d_ros/urdf_description.hpp"

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

TEST(RigManifest, ADecorationAddsKeysToTheTfOnlyManifest) {
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

// ---- Tier 2: the robot description's keys ---------------------------------------------------

namespace {

std::string UrdfFixture(const std::string& name) {
  const char* dir = std::getenv("C3D_URDF_FIXTURES");
  EXPECT_NE(dir, nullptr) << "C3D_URDF_FIXTURES is set by the test's CMake entry";
  std::ifstream in(std::string(dir == nullptr ? "." : dir) + "/" + name, std::ios::binary);
  EXPECT_TRUE(in.good()) << name;
  return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

std::shared_ptr<const UrdfDescription> Description(const std::string& xml) {
  auto description = std::make_shared<UrdfDescription>();
  std::string error;
  EXPECT_TRUE(ParseUrdfDescription(xml, description.get(), &error)) << error;
  return description;
}

double Number(const c3d::Properties& properties, const std::string& key) {
  for (const auto& [k, v] : properties) {
    if (k == key) return v.kind() == c3d::PropertyValue::Kind::kDouble ? v.as_double() : -999.0;
  }
  return std::nan("");
}

std::vector<std::string> Keys(const c3d::Properties& properties) {
  std::vector<std::string> keys;
  for (const auto& kv : properties) keys.push_back(kv.first);
  return keys;
}

c3d::pose::Manifest Decorated(const std::vector<RigFrame>& tree,
                              std::shared_ptr<const UrdfDescription> description) {
  RigManifestOptions options;
  options.manifest_id = "rig";
  options.robot_name = "raspbot-01";
  options.kinds = RigFrameKinds({{"camera_link", "camera"}, {"laser", "lidar"}});
  return BuildRigManifest(RigFramesUnder(tree, "base_link", {}), options,
                          UrdfDecoration(std::move(description)));
}

}  // namespace

// Yahboom's stock description names the wheels and the gimbal's two links; tf on the Raspbot
// publishes none of them, and the description names none of tf's sensor frames. Only base_link
// is shared, and it is the root, so the joints are tf's alone and keep their tier 1 keys.
TEST(RigManifestTier2, YahboomsDescriptionOnTheRaspbotsTfAddsObjectKeysAndNoJoint) {
  const auto yahboom = Description(UrdfFixture("raspbot_v2_yahboom.urdf"));
  const c3d::pose::Manifest tier1 = Decorated(RaspbotTree(), nullptr);
  const c3d::pose::Manifest m = Decorated(RaspbotTree(), yahboom);

  EXPECT_EQ(m.name, "raspbot-01") << "the manifest's name stays device_id";
  EXPECT_EQ(Property(m.properties, "ros.robotName"), "Raspbot-V2");
  EXPECT_EQ(Property(m.properties, "ros.urdfSha256"),
            "e18f29dc721ddf4b8fa485e08ffef60fca90e634572ce612210901fbae3d3ce6");
  EXPECT_EQ(Property(m.properties, "c3d.model.format"), "urdf");
  EXPECT_EQ(Property(m.properties, "c3d.model.formatVersion"), "1.0");
  EXPECT_EQ(Keys(m.properties), (std::vector<std::string>{"c3d.profile", "ros.robotName",
                                                          "ros.urdfSha256", "c3d.model.format",
                                                          "c3d.model.formatVersion"}));
  EXPECT_EQ(JointNames(m), JointNames(tier1)) << "tf decides the joints";
  for (std::size_t j = 0; j < m.joints.size(); ++j) {
    EXPECT_EQ(Keys(m.joints[j].properties), Keys(tier1.joints[j].properties))
        << m.joints[j].mesh_name << " is not a URDF joint's child, or is the root";
  }
  EXPECT_EQ(c3d::pose::ManifestProblem(m), "");
}

// Had tf published the gimbal's links, its continuous joints would carry what Yahboom wrote:
// effort and velocity 0, and no position limits, since a continuous joint has none.
TEST(RigManifestTier2, AContinuousJointCarriesItsLimitsAsWrittenAndNoPositionLimits) {
  std::vector<RigFrame> tree = RaspbotTree();
  tree.push_back({"arm1_Link", "base_link"});
  tree.push_back({"arm2_Link", "arm1_Link"});
  const c3d::pose::Manifest m =
      Decorated(tree, Description(UrdfFixture("raspbot_v2_yahboom.urdf")));
  ASSERT_EQ(JointNames(m), (std::vector<std::string>{"base_link", "arm1_Link", "arm2_Link",
                                                     "camera_link", "camera_optical_frame",
                                                     "laser"}));
  for (const auto& [j, joint] : {std::pair<int, std::string>{1, "arm1_Joint"}, {2, "arm2_Joint"}}) {
    const c3d::Properties& p = m.joints[j].properties;
    EXPECT_EQ(Property(p, "ros.jointName"), joint);
    EXPECT_EQ(Property(p, "ros.jointType"), "continuous");
    EXPECT_EQ(Number(p, "ros.limit.effort"), 0.0) << joint;
    EXPECT_EQ(Number(p, "ros.limit.velocity"), 0.0) << joint;
    EXPECT_EQ(Keys(p), (std::vector<std::string>{"ros.parent", "ros.jointName", "ros.jointType",
                                                 "ros.limit.effort", "ros.limit.velocity"}))
        << joint << ": no lower or upper, which would pin the gimbal at 0";
  }
}

// A URDF 1.2 arm whose tf tree is its links plus a camera on the tool: each URDF link gets its
// joint's name, type and finite limits; the camera, which no URDF joint has as a child, keeps
// tier 1 only.
TEST(RigManifestTier2, EachLinkCarriesItsJointAndOnlyItsFiniteLimits) {
  const std::vector<RigFrame> tree = {
      {"base_link", "odom"},        {"shoulder_link", "base_link"}, {"elbow_link", "shoulder_link"},
      {"slider_link", "elbow_link"}, {"tool_link", "slider_link"},  {"camera_link", "tool_link"},
      {"wheel_link", "base_link"},  {"plane_link", "base_link"},    {"float_link", "base_link"},
      {"odom", ""}};
  const c3d::pose::Manifest m =
      Decorated(tree, Description(UrdfFixture("arm_urdf_1_2_partial_limits.urdf")));
  EXPECT_EQ(Property(m.properties, "ros.robotName"), "arm_v12");
  EXPECT_EQ(Property(m.properties, "c3d.model.formatVersion"), "1.2");
  std::map<std::string, const c3d::Properties*> by_name;
  for (const auto& joint : m.joints) by_name[joint.mesh_name] = &joint.properties;
  ASSERT_EQ(by_name.size(), 9u);

  EXPECT_EQ(Keys(*by_name["base_link"]), std::vector<std::string>{}) << "the root";
  const c3d::Properties& shoulder = *by_name["shoulder_link"];
  EXPECT_EQ(Property(shoulder, "ros.jointName"), "shoulder_joint");
  EXPECT_EQ(Property(shoulder, "ros.jointType"), "revolute");
  EXPECT_EQ(Number(shoulder, "ros.limit.lower"), -1.5);
  EXPECT_EQ(Number(shoulder, "ros.limit.upper"), 1.5);
  EXPECT_EQ(Number(shoulder, "ros.limit.velocity"), 2.0);
  EXPECT_EQ(Number(shoulder, "ros.limit.acceleration"), 4.0);
  EXPECT_EQ(Number(shoulder, "ros.limit.deceleration"), 4.0) << "acceleration's, unset";
  EXPECT_FALSE(Has(shoulder, "ros.limit.effort")) << "unset in 1.2 is unbounded: omitted";
  EXPECT_FALSE(Has(shoulder, "ros.limit.jerk"));
  EXPECT_EQ(Keys(*by_name["elbow_link"]),
            (std::vector<std::string>{"ros.parent", "ros.jointName", "ros.jointType",
                                      "ros.limit.lower", "ros.limit.upper", "ros.limit.effort",
                                      "ros.limit.velocity", "ros.limit.acceleration",
                                      "ros.limit.deceleration", "ros.limit.jerk"}));
  EXPECT_EQ(Number(*by_name["elbow_link"], "ros.limit.jerk"), 50.0);
  EXPECT_EQ(Keys(*by_name["slider_link"]),
            (std::vector<std::string>{"ros.parent", "ros.jointName", "ros.jointType",
                                      "ros.limit.lower", "ros.limit.upper"}));
  EXPECT_EQ(Keys(*by_name["wheel_link"]),
            (std::vector<std::string>{"ros.parent", "ros.jointName", "ros.jointType",
                                      "ros.limit.velocity"}))
      << "effort=\"inf\" is omitted like an unset one";
  EXPECT_EQ(Keys(*by_name["plane_link"]),
            (std::vector<std::string>{"ros.parent", "ros.jointName", "ros.jointType",
                                      "ros.limit.effort"}));
  EXPECT_EQ(Property(*by_name["tool_link"], "ros.jointType"), "fixed");
  EXPECT_EQ(Property(*by_name["float_link"], "ros.jointType"), "floating");
  EXPECT_EQ(Property(*by_name["plane_link"], "ros.jointType"), "planar");
  EXPECT_EQ(Property(*by_name["wheel_link"], "ros.jointType"), "continuous");
  EXPECT_EQ(Property(*by_name["slider_link"], "ros.jointType"), "prismatic");
  EXPECT_EQ(Keys(*by_name["camera_link"]), (std::vector<std::string>{"ros.parent", "c3d.kind"}))
      << "a frame outside the description keeps tier 1 only";
  EXPECT_EQ(c3d::pose::ManifestProblem(m), "");
}

TEST(RigManifestTier2, AValueThePlatformCouldNotTakeIsLeftOutAndTheRigSurvives) {
  const std::string long_name(c3d::pose::kMaxPropertyStringBytes + 1, 'n');
  const auto description = Description(
      "<robot name=\"" + long_name + "\" version=\"1.2\"><link name=\"base_link\"/>"
      "<link name=\"laser\"/><link name=\"camera_link\"/>"
      "<joint name=\"" + long_name + "\" type=\"revolute\"><parent link=\"base_link\"/>"
      "<child link=\"laser\"/><limit lower=\"-1\" upper=\"1\" effort=\"1e39\"/></joint>"
      "<joint name=\"pan\" type=\"revolute\"><parent link=\"base_link\"/>"
      "<child link=\"camera_link\"/><limit lower=\"-1e300\" upper=\"0.5\"/></joint></robot>");
  const c3d::pose::Manifest m = Decorated(RaspbotTree(), description);
  EXPECT_EQ(c3d::pose::ManifestProblem(m), "");
  EXPECT_EQ(Property(m.properties, "ros.robotName"), "raspbot-01")
      << "a name past the platform's string limit keeps the device_id";
  EXPECT_EQ(Property(m.properties, "c3d.model.format"), "urdf");
  const c3d::Properties& laser = m.joints[3].properties;
  EXPECT_FALSE(Has(laser, "ros.jointName"));
  EXPECT_EQ(Property(laser, "ros.jointType"), "revolute");
  EXPECT_FALSE(Has(laser, "ros.limit.effort")) << "past float32";
  EXPECT_EQ(Number(laser, "ros.limit.upper"), 1.0);
  EXPECT_FALSE(Has(m.joints[1].properties, "ros.limit.lower")) << "past float32";
  EXPECT_EQ(Number(m.joints[1].properties, "ros.limit.upper"), 0.5);
}

// robot_frame is often a URDF joint's child (base_footprint -> base_link). It is still the rig's
// root, posed in the reference frame, so no joint keys go on it.
TEST(RigManifestTier2, TheRootCarriesNoJointKeysThoughTheDescriptionHasAJointForIt) {
  const c3d::pose::Manifest m = Decorated(
      RaspbotTree(),
      Description("<robot name=\"r\"><link name=\"base_footprint\"/><link name=\"base_link\"/>"
                  "<link name=\"laser\"/><joint name=\"footprint\" type=\"fixed\">"
                  "<parent link=\"base_footprint\"/><child link=\"base_link\"/></joint>"
                  "<joint name=\"laser_mount\" type=\"fixed\"><parent link=\"base_link\"/>"
                  "<child link=\"laser\"/></joint></robot>"));
  EXPECT_EQ(Keys(m.joints[0].properties), std::vector<std::string>{});
  EXPECT_EQ(Property(m.joints[3].properties, "ros.jointName"), "laser_mount");
}

// A description in another encoding than UTF-8 parses, and its names cannot go on the wire. The
// manifest the platform would refuse is never sent: it stays tier 1.
TEST(RigManifestTier2, ADescriptionWhoseNamesAreNotUtf8LeavesTheManifestTier1) {
  const c3d::pose::Manifest tier1 = Decorated(RaspbotTree(), nullptr);
  const c3d::pose::Manifest m = Decorated(
      RaspbotTree(),
      Description("<robot name=\"r\"><link name=\"base_link\"/><link name=\"laser\"/>"
                  "<joint name=\"m\xe9tal\" type=\"fixed\"><parent link=\"base_link\"/>"
                  "<child link=\"laser\"/></joint></robot>"));
  EXPECT_EQ(Keys(m.properties), Keys(tier1.properties));
  EXPECT_EQ(Property(m.properties, "ros.robotName"), "raspbot-01");
  for (std::size_t j = 0; j < m.joints.size(); ++j) {
    EXPECT_EQ(Keys(m.joints[j].properties), Keys(tier1.joints[j].properties)) << j;
  }
  EXPECT_EQ(c3d::pose::ManifestProblem(m), "");
}

// The gripper's <ros2_control>, <transmission> and <gazebo> carry an address, serial ports and a
// password. None of it is in any key or value of the manifest the description decorates.
TEST(RigManifestTier2, NothingOutsideTheWhitelistReachesTheManifest) {
  const std::string xml = UrdfFixture("gripper_with_ros2_control.urdf");
  const c3d::pose::Manifest m = Decorated(
      {{"base_link", "map"}, {"finger_link", "base_link"}, {"map", ""}}, Description(xml));
  std::string all = m.manifest_id + m.name + m.mesh + m.file_type + m.root;
  const auto add = [&all](const c3d::Properties& properties) {
    for (const auto& [key, value] : properties) {
      all += "|" + key + "=" +
             (value.kind() == c3d::PropertyValue::Kind::kString ? value.as_string() : "#");
    }
  };
  add(m.properties);
  for (const auto& joint : m.joints) {
    all += "|" + joint.mesh_name;
    add(joint.properties);
  }
  EXPECT_NE(all.find("ros.jointName=finger_joint"), std::string::npos) << all;
  EXPECT_EQ(Property(m.joints[1].properties, "ros.jointType"), "prismatic");
  for (const char* secret :
       {"192.0.2", "SECRET", "ttyACM", "ttyUSB", "hunter2", "acme", "Acme", "package://"}) {
    EXPECT_EQ(all.find(secret), std::string::npos) << secret << " in: " << all;
  }
  EXPECT_EQ(Number(m.joints[1].properties, "ros.limit.upper"), 0.04)
      << "finger_joint's own limit, not the one on the <joint> nested in <ros2_control>";
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

namespace {

// The FakeTf Raspbot as a URDF would describe it, with its gimbal as a revolute joint; the laser
// is not in it.
std::string GimbalUrdf(const std::string& upper) {
  return "<robot name=\"raspbot\"><link name=\"base_link\"/><link name=\"camera_link\"/>"
         "<link name=\"camera_optical_frame\"/>"
         "<joint name=\"gimbal_pan\" type=\"revolute\"><parent link=\"base_link\"/>"
         "<child link=\"camera_link\"/><limit lower=\"-1.5\" upper=\"" + upper +
         "\" effort=\"1\" velocity=\"2\"/></joint>"
         "<joint name=\"optical\" type=\"fixed\"><parent link=\"camera_link\"/>"
         "<child link=\"camera_optical_frame\"/></joint></robot>";
}

}  // namespace

TEST_F(TfRigSamplerTest, ADescriptionSetBeforeTheFirstTickDecoratesTheFirstManifest) {
  TfRigSampler sampler(tf_, config_);
  sampler.SetDescription(Description(GimbalUrdf("1.5")));
  Raspbot(100.0);
  Tick(sampler, kWire0);
  ASSERT_EQ(sink_.manifests.size(), 1u);
  const c3d::pose::Manifest& m = sink_.manifests[0];
  EXPECT_EQ(Property(m.properties, "ros.robotName"), "raspbot");
  EXPECT_EQ(Property(m.joints[1].properties, "ros.jointName"), "gimbal_pan");
  EXPECT_EQ(Property(m.joints[2].properties, "ros.jointName"), "optical");
  EXPECT_FALSE(Has(m.joints[3].properties, "ros.jointName")) << "laser is not in the URDF";
  Raspbot(100.1);
  Tick(sampler, kWire0 + 0.1);
  EXPECT_EQ(sink_.manifests.size(), 1u);
}

// A description that arrives or changes mid-session re-sends the manifest with the same joints,
// at the next tick, and the samples go on under it. The same description again sends nothing;
// a description withdrawn sends the tier 1 manifest.
TEST_F(TfRigSamplerTest, ANewDescriptionMidSessionReSendsTheManifestWithTheSameJoints) {
  TfRigSampler sampler(tf_, config_);
  Raspbot(100.0);
  Tick(sampler, kWire0);
  ASSERT_EQ(sink_.manifests.size(), 1u);
  EXPECT_FALSE(Has(sink_.manifests[0].properties, "ros.urdfSha256")) << "tf only, at first";

  const auto first = Description(GimbalUrdf("1.5"));
  sampler.SetDescription(first);
  Raspbot(100.1);
  ASSERT_EQ(Tick(sampler, kWire0 + 0.1), TfRigSampler::Outcome::kRecorded);
  ASSERT_EQ(sink_.manifests.size(), 2u);
  EXPECT_EQ(JointNames(sink_.manifests[1]), JointNames(sink_.manifests[0]));
  EXPECT_EQ(Property(sink_.manifests[1].properties, "ros.urdfSha256"), first->sha256);
  EXPECT_EQ(Number(sink_.manifests[1].joints[1].properties, "ros.limit.upper"), 1.5);
  EXPECT_EQ(sink_.samples.size(), 2u);

  Raspbot(100.2);
  Tick(sampler, kWire0 + 0.2);
  sampler.SetDescription(Description(GimbalUrdf("1.5")));
  Raspbot(100.3);
  Tick(sampler, kWire0 + 0.3);
  EXPECT_EQ(sink_.manifests.size(), 2u)
      << "the same description, delivered again, is not a change";

  sampler.SetDescription(Description(GimbalUrdf("1.25")));
  Raspbot(100.4);
  Tick(sampler, kWire0 + 0.4);
  ASSERT_EQ(sink_.manifests.size(), 3u);
  EXPECT_EQ(Number(sink_.manifests[2].joints[1].properties, "ros.limit.upper"), 1.25);
  EXPECT_NE(Property(sink_.manifests[2].properties, "ros.urdfSha256"), first->sha256);

  sampler.SetDescription(nullptr);
  Raspbot(100.5);
  Tick(sampler, kWire0 + 0.5);
  ASSERT_EQ(sink_.manifests.size(), 4u);
  EXPECT_EQ(Keys(sink_.manifests[3].properties),
            (std::vector<std::string>{"c3d.profile", "ros.robotName"}));
  EXPECT_EQ(Property(sink_.manifests[3].properties, "ros.robotName"), "raspbot-01");
  EXPECT_EQ(sink_.samples.size(), 6u);
  EXPECT_EQ(sampler.manifest_sends(), 4u);
}

TEST_F(TfRigSamplerTest, ARefusedReSendIsTriedAgainAndTheDescriptionOutlivesTheSession) {
  TfRigSampler sampler(tf_, config_);
  Raspbot(100.0);
  Tick(sampler, kWire0);
  sampler.SetDescription(Description(GimbalUrdf("1.5")));
  sink_.refuse_manifests = true;
  Raspbot(100.1);
  EXPECT_EQ(Tick(sampler, kWire0 + 0.1), TfRigSampler::Outcome::kRefused);
  sink_.refuse_manifests = false;
  Raspbot(100.2);
  Tick(sampler, kWire0 + 0.2);
  ASSERT_EQ(sink_.manifests.size(), 2u);
  EXPECT_TRUE(Has(sink_.manifests[1].properties, "ros.urdfSha256"));

  sampler.Reset();
  Raspbot(100.3);
  Tick(sampler, kWire0 + 0.3);
  ASSERT_EQ(sink_.manifests.size(), 3u);
  EXPECT_TRUE(Has(sink_.manifests[2].properties, "ros.urdfSha256"))
      << "the next session's first manifest is decorated: the robot did not change";
  Raspbot(100.4);
  Tick(sampler, kWire0 + 0.4);
  EXPECT_EQ(sink_.manifests.size(), 3u);
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
