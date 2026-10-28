// The robot description parser: what it reads from a URDF, how each format version resolves a
// joint's limits, what it refuses, and that nothing outside its whitelist survives into anything
// it returns or says. Three descriptions under test/fixtures: Yahboom's stock Raspbot V2 URDF
// byte for byte, a URDF 1.2 arm with limits given in part, and a gripper whose <ros2_control>,
// <transmission> and <gazebo> carry made-up addresses, serial ports and a password.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

#include "cognitive3d_ros/urdf_description.hpp"

using namespace cognitive3d_ros;

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

std::string Fixture(const std::string& name) {
  const char* dir = std::getenv("C3D_URDF_FIXTURES");
  EXPECT_NE(dir, nullptr) << "C3D_URDF_FIXTURES is set by the test's CMake entry";
  std::ifstream in(std::string(dir == nullptr ? "." : dir) + "/" + name, std::ios::binary);
  EXPECT_TRUE(in.good()) << name;
  return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

UrdfDescription Parse(const std::string& xml) {
  UrdfDescription description;
  std::string error;
  EXPECT_TRUE(ParseUrdfDescription(xml, &description, &error)) << error;
  return description;
}

std::string Refusal(const std::string& xml) {
  UrdfDescription description;
  std::string error;
  EXPECT_FALSE(ParseUrdfDescription(xml, &description, &error)) << xml;
  EXPECT_TRUE(description.joints.empty() && description.robot_name.empty())
      << "a refused description leaves nothing behind";
  return error;
}

const UrdfJoint& Joint(const UrdfDescription& description, const std::string& name) {
  for (const UrdfJoint& joint : description.joints) {
    if (joint.name == name) return joint;
  }
  ADD_FAILURE() << "no joint " << name;
  static const UrdfJoint none;
  return none;
}

// One joint between two links, with `limit` as the joint's <limit> element ("" for none).
std::string OneJoint(const std::string& version, const std::string& type,
                     const std::string& limit) {
  return "<robot name=\"r\"" + (version.empty() ? std::string() : " version=\"" + version + "\"") +
         "><link name=\"a\"/><link name=\"b\"/><joint name=\"j\" type=\"" + type +
         "\"><parent link=\"a\"/><child link=\"b\"/>" + limit + "</joint></robot>";
}

void ExpectLimits(const UrdfLimits& got, const UrdfLimits& want, const std::string& what) {
  EXPECT_EQ(got.lower, want.lower) << what << " lower";
  EXPECT_EQ(got.upper, want.upper) << what << " upper";
  EXPECT_EQ(got.effort, want.effort) << what << " effort";
  EXPECT_EQ(got.velocity, want.velocity) << what << " velocity";
  EXPECT_EQ(got.acceleration, want.acceleration) << what << " acceleration";
  EXPECT_EQ(got.deceleration, want.deceleration) << what << " deceleration";
  EXPECT_EQ(got.jerk, want.jerk) << what << " jerk";
}

UrdfLimits Limits(double lower, double upper, double effort, double velocity,
                  double acceleration = kInf, double deceleration = kInf, double jerk = kInf) {
  UrdfLimits limits;
  limits.lower = lower;
  limits.upper = upper;
  limits.effort = effort;
  limits.velocity = velocity;
  limits.acceleration = acceleration;
  limits.deceleration = deceleration;
  limits.jerk = jerk;
  return limits;
}

}  // namespace

// ---- Yahboom's Raspbot V2 description, as their image ships it ------------------------------

TEST(UrdfDescription, ReadsYahboomsRaspbotAsShipped) {
  const std::string xml = Fixture("raspbot_v2_yahboom.urdf");
  ASSERT_EQ(xml.size(), 9569u) << "the fixture is the file as shipped, CRLF line ends and all";
  const UrdfDescription d = Parse(xml);
  EXPECT_EQ(d.robot_name, "Raspbot-V2");
  EXPECT_EQ(d.format_version, "1.0") << "no version attribute is URDF 1.0";
  // Computed with `shasum -a 256` over the shipped file, not with the SDK's own SHA-256.
  EXPECT_EQ(d.sha256, "e18f29dc721ddf4b8fa485e08ffef60fca90e634572ce612210901fbae3d3ce6");
  EXPECT_EQ(d.links, (std::vector<std::string>{"base_link", "l1_Link", "l2__Link", "r1_Link",
                                               "r2_Link", "arm1_Link", "arm2_Link"}))
      << "l2__Link keeps its double underscore";
  ASSERT_EQ(d.joints.size(), 6u);
  for (const UrdfJoint& joint : d.joints) {
    EXPECT_EQ(joint.type, "continuous") << joint.name;
    EXPECT_FALSE(joint.mimic) << joint.name;
  }
  EXPECT_EQ(Joint(d, "l2_Joint").parent, "base_link");
  EXPECT_EQ(Joint(d, "l2_Joint").child, "l2__Link");
  EXPECT_EQ(Joint(d, "arm2_Joint").parent, "arm1_Link");
  EXPECT_FALSE(Joint(d, "l1_Joint").has_limit);
  ExpectLimits(Joint(d, "l1_Joint").limits, UrdfLimits(), "l1_Joint, which has no <limit>");
  // The gimbal's `<limit lower="0" upper="0" effort="0" velocity="0">` on a continuous joint:
  // effort and velocity as written, and no position limits, since a continuous joint has none.
  // A reader that took 0/0 as the range would pin the gimbal at zero.
  for (const char* gimbal : {"arm1_Joint", "arm2_Joint"}) {
    EXPECT_TRUE(Joint(d, gimbal).has_limit) << gimbal;
    ExpectLimits(Joint(d, gimbal).limits, Limits(-kInf, kInf, 0.0, 0.0), gimbal);
  }
  EXPECT_EQ(d.JointWithChild("arm2_Link"), &Joint(d, "arm2_Joint"));
  EXPECT_EQ(d.JointWithChild("base_link"), nullptr) << "the root is no joint's child";
  EXPECT_EQ(d.JointWithChild("camera_link"), nullptr) << "not in Yahboom's description";
  EXPECT_EQ(DescribeUrdf(d),
            "robot 'Raspbot-V2', URDF 1.0, 7 link(s), 6 joint(s), sha256 e18f29dc721d");
}

TEST(UrdfDescription, TheHashIsOverTheBytesAsReceived) {
  const std::string crlf = Fixture("raspbot_v2_yahboom.urdf");
  std::string lf;
  for (const char c : crlf) {
    if (c != '\r') lf += c;
  }
  EXPECT_NE(Parse(lf).sha256, Parse(crlf).sha256)
      << "the same robot with other line ends is another description";
  EXPECT_EQ(Parse(lf).robot_name, "Raspbot-V2");
}

// ---- URDF 1.2: every unset limit unbounded ---------------------------------------------------

TEST(UrdfDescription, Urdf12ResolvesEachUnsetLimitToUnbounded) {
  const UrdfDescription d = Parse(Fixture("arm_urdf_1_2_partial_limits.urdf"));
  EXPECT_EQ(d.robot_name, "arm_v12");
  EXPECT_EQ(d.format_version, "1.2");
  ASSERT_EQ(d.joints.size(), 7u);
  // No effort: unbounded. Deceleration not given: acceleration's.
  ExpectLimits(Joint(d, "shoulder_joint").limits, Limits(-1.5, 1.5, kInf, 2.0, 4.0, 4.0, kInf),
               "shoulder_joint");
  ExpectLimits(Joint(d, "elbow_joint").limits, Limits(-2.0, 2.0, 30, 1.5, 3.0, 6.0, 50),
               "elbow_joint, every limit given");
  EXPECT_TRUE(Joint(d, "elbow_joint").mimic);
  EXPECT_FALSE(Joint(d, "shoulder_joint").mimic);
  ExpectLimits(Joint(d, "slider_joint").limits, Limits(0.0, 0.3, kInf, kInf), "slider_joint");
  // `effort="inf"` written out is unbounded too.
  ExpectLimits(Joint(d, "wheel_joint").limits, Limits(-kInf, kInf, kInf, 10.0), "wheel_joint");
  // Position limits belong to revolute and prismatic joints only.
  ExpectLimits(Joint(d, "plane_joint").limits, Limits(-kInf, kInf, 5.0, kInf), "plane_joint");
  for (const char* free : {"tool_joint", "float_joint"}) {
    EXPECT_EQ(Joint(d, free).type, std::string(free) == "tool_joint" ? "fixed" : "floating");
    EXPECT_FALSE(Joint(d, free).has_limit) << free;
    ExpectLimits(Joint(d, free).limits, UrdfLimits(), free);
  }
}

TEST(UrdfDescription, Urdf12RequiresBothPositionLimitsOnRevoluteAndPrismatic) {
  for (const char* type : {"revolute", "prismatic"}) {
    EXPECT_NE(Refusal(OneJoint("1.2", type, "<limit lower=\"0\"/>")).find("upper"),
              std::string::npos)
        << type;
    EXPECT_NE(Refusal(OneJoint("1.2", type, "")).find("no <limit>"), std::string::npos) << type;
  }
  EXPECT_EQ(Parse(OneJoint("1.2", "revolute", "<limit lower=\"-1\" upper=\"1\"/>"))
                .joints[0]
                .limits.effort,
            kInf);
}

// ---- URDF 1.0 and 1.1: an unset position limit is 0 -----------------------------------------

TEST(UrdfDescription, BeforeUrdf12AnUnsetPositionLimitIsZeroAndEffortAndVelocityAreRequired) {
  for (const char* version : {"", "1.0", "1.1"}) {
    const std::string what = std::string("version '") + version + "'";
    const UrdfDescription d =
        Parse(OneJoint(version, "revolute", "<limit effort=\"3\" velocity=\"4\"/>"));
    EXPECT_EQ(d.format_version, *version == '\0' ? "1.0" : version) << what;
    ExpectLimits(d.joints[0].limits, Limits(0.0, 0.0, 3.0, 4.0), what);
    // 1.2's limits are not part of these versions: written anyway, they stay unbounded.
    ExpectLimits(Parse(OneJoint(version, "prismatic",
                                "<limit lower=\"-1\" effort=\"3\" velocity=\"4\" "
                                "acceleration=\"9\" deceleration=\"9\" jerk=\"9\"/>"))
                     .joints[0]
                     .limits,
                 Limits(-1.0, 0.0, 3.0, 4.0), what + " with 1.2 attributes");
    EXPECT_NE(Refusal(OneJoint(version, "revolute", "<limit velocity=\"4\"/>")).find("effort"),
              std::string::npos)
        << what;
    EXPECT_NE(Refusal(OneJoint(version, "continuous", "<limit effort=\"4\"/>")).find("velocity"),
              std::string::npos)
        << what;
    EXPECT_NE(Refusal(OneJoint(version, "prismatic", "")).find("no <limit>"), std::string::npos)
        << what;
    EXPECT_FALSE(Parse(OneJoint(version, "continuous", "")).joints[0].has_limit) << what;
  }
}

TEST(UrdfDescription, ALimitReadsAsANumberInTheCLocaleOrIsRefused) {
  const auto effort = [](const std::string& text) {
    UrdfDescription d;
    std::string error;
    const bool ok = ParseUrdfDescription(
        OneJoint("1.2", "continuous", "<limit effort=\"" + text + "\"/>"), &d, &error);
    return ok ? d.joints[0].limits.effort : std::nan("");
  };
  EXPECT_EQ(effort("2.5"), 2.5);
  EXPECT_EQ(effort(" +2.5 "), 2.5);
  EXPECT_EQ(effort("-1e-3"), -1e-3);
  EXPECT_EQ(effort("INF"), kInf);
  for (const char* bad : {"", "2,5", "nan", "1e400", "2.5 kg", "++1", "0x10x"}) {
    EXPECT_TRUE(std::isnan(effort(bad))) << "'" << bad << "'";
  }
}

// ---- What is refused, and what a refusal says ------------------------------------------------

TEST(UrdfDescription, ADescriptionTheFormatRefusesIsRefusedWhole) {
  const std::vector<std::pair<std::string, std::string>> cases = {
      {"not xml <robot", "not well-formed XML"},
      {"", "not well-formed XML"},
      {"<model name=\"r\"/>", "not <robot>"},
      {"<robot/>", "no name"},
      {"<robot name=\"r\" version=\"2.0\"/>", "1.0, 1.1 or 1.2"},
      {OneJoint("", "hinge", ""), "joint 'j' is not of a URDF joint type"},
      {"<robot name=\"r\"><link name=\"a\"/><joint name=\"j\" type=\"fixed\"><parent link=\"a\"/>"
       "<child link=\"b\"/></joint></robot>",
       "names a link the description does not declare"},
      {"<robot name=\"r\"><link name=\"a\"/><link name=\"a\"/></robot>", "declared twice"},
      {"<robot name=\"r\"><link name=\"a\"/><link name=\"b\"/><link name=\"c\"/>"
       "<joint name=\"j\" type=\"fixed\"><parent link=\"a\"/><child link=\"c\"/></joint>"
       "<joint name=\"k\" type=\"fixed\"><parent link=\"b\"/><child link=\"c\"/></joint></robot>",
       "child of both joint 'j' and joint 'k'"},
      {OneJoint("", "fixed", "").replace(OneJoint("", "fixed", "").find("<child"), 17, ""),
       "no <child link>"},
  };
  for (const auto& [xml, said] : cases) {
    EXPECT_NE(Refusal(xml).find(said), std::string::npos) << xml << " -> " << Refusal(xml);
  }
}

// ---- The whitelist ---------------------------------------------------------------------------

namespace {

// Every string the parse result holds, joined, plus the log line.
std::string EverythingReturned(const UrdfDescription& d) {
  std::string all = d.robot_name + "|" + d.format_version + "|" + d.sha256 + "|" + DescribeUrdf(d);
  for (const auto& link : d.links) all += "|" + link;
  for (const auto& joint : d.joints) {
    all += "|" + joint.name + "|" + joint.type + "|" + joint.parent + "|" + joint.child;
  }
  return all;
}

const char* const kSecrets[] = {"192.0.2", "SECRET", "ttyACM", "ttyUSB", "hunter2",
                                "acme",    "Acme",   "package://", "/dev/"};

}  // namespace

TEST(UrdfDescription, NothingOutsideTheWhitelistSurvivesTheParse) {
  const std::string xml = Fixture("gripper_with_ros2_control.urdf");
  ASSERT_NE(xml.find("192.0.2.77"), std::string::npos) << "the fixture carries what is tested";
  const UrdfDescription d = Parse(xml);
  EXPECT_EQ(d.robot_name, "gripper_cell");
  EXPECT_EQ(d.links, (std::vector<std::string>{"base_link", "finger_link"}));
  ASSERT_EQ(d.joints.size(), 1u)
      << "a <joint> inside <ros2_control> or <transmission> is not one of the robot's joints";
  EXPECT_EQ(d.joints[0].name, "finger_joint");
  ExpectLimits(d.joints[0].limits, Limits(0.0, 0.04, 20.0, 0.1), "finger_joint");
  const std::string returned = EverythingReturned(d);
  for (const char* secret : kSecrets) {
    EXPECT_EQ(returned.find(secret), std::string::npos) << secret << " in: " << returned;
  }
}

TEST(UrdfDescription, ARefusalNeverQuotesTheDescription) {
  const std::string xml = Fixture("gripper_with_ros2_control.urdf");
  std::vector<std::string> broken = {
      // Malformed inside <ros2_control>, where tinyxml2's own message would name the element.
      [&] {
        std::string x = xml;
        return x.replace(x.find("</hardware>"), 11, "</hardware_192.0.2.77>");
      }(),
      // A robot-level limit that holds a secret instead of a number.
      [&] {
        std::string x = xml;
        return x.replace(x.find("effort=\"20\""), 11, "effort=\"192.0.2.77\"");
      }(),
      // A malformed element whose own name is the secret, which tinyxml2's message names.
      [&] {
        std::string x = xml;
        const std::string state = "<state_interface name=\"position\"/>";
        return x.replace(x.find(state), state.size(), "<SECRET_192.0.2.77 value=bare/>");
      }(),
      // An attribute value that is not closed, inside <transmission>.
      [&] {
        std::string x = xml;
        return x.replace(x.find("SECRET_tx_192.0.2.78\""), 21, "SECRET_tx_192.0.2.78");
      }(),
  };
  for (const std::string& x : broken) {
    const std::string error = Refusal(x);
    EXPECT_FALSE(error.empty());
    for (const char* secret : kSecrets) {
      EXPECT_EQ(error.find(secret), std::string::npos) << secret << " in: " << error;
    }
  }
}
