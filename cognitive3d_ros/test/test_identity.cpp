// The host identity every ROS-side producer of a session sends.
//
// Every case asserts the VALUE sent, not merely that `c3d.device.os` is non-empty: a parser that
// read `NAME=` instead of `PRETTY_NAME=` would send "Ubuntu" instead of "Ubuntu 24.04.1 LTS" and
// pass any non-empty check. Everything here drives files on disk rather than the host's own
// /etc/os-release, so the cases that matter (absent, stateless, single-quoted) are reachable.
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>
#include <rmw/rmw.h>

#include "cognitive3d_ros/identity.hpp"

namespace fs = std::filesystem;
using cognitive3d_ros::HostOperatingSystem;
using cognitive3d_ros::RosDiscoveryRange;
using cognitive3d_ros::RosDistro;
using cognitive3d_ros::RosDomainId;

namespace {

class IdentityTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = fs::temp_directory_path() /
           ("c3d_identity_" + std::to_string(::getpid()) + "_" + std::to_string(counter_++));
    fs::create_directories(dir_);
  }
  void TearDown() override {
    std::error_code ec;
    fs::remove_all(dir_, ec);
  }
  // Writes an os-release file and returns its path. `nullptr` means "this file does not exist".
  std::string Release(const char* name, const char* contents) {
    const auto path = dir_ / name;
    if (contents == nullptr) return path.string();
    std::ofstream(path) << contents;
    return path.string();
  }
  fs::path dir_;
  static int counter_;
};
int IdentityTest::counter_ = 0;

// A real Ubuntu file: NAME comes BEFORE PRETTY_NAME, which is what makes the key a real choice.
constexpr const char* kUbuntu =
    "PRETTY_NAME=\"Ubuntu 24.04.1 LTS\"\n"
    "NAME=\"Ubuntu\"\n"
    "VERSION_ID=\"24.04\"\n";

TEST_F(IdentityTest, ReadsPrettyNameAndNotSomeOtherKeyThatEndsInNAME) {
  // Order reversed against kUbuntu so neither "first line wins" nor "last line wins" can pass by
  // accident, and so reading `NAME=` instead of `PRETTY_NAME=` yields "Ubuntu" and fails loudly.
  const auto path = Release("os-release",
                            "NAME=\"Ubuntu\"\n"
                            "PRETTY_NAME=\"Ubuntu 24.04.1 LTS\"\n");
  EXPECT_EQ(HostOperatingSystem(path, "/nonexistent"), "Ubuntu 24.04.1 LTS");
}

TEST_F(IdentityTest, StripsSingleQuotesAsWellAsDouble) {
  // os-release(5) permits both. A parser that strips only double quotes sends a single-quoted
  // value with its apostrophes -- a dashboard value of its own, separate from every other session
  // on the same OS.
  EXPECT_EQ(HostOperatingSystem(Release("os-release", "PRETTY_NAME='Acme Robot OS 1'\n"), "/none"),
            "Acme Robot OS 1");
}

TEST_F(IdentityTest, AnUnquotedValueIsTakenAsItStands) {
  EXPECT_EQ(HostOperatingSystem(Release("os-release", "PRETTY_NAME=Alpine\n"), "/none"), "Alpine");
}

TEST_F(IdentityTest, AMismatchedQuoteIsLeftAloneRatherThanHalfStripped) {
  EXPECT_EQ(HostOperatingSystem(Release("os-release", "PRETTY_NAME=\"Truncated\n"), "/none"),
            "\"Truncated");
}

TEST_F(IdentityTest, ACarriageReturnDoesNotSurviveIntoTheValue) {
  // With the CR still attached the closing quote is no longer last, so the quote strip also misses
  // and the value goes out as `Ubuntu 24.04.1 LTS"` with a stray quote AND a control character.
  EXPECT_EQ(HostOperatingSystem(Release("os-release", "PRETTY_NAME=\"Ubuntu 24.04.1 LTS\"\r\n"),
                                "/none"),
            "Ubuntu 24.04.1 LTS");
}

TEST_F(IdentityTest, FallsBackToUsrLibForAStatelessSystem) {
  // os-release(5) names /usr/lib/os-release as the fallback. Reading only /etc would report a
  // system that ships one as `unknown` while the answer is on disk.
  const auto missing = Release("os-release", nullptr);
  const auto fallback = Release("usr-lib-os-release", kUbuntu);
  EXPECT_EQ(HostOperatingSystem(missing, fallback), "Ubuntu 24.04.1 LTS");
}

TEST_F(IdentityTest, APresentButUselessPrimaryStillFallsBack) {
  const auto primary = Release("os-release", "NAME=\"Nothing useful\"\n");
  const auto fallback = Release("usr-lib-os-release", kUbuntu);
  EXPECT_EQ(HostOperatingSystem(primary, fallback), "Ubuntu 24.04.1 LTS");
}

TEST_F(IdentityTest, AnEmptyPrettyNameIsNotAnAnswer) {
  EXPECT_EQ(HostOperatingSystem(Release("os-release", "PRETTY_NAME=\"\"\n"), "/none"), "unknown");
  EXPECT_EQ(HostOperatingSystem(Release("os-release", "PRETTY_NAME=\n"), "/none"), "unknown");
}

TEST_F(IdentityTest, BothFilesAbsentReportsUnknownRatherThanNothing) {
  // An absent answer must not read as a wrong one, and an empty string on the wire reads as
  // "reported nothing" where a missing key reads as "never asked".
  EXPECT_EQ(HostOperatingSystem("/nonexistent/etc", "/nonexistent/usr"), "unknown");
}

TEST_F(IdentityTest, TheDefaultPathsAreTheSpecifiedOnes) {
  // The defaults are what every caller actually uses; a fixture-only test would say nothing about
  // them. Not an assertion on this machine's own OS -- only that the defaults resolve somewhere
  // and never yield an empty string.
  EXPECT_FALSE(HostOperatingSystem().empty());
}

TEST(RosDistroTest, AnEmptyVariableIsAsAbsentAsNoVariable) {
  // The VALUE decides, not the getenv POINTER: testing the pointer sends `export ROS_DISTRO=` as
  // an empty string, under `c3d.ros.distro` and under the cross-SDK `c3d.app.engine.version`,
  // which carries the same value.
  const char* const original = std::getenv("ROS_DISTRO");
  const std::string saved = original ? original : "";
  const bool had = original != nullptr;

  ::setenv("ROS_DISTRO", "", 1);
  EXPECT_EQ(RosDistro(), "unknown");
  ::unsetenv("ROS_DISTRO");
  EXPECT_EQ(RosDistro(), "unknown");
  ::setenv("ROS_DISTRO", "jazzy", 1);
  EXPECT_EQ(RosDistro(), "jazzy");

  if (had) {
    ::setenv("ROS_DISTRO", saved.c_str(), 1);
  } else {
    ::unsetenv("ROS_DISTRO");
  }
}

// A session that cannot name its domain and discovery range cannot establish whose data it holds:
// at the defaults, two robots on one network join one ROS graph and every measurement afterwards
// is the union of both, with nothing in the payload to reveal it.
TEST(RosDiscoveryIdentityTest, AnAbsentVariableIsReportedAbsentRatherThanResolved) {
  const char* const original = std::getenv("ROS_DOMAIN_ID");
  const std::string saved = original ? original : "";
  const bool had = original != nullptr;

  // "0" is the domain an absent ROS_DOMAIN_ID really produces, and reporting it here is exactly
  // the mistake: it would be indistinguishable from an operator having chosen 0, and the
  // difference between a value and an unmade decision is why this property exists.
  ::unsetenv("ROS_DOMAIN_ID");
  EXPECT_EQ(RosDomainId(), "unset");
  ::setenv("ROS_DOMAIN_ID", "", 1);
  EXPECT_EQ(RosDomainId(), "unset");
  ::setenv("ROS_DOMAIN_ID", "0", 1);
  EXPECT_EQ(RosDomainId(), "0");
  ::setenv("ROS_DOMAIN_ID", "42", 1);
  EXPECT_EQ(RosDomainId(), "42");

  if (had) {
    ::setenv("ROS_DOMAIN_ID", saved.c_str(), 1);
  } else {
    ::unsetenv("ROS_DOMAIN_ID");
  }
}

TEST(RosDiscoveryIdentityTest, TheDiscoveryRangeIsRecordedVerbatim) {
  const char* const original = std::getenv("ROS_AUTOMATIC_DISCOVERY_RANGE");
  const std::string saved = original ? original : "";
  const bool had = original != nullptr;

  ::unsetenv("ROS_AUTOMATIC_DISCOVERY_RANGE");
  EXPECT_EQ(RosDiscoveryRange(), "unset");
  ::setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "", 1);
  EXPECT_EQ(RosDiscoveryRange(), "unset");
  // Verbatim, not normalised. This is a transcript of the environment the run was given; a value
  // the local ROS does not recognise is exactly the thing worth being able to read back later.
  ::setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST", 1);
  EXPECT_EQ(RosDiscoveryRange(), "LOCALHOST");
  ::setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "localhost", 1);
  EXPECT_EQ(RosDiscoveryRange(), "localhost");
  ::setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "SUBNET", 1);
  EXPECT_EQ(RosDiscoveryRange(), "SUBNET");

  if (had) {
    ::setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", saved.c_str(), 1);
  } else {
    ::unsetenv("ROS_AUTOMATIC_DISCOVERY_RANGE");
  }
}

}  // namespace

TEST(RuntimeDescription, NamesTheDistroAndTheRmw) {
  const std::string described = cognitive3d_ros::RuntimeDescription();
  EXPECT_EQ(described.rfind("ROS 2 ", 0), 0u) << described;
  EXPECT_NE(described.find(cognitive3d_ros::RosDistro()), std::string::npos) << described;
  EXPECT_NE(described.find(rmw_get_implementation_identifier()), std::string::npos) << described;
}
