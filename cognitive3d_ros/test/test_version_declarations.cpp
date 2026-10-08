// `kSdkVersion` is the authoritative version: it is what reaches the wire as `c3d.version`, and it
// is the only one a stored session can be attributed by. `core/CMakeLists.txt` derives
// `project(VERSION)` from it, `Config::app_version` is derived from it, and the git tag is checked
// by the release workflow, being the one declaration nothing inside the repo can see. The three
// package.xml versions are checked here, in the ament package that sits above both, because
// `core` must not know that ROS exists. `tools/bump_version.sh` rewrites every declaration.
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "c3d/version.hpp"

namespace fs = std::filesystem;

namespace {

// The repo root, passed in by CMake. Not derived from the cwd: ctest's working directory is the
// build tree, and a test that quietly finds nothing there would report agreement it never checked.
const fs::path kRepoRoot{C3D_REPO_ROOT};

std::string ReadWhole(const fs::path& path) {
  std::ifstream in(path);
  return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

// The FIRST <version> element, which in a package.xml is the package's own. Deliberately not a
// regex over the whole file: a dependency's version constraint must not be able to satisfy this.
std::string PackageVersion(const std::string& xml) {
  const auto open = xml.find("<version>");
  if (open == std::string::npos) return {};
  const auto start = open + std::string("<version>").size();
  const auto close = xml.find("</version>", start);
  if (close == std::string::npos) return {};
  return xml.substr(start, close - start);
}

const std::vector<std::string> kManifests = {
    "cognitive3d_ros/package.xml",
    "cognitive3d_ros_msgs/package.xml",
    "cognitive3d_ros_nav2/package.xml",
};

}  // namespace

TEST(VersionDeclarations, EveryPackageManifestAgreesWithTheVersionThatReachesTheWire) {
  for (const auto& relative : kManifests) {
    const fs::path path = kRepoRoot / relative;
    ASSERT_TRUE(fs::exists(path))
        << path << " is not where this test was told the repo is (" << kRepoRoot
        << "). A version check that cannot find the files reports agreement it never made.";
    const std::string declared = PackageVersion(ReadWhole(path));
    ASSERT_FALSE(declared.empty()) << relative << " has no <version> element";
    EXPECT_EQ(declared, std::string(c3d::kSdkVersion))
        << relative << " says " << declared << " while core/include/c3d/version.hpp says "
        << c3d::kSdkVersion << ". kSdkVersion is authoritative: it is what goes on the wire as "
        << "c3d.version, so a session recorded by this build would claim a version no package "
        << "manifest declares. Run tools/bump_version.sh <x.y.z>.";
  }
}

TEST(VersionDeclarations, TheAuthoritativeVersionIsAVersion) {
  // core/CMakeLists.txt derives project(VERSION) from kSdkVersion, so that pair cannot disagree.
  // The assertion here is that the authoritative value is a version at all rather than a name, which
  // is the shape the wire check in core/test/test_session.cpp also pins.
  const std::string version{c3d::kSdkVersion};
  ASSERT_FALSE(version.empty());
  EXPECT_NE(version.find('.'), std::string::npos) << "kSdkVersion is not a version: " << version;
  EXPECT_EQ(version.find_first_not_of("0123456789.-+abcdefghijklmnopqrstuvwxyz"),
            std::string::npos)
      << "kSdkVersion has characters no semver carries: " << version;
}

TEST(VersionDeclarations, ThePackageVersionParserTakesThePackagesOwnAndNotADependencysConstraint) {
  // The parser is three lines and it is load-bearing, so it gets its own case rather than being
  // trusted because the real files happen to pass.
  EXPECT_EQ(PackageVersion("<package><name>x</name><version>1.2.3</version>"
                           "<depend version_gte=\"9.9.9\">y</depend></package>"),
            "1.2.3");
  EXPECT_EQ(PackageVersion("<package><name>x</name></package>"), "");
  EXPECT_EQ(PackageVersion("<version>1.2.3"), "");
}
