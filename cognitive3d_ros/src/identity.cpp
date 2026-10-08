#include "cognitive3d_ros/identity.hpp"

#include <cstdlib>
#include <fstream>

#include <rmw/rmw.h>

namespace cognitive3d_ros {
namespace {

constexpr const char* kUnknown = "unknown";

// Strips one matched pair of surrounding quotes. os-release(5) allows both forms; a value quoted
// with one and not the other is left alone rather than half-stripped.
std::string Unquote(std::string value) {
  if (value.size() >= 2 && value.front() == value.back() &&
      (value.front() == '"' || value.front() == '\'')) {
    value = value.substr(1, value.size() - 2);
  }
  return value;
}

std::string PrettyNameIn(const std::string& path) {
  std::ifstream release(path);
  std::string line;
  while (std::getline(release, line)) {
    // A CRLF file would otherwise put a carriage return inside the value, and the quote check
    // below would then not see the closing quote either.
    if (!line.empty() && line.back() == '\r') line.pop_back();
    constexpr const char* kKey = "PRETTY_NAME=";
    constexpr std::size_t kKeyLength = 12;
    if (line.rfind(kKey, 0) != 0) continue;
    const std::string value = Unquote(line.substr(kKeyLength));
    if (!value.empty()) return value;
  }
  return {};
}

// An empty variable counts as unset, for RosDistro's reason: an absent answer must not read as a
// wrong one.
std::string EnvOrUnset(const char* name) {
  const char* const value = std::getenv(name);
  if (value == nullptr || *value == '\0') return "unset";
  return std::string(value);
}

}  // namespace

std::string RosDistro() {
  const char* const distro = std::getenv("ROS_DISTRO");
  if (distro == nullptr || *distro == '\0') return kUnknown;
  return std::string(distro);
}

std::string RuntimeDescription() {
  const char* const rmw = rmw_get_implementation_identifier();
  return "ROS 2 " + RosDistro() + ", " + (rmw != nullptr ? rmw : kUnknown);
}

std::string RosDomainId() { return EnvOrUnset("ROS_DOMAIN_ID"); }

std::string RosDiscoveryRange() { return EnvOrUnset("ROS_AUTOMATIC_DISCOVERY_RANGE"); }

// The variable is read through the rmw discovery options, which arrived with this header.
bool DistroHasDiscoveryRange() {
#if __has_include(<rmw/discovery_options.h>)
  return true;
#else
  return false;
#endif
}

std::string HostOperatingSystem(const std::string& primary, const std::string& fallback) {
  std::string pretty = PrettyNameIn(primary);
  if (pretty.empty()) pretty = PrettyNameIn(fallback);
  return pretty.empty() ? std::string(kUnknown) : pretty;
}

void SetHostIdentityProperties(c3d::Session& session) {
  const std::string distro = RosDistro();
  session.SetProperty("c3d.ros.distro", distro);
  // The same value under the cross-SDK name. `c3d.ros.*` is this integration's own namespace and a
  // dashboard view spanning several SDKs cannot know it; `c3d.app.engine.version` is the field
  // that pairs with `c3d.app.engine` everywhere. ROS 2's version IS its distro, so this is the
  // engine's version rather than a second fact.
  session.SetProperty("c3d.app.engine.version", distro);
  session.SetProperty("c3d.device.os", HostOperatingSystem());
  session.SetProperty("c3d.ros.domain_id", RosDomainId());
  if (DistroHasDiscoveryRange()) {
    session.SetProperty("c3d.ros.discovery_range", RosDiscoveryRange());
  }
}

}  // namespace cognitive3d_ros
