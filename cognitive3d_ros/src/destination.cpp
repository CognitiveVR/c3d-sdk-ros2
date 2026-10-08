#include "cognitive3d_ros/destination.hpp"

#include <algorithm>
#include <cctype>

namespace cognitive3d_ros {

bool ResolveDestination(const std::function<const char*(const char*)>& lookup, Destination* out,
                        std::string* error) {
  c3d::ResolvedEnvironment resolved;
  if (!c3d::ResolveEnvironment(lookup(c3d::kEnvironmentVariable), lookup(c3d::kDataHostVariable),
                               lookup(c3d::kApiHostVariable), &resolved, error)) {
    return false;
  }
  out->environment = resolved.name;
  out->hosts = resolved.hosts;
  return true;
}

std::string GatewayHost(const std::string& gateway) {
  std::string host = gateway;
  std::transform(host.begin(), host.end(), host.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  const auto first = host.find_first_not_of(" \t");
  host = first == std::string::npos ? std::string() : host.substr(first);
  host.erase(host.find_last_not_of(" \t") + 1);
  const auto scheme = host.find("://");
  if (scheme != std::string::npos) host = host.substr(scheme + 3);
  const auto path = host.find_first_of("/?#");
  if (path != std::string::npos) host.erase(path);
  const auto userinfo = host.rfind('@');
  if (userinfo != std::string::npos) host = host.substr(userinfo + 1);
  if (!host.empty() && host.front() == '[') {
    const auto close = host.find(']');
    host = host.substr(1, close == std::string::npos ? std::string::npos : close - 1);
  } else if (const auto colon = host.find(':');
             colon != std::string::npos && host.find(':', colon + 1) == std::string::npos) {
    host.erase(colon);
  }
  while (!host.empty() && host.back() == '.') host.pop_back();
  return host;
}

bool NamesACognitive3DHost(const std::string& gateway, const std::string& data_host) {
  const std::string host = GatewayHost(gateway);
  if (host.empty()) return false;
  if (host == GatewayHost(data_host)) return true;
  const std::string domain = "cognitive3d.com";
  if (host == domain) return true;
  if (host.size() > domain.size() &&
      host.compare(host.size() - domain.size() - 1, std::string::npos, "." + domain) == 0) {
    return true;
  }
  return !c3d::EnvironmentOwningDataHost(host).empty();
}

std::string DescribeDestination(const Destination& destination, const std::string& gateway) {
  const std::string name =
      destination.environment == c3d::kCustomEnvironment
          ? std::string("custom hosts (") + c3d::kDataHostVariable + " / " +
                c3d::kApiHostVariable + ")"
          : destination.environment;
  std::string line =
      name + ": telemetry to " + (gateway.empty() ? destination.hosts.data : gateway);
  if (!gateway.empty() && gateway != destination.hosts.data) {
    line += " (the gateway parameter overrides " + destination.hosts.data + ")";
  }
  return line + ", metadata from " + destination.hosts.api;
}

}  // namespace cognitive3d_ros
