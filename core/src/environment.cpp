#include "c3d/environment.hpp"

#include <cstdlib>
#include <sstream>

namespace c3d {
namespace {

struct Entry {
  const char* name;
  const char* data;
  const char* api;
};

// The ONE table. Every host literal this repo sends to lives here; anything else naming a host is
// a copy that can drift out of step with its pair.
constexpr Entry kTable[] = {
    {"prod", "data.cognitive3d.com", "api.cognitive3d.com"},
};

bool IsAlnum(char c) {
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9');
}

// A host name or IPv4 literal with an optional :port, and nothing else. A scheme, a path, a
// query or `user@` would let the value move the request somewhere its host part does not say.
bool IsBareHost(const std::string& value) {
  const std::size_t colon = value.find(':');
  const std::string host = value.substr(0, colon);
  if (host.empty() || !IsAlnum(host.front()) || !IsAlnum(host.back())) return false;
  for (char c : host) {
    if (!IsAlnum(c) && c != '.' && c != '-') return false;
  }
  if (colon == std::string::npos) return true;
  const std::string port = value.substr(colon + 1);
  if (port.empty() || port.size() > 5) return false;
  for (char c : port) {
    if (c < '0' || c > '9') return false;
  }
  return std::stoi(port) <= 65535;
}

std::string ProblemWithHost(const char* variable, const char* value) {
  const std::string text = value;
  if (text.empty()) return std::string(variable) + " is set but empty";
  if (!IsBareHost(text)) {
    return std::string(variable) +
           " must be a bare host name with an optional :port -- no scheme, path or "
           "credentials -- got '" +
           text + "'";
  }
  return {};
}

}  // namespace

const char kDefaultEnvironment[] = "prod";
const char kEnvironmentVariable[] = "C3D_ENVIRONMENT";
const char kDataHostVariable[] = "C3D_DATA_HOST";
const char kApiHostVariable[] = "C3D_API_HOST";
const char kCustomEnvironment[] = "custom";

bool HostsForEnvironment(const std::string& name, Hosts* hosts) {
  for (const Entry& entry : kTable) {
    if (name == entry.name) {
      if (hosts != nullptr) *hosts = Hosts{entry.data, entry.api};
      return true;
    }
  }
  return false;
}

std::string EnvironmentOwningDataHost(const std::string& host) {
  for (const Entry& entry : kTable) {
    if (host == entry.data) return entry.name;
  }
  return {};
}

Hosts DefaultHosts() {
  Hosts hosts;
  HostsForEnvironment(kDefaultEnvironment, &hosts);
  return hosts;
}

std::string KnownEnvironments() {
  std::ostringstream names;
  bool first = true;
  for (const Entry& entry : kTable) {
    if (!first) names << ", ";
    names << entry.name;
    first = false;
  }
  return names.str();
}

bool ResolveEnvironment(const char* environment, const char* data_host, const char* api_host,
                        ResolvedEnvironment* out, std::string* error) {
  auto fail = [error](const std::string& message) {
    if (error != nullptr) *error = message;
    return false;
  };
  const std::string name = environment == nullptr ? kDefaultEnvironment : environment;
  Hosts hosts;
  if (!HostsForEnvironment(name, &hosts)) {
    return fail(std::string(kEnvironmentVariable) + " must be " + KnownEnvironments() +
                " or unset, got '" + name +
                "'. Matched exactly: no trimming, no case folding. Other hosts are reached with " +
                kDataHostVariable + " and " + kApiHostVariable + ", set together.");
  }
  if ((data_host == nullptr) != (api_host == nullptr)) {
    const char* set = data_host != nullptr ? kDataHostVariable : kApiHostVariable;
    const char* unset = data_host != nullptr ? kApiHostVariable : kDataHostVariable;
    return fail(std::string(set) + " is set but " + unset +
                " is not. The two hosts are one decision: set both, or neither.");
  }
  if (data_host != nullptr && environment != nullptr) {
    return fail(std::string(kEnvironmentVariable) + " and " + kDataHostVariable + " / " +
                kApiHostVariable + " are both set; name the hosts one way.");
  }
  ResolvedEnvironment resolved;
  resolved.name = name;
  resolved.hosts = hosts;
  if (data_host != nullptr) {
    for (const auto& [variable, value] : {std::make_pair(kDataHostVariable, data_host),
                                          std::make_pair(kApiHostVariable, api_host)}) {
      const std::string problem = ProblemWithHost(variable, value);
      if (!problem.empty()) return fail(problem);
    }
    resolved.name = kCustomEnvironment;
    resolved.hosts = Hosts{data_host, api_host};
    resolved.custom_hosts = true;
  }
  if (out != nullptr) *out = resolved;
  return true;
}

bool ResolveEnvironmentFromProcess(ResolvedEnvironment* out, std::string* error) {
  return ResolveEnvironment(std::getenv(kEnvironmentVariable), std::getenv(kDataHostVariable),
                            std::getenv(kApiHostVariable), out, error);
}

std::string DescribeEnvironment(const ResolvedEnvironment& environment,
                                const std::string& data_host_in_use) {
  std::string line = "environment: " + environment.name;
  if (environment.custom_hosts) {
    line += " (CUSTOM HOSTS from " + std::string(kDataHostVariable) + " and " + kApiHostVariable +
            ", not a Cognitive3D production environment)";
  }
  line += " -- telemetry to " + data_host_in_use;
  if (data_host_in_use != environment.hosts.data) line += " (OVERRIDDEN by the gateway parameter)";
  line += ", metadata to " + environment.hosts.api;
  return line;
}

}  // namespace c3d
