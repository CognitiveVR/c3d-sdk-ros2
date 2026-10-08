#pragma once

#include <functional>
#include <string>

#include "c3d/environment.hpp"

namespace cognitive3d_ros {

// Where a run sends telemetry (the data host) and reads metadata (the api host), and the
// environment name the pair goes by: a table name, or c3d::kCustomEnvironment.
struct Destination {
  std::string environment;
  c3d::Hosts hosts;
};

// The destination, from the PROCESS ENVIRONMENT and nothing else: a params file, which is not a
// trusted input, never chooses where an organisation key is sent. The rules are
// c3d::ResolveEnvironment's, which the tools under tools/ share, so no half of the toolchain can
// resolve a host another half refuses. `lookup` returns a variable's value, or null when it is
// unset. `out` is untouched on a refusal.
bool ResolveDestination(const std::function<const char*(const char*)>& lookup, Destination* out,
                        std::string* error);

// The host a `gateway` value names, for comparing hosts however they are spelled: lowercased,
// with any scheme, userinfo, port, path and trailing dot removed, and an IPv6 literal unbracketed.
// `HTTP://user@Data.Example.com.:80/x` is `data.example.com`.
std::string GatewayHost(const std::string& gateway);

// Whether `gateway` names a Cognitive3D host in any spelling GatewayHost folds: `data_host` (the
// data host this process resolved), any environment's data host, or cognitive3d.com or one of its
// subdomains.
bool NamesACognitive3DHost(const std::string& gateway, const std::string& data_host);

// One line for a log or a report: which environment, the data host telemetry goes to -- with
// `gateway`, when it differs, as the override it is -- and the api host metadata is read from.
std::string DescribeDestination(const Destination& destination, const std::string& gateway);

}  // namespace cognitive3d_ros
