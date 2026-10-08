#pragma once

#include <string>
#include <vector>

// Which Cognitive3D environment a run talks to.
//
// An environment is a PAIR of hosts, not one, and they are a single decision. The data host
// receives telemetry, scenes and dynamic objects under an application or developer key; the api
// host answers metadata queries under an organisation key. A configuration that moves one and not
// the other sends a session to one environment and reads it back from the other, and a production
// key makes the mismatched half succeed, so nothing reports it.
namespace c3d {

struct Hosts {
  std::string data;  // telemetry, scenes, dynamic objects
  std::string api;   // metadata, read and act
};

// The environment used when nothing selects one. Production: every client ships prod, so the
// variable must not be required to reach it.
extern const char kDefaultEnvironment[];

// The environment variable that selects one.
extern const char kEnvironmentVariable[];

// The override pair: two process-environment variables that, set together, replace the table's
// hosts. Read from the process environment only and never from a ROS parameter, because a params
// file is not a trusted input and must not choose where an organisation key is sent.
extern const char kDataHostVariable[];
extern const char kApiHostVariable[];

// The name a run resolves to while the override pair is in force. Not a value of
// kEnvironmentVariable: setting it there is refused like any other unknown name.
extern const char kCustomEnvironment[];

// Resolves an environment NAME to its host pair. Returns false for anything not in the table.
//
// The match is EXACT -- no trimming, no case folding -- and the empty string is NOT a member, so a
// caller that maps "the variable is unset" to a default must do so before asking. Either leniency
// lets a trailing space or a blank value resolve on one side of the toolchain while the other
// refuses it, and the two halves of a toolchain must never disagree about which environment a run
// is in.
bool HostsForEnvironment(const std::string& name, Hosts* hosts);

// The environment whose DATA host this is, or empty when no environment claims it.
//
// Lets a caller tell a loopback capture stub -- the legitimate reason to override the gateway --
// from a gateway naming a DIFFERENT environment's host, which is never a capture and is exactly
// the cross-environment split this table exists to prevent.
std::string EnvironmentOwningDataHost(const std::string& host);

// The prod pair, for defaults that need a host before any configuration is read.
Hosts DefaultHosts();

// Every name the table knows, comma separated, for error messages.
std::string KnownEnvironments();

struct ResolvedEnvironment {
  std::string name;  // a table name, or kCustomEnvironment
  Hosts hosts;
  bool custom_hosts = false;
};

// Resolves the three variables. Each argument is that variable's value, or null when it is unset.
//
//   * kEnvironmentVariable unset or exactly "prod" is production; any other value, the empty
//     string included, is refused.
//   * kDataHostVariable and kApiHostVariable are set together or not at all, and never beside
//     kEnvironmentVariable: the hosts are named one way. Set, each must be a bare host name with
//     an optional :port, and together they replace the production pair.
//
// Returns false with `error` naming the variable at fault. tools/c3d_api/env.py implements the
// same rules, and core/test/environment_cases.json is the table both are tested against.
bool ResolveEnvironment(const char* environment, const char* data_host, const char* api_host,
                        ResolvedEnvironment* out, std::string* error);

// ResolveEnvironment over the process environment.
bool ResolveEnvironmentFromProcess(ResolvedEnvironment* out, std::string* error);

// One line naming the environment and both hosts, for a log. `data_host_in_use` is the host
// telemetry actually goes to, which a `gateway` override can make differ from the resolved one;
// the line says which of the two moved it.
std::string DescribeEnvironment(const ResolvedEnvironment& environment,
                                const std::string& data_host_in_use);

}  // namespace c3d
