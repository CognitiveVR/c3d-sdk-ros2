// Remote variables and A/B assignments, read from the platform once per session.
//
// The platform draws an arm per `identifier` deterministically (a hash of the string, nothing
// stored), returns only the RESOLVED value per variable -- never the arm's name -- and reserves
// nothing about how the value is recorded. The dashboard is what gives the property prefix
// below its meaning: any session property named `c3d.remote_variable.<name>` is sliced as an
// A/B dimension. Ints must reach the wire as JSON numbers or that slice cannot see them.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "c3d/config.hpp"
#include "c3d/transport.hpp"
#include "c3d/types.hpp"

namespace c3d {

class Session;

struct RemoteVariable {
  std::string name;     // remoteVariableName: the key the dashboard slices on
  PropertyValue value;  // typed as the wire says: int, boolean or string
};

struct RemoteVariableSet {
  std::vector<RemoteVariable> variables;  // wire order: abTests, then remoteConfigurations
  std::vector<std::string> skipped;       // items the parser dropped, each named with why
  bool fetched = false;                   // a 200 with cvr-request-time and a parseable body
  long status = 0;
  std::string error;                      // empty when fetched
  // {"<name>": <typed value>, ...} in wire order; "{}" when there are none. A duplicate name
  // resolves to its last entry, which is how the Unity SDK's property writer resolves it.
  std::string ToJson() const;
};

// {protocol}://{gateway}/v{network_version}/remotevariables?identifier=<sha256 hex of identifier>
//
// The platform draws an arm from a hash of the string it is sent, and that hash spreads
// sequentially named identifiers poorly: `robot-01` to `robot-10` all draw the same arm. The
// lowercase hex SHA-256 of the identifier is what goes on the wire, so near-identical names land
// in unrelated arms. The session still records the identifier itself as its participant.
std::string RemoteVariablesUrl(const Config& config, const std::string& identifier);

// Reads both arrays. `type` names the value field ("int" -> valueInt, "boolean" -> valueBoolean,
// "string" -> valueString); an item whose field is missing, null or mistyped is skipped and
// named in `out->skipped` without failing the set. Malformed JSON or a non-object fails it.
// `out` is required; `error` may be null.
bool ParseRemoteVariables(const std::string& body, RemoteVariableSet* out, std::string* error);

// Blocking, on the caller's thread, bounded by the reader's own timeout. Never throws.
RemoteVariableSet FetchRemoteVariables(IHttpGet& reader, const Config& config,
                                       const std::string& identifier);

// SetProperty("c3d.remote_variable." + name, value) for each variable, in order.
void ApplyRemoteVariables(Session& session, const RemoteVariableSet& set);

}  // namespace c3d
