// Where a run sends telemetry and reads metadata, decided from the process environment alone.

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "cognitive3d_ros/destination.hpp"

namespace {

using cognitive3d_ros::Destination;
using cognitive3d_ros::DescribeDestination;
using cognitive3d_ros::ResolveDestination;
using Env = std::map<std::string, std::string>;

bool Resolve(const Env& env, Destination* out, std::string* error) {
  return ResolveDestination(
      [&env](const char* name) -> const char* {
        const auto found = env.find(name);
        return found == env.end() ? nullptr : found->second.c_str();
      },
      out, error);
}

// The rules are core's, and core and the Python tools are tested against this same table; what
// this pins is that the node reads all three variables, each under its own name, and keeps both
// hosts of what core resolved.
TEST(Destination, EveryCaseInTheTableCoreAndTheToolsAreTestedAgainst) {
  std::ifstream in(C3D_ENVIRONMENT_CASES);
  ASSERT_TRUE(in) << "cannot open " << C3D_ENVIRONMENT_CASES;
  const nlohmann::json table = nlohmann::json::parse(in);
  ASSERT_GE(table["cases"].size(), 20u);
  for (const auto& c : table["cases"]) {
    Env env;
    for (const auto& [name, value] : c["env"].items()) env[name] = value.get<std::string>();
    SCOPED_TRACE(c["env"].dump());
    Destination destination{"sentinel", {"sentinel-data", "sentinel-api"}};
    std::string error;
    const bool accepted = Resolve(env, &destination, &error);
    ASSERT_EQ(accepted, c["accept"].get<bool>()) << error;
    if (accepted) {
      EXPECT_EQ(destination.environment, c["name"].get<std::string>());
      EXPECT_EQ(destination.hosts.data, c["data"].get<std::string>());
      EXPECT_EQ(destination.hosts.api, c["api"].get<std::string>());
    } else {
      EXPECT_EQ(destination.environment, "sentinel") << "a refusal must leave the output untouched";
      for (const auto& variable : c["mentions"]) {
        EXPECT_NE(error.find(variable.get<std::string>()), std::string::npos)
            << "the refusal must name " << variable << ": " << error;
      }
    }
  }
}

TEST(Destination, TheDescriptionSaysWhichEnvironmentAndWhereEachHalfGoes) {
  Destination production;
  std::string error;
  ASSERT_TRUE(Resolve({}, &production, &error));
  const std::string plain = DescribeDestination(production, production.hosts.data);
  EXPECT_EQ(plain.rfind(std::string(c3d::kDefaultEnvironment) + ": ", 0), 0u) << plain;
  EXPECT_NE(plain.find(production.hosts.api), std::string::npos) << plain;
  EXPECT_EQ(plain.find("overrides"), std::string::npos) << plain;

  const std::string stub = DescribeDestination(production, "127.0.0.1:8787");
  EXPECT_NE(stub.find("127.0.0.1:8787"), std::string::npos) << stub;
  EXPECT_NE(stub.find("overrides " + production.hosts.data), std::string::npos) << stub;

  Destination custom;
  ASSERT_TRUE(Resolve(
      {{"C3D_DATA_HOST", "data.example.test"}, {"C3D_API_HOST", "api.example.test"}}, &custom,
      &error));
  const std::string named = DescribeDestination(custom, "");
  EXPECT_EQ(named.rfind("custom hosts (C3D_DATA_HOST / C3D_API_HOST): ", 0), 0u) << named;
  EXPECT_NE(named.find("data.example.test"), std::string::npos) << named;
  EXPECT_NE(named.find("api.example.test"), std::string::npos) << named;
}

TEST(GatewayHost, FoldsEverySpellingOfOneHost) {
  using cognitive3d_ros::GatewayHost;
  for (const std::string spelling :
       {"data.cognitive3d.com", "data.cognitive3d.com:80", "DATA.COGNITIVE3D.COM",
        "data.cognitive3d.com.", "http://data.cognitive3d.com", "HTTPS://Data.Cognitive3D.com.:443",
        "user:secret@data.cognitive3d.com:8080/v0/gaze", " data.cognitive3d.com "}) {
    EXPECT_EQ(GatewayHost(spelling), "data.cognitive3d.com") << "'" << spelling << "'";
  }
  EXPECT_EQ(GatewayHost("127.0.0.1:8787"), "127.0.0.1");
  EXPECT_EQ(GatewayHost("[::1]:8787"), "::1");
  EXPECT_EQ(GatewayHost("::1"), "::1");
  EXPECT_EQ(GatewayHost(""), "");
}

// `protocol: http` is refused beside any of these, so the key cannot reach a Cognitive3D host in
// cleartext through a spelling the comparison missed. A stub on loopback is what http is for.
TEST(GatewayHost, ACognitive3DHostIsRecognisedHoweverItIsSpelled) {
  using cognitive3d_ros::NamesACognitive3DHost;
  const std::string data = c3d::DefaultHosts().data;
  for (const std::string& gateway : std::vector<std::string>{
           data, "data.cognitive3d.com:80", "DATA.COGNITIVE3D.COM", "data.cognitive3d.com.",
           "http://data.cognitive3d.com", "api.cognitive3d.com", "cognitive3d.com",
           "a.b.cognitive3d.com:9000"}) {
    EXPECT_TRUE(NamesACognitive3DHost(gateway, data)) << "'" << gateway << "'";
  }
  // A custom data host, in another spelling.
  EXPECT_TRUE(NamesACognitive3DHost("Staging.Example.Test:443", "staging.example.test"));
  for (const std::string gateway :
       {"127.0.0.1:8787", "localhost:1", "[::1]:8080", "stub.local", "notcognitive3d.com",
        "cognitive3d.com.example.test", ""}) {
    EXPECT_FALSE(NamesACognitive3DHost(gateway, data)) << "'" << gateway << "'";
  }
}

}  // namespace
