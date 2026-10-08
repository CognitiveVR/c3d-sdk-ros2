#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "c3d/config.hpp"
#include "c3d/environment.hpp"

namespace {

using c3d::Hosts;
using c3d::HostsForEnvironment;
using c3d::ResolvedEnvironment;
using c3d::ResolveEnvironment;

// The test that matters is the ACCEPT/REJECT DECISION, not the host strings: two halves of a
// toolchain can agree on where a name points and still disagree about whether a trailing space,
// an empty string or a capitalised name counts as that name at all.
TEST(Environment, TheAcceptRejectDecisionIsPinnedOverTwelveValues) {
  const std::vector<std::pair<const char*, bool>> cases = {
      {"prod", true},
      {"", false},
      {" ", false},
      {"prod ", false},
      {" prod", false},
      {"PROD", false},
      {"dev", false},
      {"Dev", false},
      {"production", false},
      {"development", false},
      {"staging", false},
      {"custom", false},
  };
  for (const auto& [value, accepted] : cases) {
    Hosts hosts;
    EXPECT_EQ(HostsForEnvironment(value, &hosts), accepted)
        << "matched exactly: no trimming, no case folding. value: '" << value << "'";
  }
}

// An environment is a PAIR. A table that resolved one host and left the other would reintroduce
// the split the pair exists to prevent, so both must be present and neither may be derived from
// the other.
TEST(Environment, ProductionResolvesBothHostsAndTheyDiffer) {
  Hosts hosts;
  ASSERT_TRUE(HostsForEnvironment("prod", &hosts));
  EXPECT_FALSE(hosts.data.empty());
  EXPECT_FALSE(hosts.api.empty());
  EXPECT_NE(hosts.data, hosts.api);
}

// A rejected name must not leave a half-written pair behind for a caller that ignores the return.
TEST(Environment, ARejectedNameLeavesTheOutputUntouched) {
  Hosts hosts{"sentinel-data", "sentinel-api"};
  EXPECT_FALSE(HostsForEnvironment("staging", &hosts));
  EXPECT_EQ(hosts.data, "sentinel-data");
  EXPECT_EQ(hosts.api, "sentinel-api");

  EXPECT_FALSE(HostsForEnvironment("staging", nullptr)) << "a null sink must not crash";
}

// Absence is production. This is the one place the SDK must not be fatal on a missing value:
// every client ships prod and must never need the variable set to reach it.
TEST(Environment, TheDefaultIsProductionAndAFreshConfigAlreadyCarriesIt) {
  Hosts from_default;
  ASSERT_TRUE(HostsForEnvironment(c3d::kDefaultEnvironment, &from_default));
  EXPECT_EQ(c3d::DefaultHosts().data, from_default.data);
  EXPECT_EQ(c3d::DefaultHosts().api, from_default.api);

  const c3d::Config fresh;
  EXPECT_EQ(fresh.environment, std::string(c3d::kDefaultEnvironment));
  EXPECT_EQ(fresh.gateway, from_default.data);
}

// Tells a legitimate capture stub from a gateway naming an environment's host.
TEST(Environment, EveryDataHostIsAttributableToTheEnvironmentThatOwnsIt) {
  Hosts prod;
  ASSERT_TRUE(HostsForEnvironment("prod", &prod));
  EXPECT_EQ(c3d::EnvironmentOwningDataHost(prod.data), "prod");

  // An api host is not a data host, and neither is a capture stub.
  for (const std::string& not_a_data_host :
       {prod.api, std::string("127.0.0.1:8787"), std::string("localhost"), std::string(""),
        std::string("data.cognitive3d.com.evil.example")}) {
    EXPECT_TRUE(c3d::EnvironmentOwningDataHost(not_a_data_host).empty()) << not_a_data_host;
  }
}

TEST(Environment, TheErrorMessageNamesEveryAcceptedValue) {
  EXPECT_EQ(c3d::KnownEnvironments(), "prod");
}

// ---------------------------------------------------------------------------
// ResolveEnvironment: the shared case table
// ---------------------------------------------------------------------------

const char* Variable(const nlohmann::json& env, const char* name) {
  const auto it = env.find(name);
  if (it == env.end()) return nullptr;
  return it->get_ref<const std::string&>().c_str();
}

TEST(ResolveEnvironment, EveryCaseInTheTableTheToolsAreTestedAgainst) {
  std::ifstream in(C3D_ENVIRONMENT_CASES);
  ASSERT_TRUE(in) << "cannot open " << C3D_ENVIRONMENT_CASES;
  const nlohmann::json table = nlohmann::json::parse(in);
  ASSERT_GE(table["cases"].size(), 20u);
  for (const auto& c : table["cases"]) {
    const auto& env = c["env"];
    SCOPED_TRACE(env.dump());
    ResolvedEnvironment resolved;
    resolved.name = "sentinel";
    std::string error;
    const bool accepted = ResolveEnvironment(
        Variable(env, c3d::kEnvironmentVariable), Variable(env, c3d::kDataHostVariable),
        Variable(env, c3d::kApiHostVariable), &resolved, &error);
    ASSERT_EQ(accepted, c["accept"].get<bool>()) << error;
    if (accepted) {
      EXPECT_TRUE(error.empty()) << error;
      EXPECT_EQ(resolved.name, c["name"].get<std::string>());
      EXPECT_EQ(resolved.hosts.data, c["data"].get<std::string>());
      EXPECT_EQ(resolved.hosts.api, c["api"].get<std::string>());
      EXPECT_EQ(resolved.custom_hosts, c["custom"].get<bool>());
    } else {
      EXPECT_EQ(resolved.name, "sentinel") << "a refusal must leave the output untouched";
      for (const auto& variable : c["mentions"]) {
        EXPECT_NE(error.find(variable.get<std::string>()), std::string::npos)
            << "the refusal must name " << variable << ": " << error;
      }
      if (c.contains("says")) {
        EXPECT_NE(error.find(c["says"].get<std::string>()), std::string::npos) << error;
      }
    }
  }
}

TEST(ResolveEnvironment, ReadsTheProcessEnvironment) {
  ::setenv(c3d::kDataHostVariable, "data.example.test", 1);
  ::unsetenv(c3d::kApiHostVariable);
  ResolvedEnvironment resolved;
  std::string error;
  EXPECT_FALSE(c3d::ResolveEnvironmentFromProcess(&resolved, &error));
  EXPECT_NE(error.find(c3d::kApiHostVariable), std::string::npos) << error;

  ::setenv(c3d::kApiHostVariable, "api.example.test", 1);
  ASSERT_TRUE(c3d::ResolveEnvironmentFromProcess(&resolved, &error)) << error;
  EXPECT_EQ(resolved.hosts.data, "data.example.test");
  EXPECT_EQ(resolved.hosts.api, "api.example.test");
  ::unsetenv(c3d::kDataHostVariable);
  ::unsetenv(c3d::kApiHostVariable);
}

// The resolved-environment log line is how an archived log answers "where did this run send its
// data": custom hosts must be unmistakable there, and so must a gateway override.
TEST(ResolveEnvironment, TheLogLineSaysWhenCustomHostsOrAGatewayOverrideAreInForce) {
  ResolvedEnvironment prod;
  ASSERT_TRUE(ResolveEnvironment(nullptr, nullptr, nullptr, &prod, nullptr));
  const std::string plain = c3d::DescribeEnvironment(prod, prod.hosts.data);
  EXPECT_EQ(plain,
            "environment: prod -- telemetry to data.cognitive3d.com, metadata to "
            "api.cognitive3d.com");

  const std::string overridden = c3d::DescribeEnvironment(prod, "127.0.0.1:8787");
  EXPECT_NE(overridden.find("127.0.0.1:8787 (OVERRIDDEN by the gateway parameter)"),
            std::string::npos)
      << overridden;

  ResolvedEnvironment custom;
  ASSERT_TRUE(
      ResolveEnvironment(nullptr, "data.example.test", "api.example.test", &custom, nullptr));
  const std::string line = c3d::DescribeEnvironment(custom, custom.hosts.data);
  EXPECT_NE(line.find("CUSTOM HOSTS"), std::string::npos) << line;
  EXPECT_NE(line.find("C3D_DATA_HOST"), std::string::npos) << line;
  EXPECT_NE(line.find("telemetry to data.example.test"), std::string::npos) << line;
  EXPECT_NE(line.find("metadata to api.example.test"), std::string::npos) << line;
  EXPECT_EQ(line.find("OVERRIDDEN"), std::string::npos) << line;
}

}  // namespace
