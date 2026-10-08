// Remote variables: the URL, the parser, the fetch and the property write. See
// core/include/c3d/remote_variables.hpp; every test here is one rule named as a claim.

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "c3d/remote_variables.hpp"
#include "c3d/session.hpp"
#include "fake_transport.hpp"

using namespace c3d;

// ---------------------------------------------------------------------------
// FakeTransport::Get
// ---------------------------------------------------------------------------

TEST(FakeTransportGet, ReplaysScriptedResponsesInOrderAndRecordsTheRequest) {
  FakeTransport transport;
  HttpResponse first;
  first.status = 401;
  first.saw_request_time_header = true;
  transport.ScriptGet(first);

  const std::vector<std::string> headers = {"Authorization: APIKEY:DATA k"};
  const HttpResponse got = transport.Get("https://g/v0/remotevariables?identifier=x", headers);
  EXPECT_EQ(got.status, 401);

  // Unscripted: a 200 from the gateway, with an empty object body.
  const HttpResponse next = transport.Get("https://g/v0/remotevariables?identifier=y", headers);
  EXPECT_EQ(next.status, 200);
  EXPECT_TRUE(next.saw_request_time_header);
  EXPECT_EQ(next.body, "{}");

  ASSERT_EQ(transport.gets().size(), 2u);
  EXPECT_EQ(transport.gets()[0].url, "https://g/v0/remotevariables?identifier=x");
  EXPECT_EQ(transport.gets()[0].headers, headers);
  EXPECT_EQ(transport.count(), 0u) << "reads must not be counted as POSTs";
}

// ---------------------------------------------------------------------------
// RemoteVariablesUrl
// ---------------------------------------------------------------------------

namespace {
Config GatewayConfig(const std::string& gateway) {
  Config cfg;
  cfg.protocol = "https";
  cfg.gateway = gateway;
  cfg.network_version = "0";
  cfg.application_key = "APPKEY";
  return cfg;
}
}  // namespace

TEST(RemoteVariablesUrl, UsesTheDataGatewayAndTheNetworkVersion) {
  EXPECT_EQ(RemoteVariablesUrl(GatewayConfig("data.example.test"), "abc"),
            "https://data.example.test/v0/remotevariables?identifier="
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}

TEST(RemoteVariablesUrl, HonoursProtocolAndNetworkVersion) {
  Config cfg = GatewayConfig("127.0.0.1:1");
  cfg.protocol = "http";
  cfg.network_version = "7";
  EXPECT_EQ(RemoteVariablesUrl(cfg, "abc"),
            "http://127.0.0.1:1/v7/remotevariables?identifier="
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}

// The digests below were computed with Python's hashlib, not with the SDK's own SHA-256.
TEST(RemoteVariablesUrl, SendsTheLowercaseHexSha256OfTheIdentifierAndNeverTheIdentifier) {
  const struct {
    const char* identifier;
    const char* digest;
  } cases[] = {
      {"3f2a9c1e-7b4d-4e1a-9c2b-0d5e6f7a8b9c",
       "8e03ab39fc690fc0b2526513a43e6060326682ee7f70e135e3781bfc76295acc"},
      // Bytes a query string would otherwise need escaped, and bytes above ASCII: the digest is
      // taken over the identifier's exact bytes, whatever they are.
      {"a/b c+d#e&f=g", "1167ec3b69bf7a031447329b3488f14b2171be484284369779c29a7a779522e3"},
      {"caf\xC3\xA9", "850f7dc43910ff890f8879c0ed26fe697c93a067ad93a7d50f466a7028a9bf4e"},
  };
  for (const auto& c : cases) {
    const std::string url = RemoteVariablesUrl(GatewayConfig("g"), c.identifier);
    EXPECT_EQ(url, std::string("https://g/v0/remotevariables?identifier=") + c.digest);
    EXPECT_EQ(url.find(c.identifier), std::string::npos) << "the raw identifier reached the URL";
  }
}

TEST(RemoteVariablesUrl, SequentiallyNamedIdentifiersAreSentAsUnrelatedStrings) {
  // The failure this exists for: the platform's draw over `robot-01` and `robot-02` lands both
  // in one arm. Their digests share no prefix.
  EXPECT_EQ(RemoteVariablesUrl(GatewayConfig("g"), "robot-01"),
            "https://g/v0/remotevariables?identifier="
            "1d2186ba0fd0a394cc2e45c9ef3e103bbc4dea5c3a9a1248ac15a48bb147ef0c");
  EXPECT_EQ(RemoteVariablesUrl(GatewayConfig("g"), "robot-02"),
            "https://g/v0/remotevariables?identifier="
            "6f619a82930ff0161d413f3098328ded27e9a6103383e8978a010bf9ac421bea");
}

// ---------------------------------------------------------------------------
// ParseRemoteVariables / ToJson
// ---------------------------------------------------------------------------

namespace {
// The shape the platform returns: exactly one value* per item, chosen by `type`,
// the others null. Both arrays share it.
const char* kBody = R"({
  "abTests": [
    {"name": "maxvel", "description": "", "remoteVariableName": "nav_speed_centimeters_per_second",
     "type": "int", "valueInt": 30, "valueString": null, "valueBoolean": null}
  ],
  "remoteConfigurations": [
    {"name": "lights", "description": "", "remoteVariableName": "headlights_on",
     "type": "boolean", "valueBoolean": true},
    {"name": "greeting", "remoteVariableName": "greeting", "type": "string", "valueString": "hello"}
  ]
})";
}  // namespace

TEST(ParseRemoteVariables, ReadsBothArraysInWireOrderWithTheirTypes) {
  RemoteVariableSet set;
  std::string error;
  ASSERT_TRUE(ParseRemoteVariables(kBody, &set, &error)) << error;
  ASSERT_EQ(set.variables.size(), 3u);
  EXPECT_TRUE(set.skipped.empty());

  EXPECT_EQ(set.variables[0].name, "nav_speed_centimeters_per_second");
  ASSERT_EQ(set.variables[0].value.kind(), PropertyValue::Kind::kInt);
  EXPECT_EQ(set.variables[0].value.as_int(), 30);

  EXPECT_EQ(set.variables[1].name, "headlights_on");
  ASSERT_EQ(set.variables[1].value.kind(), PropertyValue::Kind::kBool);
  EXPECT_TRUE(set.variables[1].value.as_bool());

  EXPECT_EQ(set.variables[2].name, "greeting");
  ASSERT_EQ(set.variables[2].value.kind(), PropertyValue::Kind::kString);
  EXPECT_EQ(set.variables[2].value.as_string(), "hello");
}

TEST(ParseRemoteVariables, EmptyArraysAreASuccessWithNothingInThem) {
  RemoteVariableSet set;
  std::string error;
  ASSERT_TRUE(ParseRemoteVariables(R"({"abTests":[],"remoteConfigurations":[]})", &set, &error));
  EXPECT_TRUE(set.variables.empty());
  EXPECT_EQ(set.ToJson(), "{}");
}

TEST(ParseRemoteVariables, MissingArraysAreTolerated) {
  RemoteVariableSet set;
  std::string error;
  ASSERT_TRUE(ParseRemoteVariables("{}", &set, &error));
  EXPECT_TRUE(set.variables.empty());
}

TEST(ParseRemoteVariables, AnItemWhoseTypedFieldIsMissingIsSkippedAndNamed) {
  RemoteVariableSet set;
  std::string error;
  const char* body = R"({"abTests":[{"remoteVariableName":"speed","type":"int"}]})";
  ASSERT_TRUE(ParseRemoteVariables(body, &set, &error));
  EXPECT_TRUE(set.variables.empty());
  ASSERT_EQ(set.skipped.size(), 1u);
  EXPECT_NE(set.skipped[0].find("speed"), std::string::npos) << set.skipped[0];
  EXPECT_NE(set.skipped[0].find("valueInt"), std::string::npos) << set.skipped[0];
}

TEST(ParseRemoteVariables, AMistypedValueIsSkippedNotCoerced) {
  // "30" as a string under type int would land in the textual index if written through.
  RemoteVariableSet set;
  std::string error;
  const char* body = R"({"abTests":[{"remoteVariableName":"speed","type":"int","valueInt":"30"}]})";
  ASSERT_TRUE(ParseRemoteVariables(body, &set, &error));
  EXPECT_TRUE(set.variables.empty());
  ASSERT_EQ(set.skipped.size(), 1u);
}

TEST(ParseRemoteVariables, AnUnknownTypeIsSkipped) {
  RemoteVariableSet set;
  std::string error;
  const char* body = R"({"abTests":[{"remoteVariableName":"x","type":"float","valueInt":1}]})";
  ASSERT_TRUE(ParseRemoteVariables(body, &set, &error));
  EXPECT_TRUE(set.variables.empty());
  ASSERT_EQ(set.skipped.size(), 1u);
  EXPECT_NE(set.skipped[0].find("float"), std::string::npos) << set.skipped[0];
}

TEST(ParseRemoteVariables, ANullTypeIsSkippedNotThrown) {
  RemoteVariableSet set;
  std::string error;
  const char* body = R"({"abTests":[{"remoteVariableName":"x","type":null,"valueInt":1}]})";
  ASSERT_TRUE(ParseRemoteVariables(body, &set, &error));
  EXPECT_TRUE(set.variables.empty());
  ASSERT_EQ(set.skipped.size(), 1u);
  EXPECT_NE(set.skipped[0].find("x"), std::string::npos) << set.skipped[0];
}

TEST(ParseRemoteVariables, ANonStringTypeIsSkippedNotThrown) {
  RemoteVariableSet set;
  std::string error;
  const char* body = R"({"abTests":[{"remoteVariableName":"x","type":5,"valueInt":1}]})";
  ASSERT_TRUE(ParseRemoteVariables(body, &set, &error));
  EXPECT_TRUE(set.variables.empty());
  ASSERT_EQ(set.skipped.size(), 1u);
  EXPECT_NE(set.skipped[0].find("x"), std::string::npos) << set.skipped[0];
}

TEST(ParseRemoteVariables, AnItemWithoutANameIsSkipped) {
  RemoteVariableSet set;
  std::string error;
  ASSERT_TRUE(ParseRemoteVariables(R"({"abTests":[{"type":"int","valueInt":1}]})", &set, &error));
  EXPECT_TRUE(set.variables.empty());
  EXPECT_EQ(set.skipped.size(), 1u);
}

TEST(ParseRemoteVariables, APresentButNonArrayTopLevelKeyFailsTheSet) {
  // Read as "no items of this kind", a whole array of arms served under a wrong shape becomes a
  // session in no arm, and nothing anywhere says so. The set fails instead.
  RemoteVariableSet set;
  std::string error;
  EXPECT_FALSE(ParseRemoteVariables(R"({"abTests":{"name":"x"}})", &set, &error));
  EXPECT_NE(error.find("abTests"), std::string::npos) << error;
}

TEST(ParseRemoteVariables, ANullTopLevelKeyIsTolerated) {
  // Null is how a JSON serializer spells an absent collection, and absent is already tolerated.
  RemoteVariableSet set;
  std::string error;
  ASSERT_TRUE(ParseRemoteVariables(R"({"abTests":null})", &set, &error)) << error;
  EXPECT_TRUE(set.variables.empty());
}

TEST(ParseRemoteVariables, MalformedJsonFailsTheSet) {
  RemoteVariableSet set;
  std::string error;
  EXPECT_FALSE(ParseRemoteVariables("not json", &set, &error));
  EXPECT_FALSE(error.empty());
}

TEST(ParseRemoteVariables, ANonObjectFailsTheSet) {
  RemoteVariableSet set;
  std::string error;
  EXPECT_FALSE(ParseRemoteVariables("[]", &set, &error));
  EXPECT_NE(error.find("object"), std::string::npos) << error;
}

TEST(RemoteVariableSetToJson, KeepsWireTypesAndOrder) {
  RemoteVariableSet set;
  std::string error;
  ASSERT_TRUE(ParseRemoteVariables(kBody, &set, &error));
  EXPECT_EQ(set.ToJson(),
            R"({"nav_speed_centimeters_per_second":30,"headlights_on":true,"greeting":"hello"})");
}

TEST(RemoteVariableSetToJson, ADuplicateNameResolvesToTheLastEntry) {
  RemoteVariableSet set;
  std::string error;
  const char* body = R"({"abTests":[
    {"remoteVariableName":"speed","type":"int","valueInt":15},
    {"remoteVariableName":"speed","type":"int","valueInt":30}]})";
  ASSERT_TRUE(ParseRemoteVariables(body, &set, &error));
  EXPECT_EQ(set.variables.size(), 2u) << "the set keeps what the wire said";
  EXPECT_EQ(set.ToJson(), R"({"speed":30})");
}

// ---------------------------------------------------------------------------
// FetchRemoteVariables
// ---------------------------------------------------------------------------

namespace {
HttpResponse GatewayOk(const std::string& body) {
  HttpResponse r;
  r.status = 200;
  r.saw_request_time_header = true;
  r.body = body;
  return r;
}
}  // namespace

TEST(FetchRemoteVariables, SendsTheDataKeyAndTheOverrideHeaderToTheIdentifierUrl) {
  FakeTransport transport;
  transport.ScriptGet(GatewayOk(kBody));
  const auto set = FetchRemoteVariables(transport, GatewayConfig("data.example.test"), "uuid-1");

  const auto gets = transport.gets();
  ASSERT_EQ(gets.size(), 1u);
  const auto& request = gets[0];
  EXPECT_EQ(request.url,
            "https://data.example.test/v0/remotevariables?identifier="
            "1b95bc50c157d62c759f67e139e85c4484fd00ea9e4df270a3e89413532c1fa4")
      << "the fetch must carry the digest of the identifier, not the identifier";
  EXPECT_NE(std::find(request.headers.begin(), request.headers.end(),
                      "Authorization: APIKEY:DATA APPKEY"),
            request.headers.end());
  EXPECT_NE(std::find(request.headers.begin(), request.headers.end(),
                      "X-HTTP-Method-Override: GET"),
            request.headers.end());
  EXPECT_TRUE(set.fetched);
  EXPECT_EQ(set.status, 200);
  EXPECT_EQ(set.variables.size(), 3u);
}

TEST(FetchRemoteVariables, A200WithoutTheGatewayHeaderIsNotAnAnswer) {
  // A captive portal answers 200 to everything. Its body is a login page.
  FakeTransport transport;
  HttpResponse portal;
  portal.status = 200;
  portal.saw_request_time_header = false;
  portal.body = "<html>";
  transport.ScriptGet(portal);
  const auto set = FetchRemoteVariables(transport, GatewayConfig("g"), "id");
  EXPECT_FALSE(set.fetched);
  EXPECT_EQ(set.status, 200);
  EXPECT_NE(set.error.find("cvr-request-time"), std::string::npos) << set.error;
  EXPECT_TRUE(set.variables.empty());
}

TEST(FetchRemoteVariables, A401CarriesItsStatus) {
  FakeTransport transport;
  HttpResponse denied;
  denied.status = 401;
  denied.saw_request_time_header = true;
  transport.ScriptGet(denied);
  const auto set = FetchRemoteVariables(transport, GatewayConfig("g"), "id");
  EXPECT_FALSE(set.fetched);
  EXPECT_EQ(set.status, 401);
  EXPECT_NE(set.error.find("401"), std::string::npos) << set.error;
}

TEST(FetchRemoteVariables, NoResponseCarriesTheTransportError) {
  FakeTransport transport;
  transport.SetOffline(true);
  const auto set = FetchRemoteVariables(transport, GatewayConfig("g"), "id");
  EXPECT_FALSE(set.fetched);
  EXPECT_EQ(set.status, 0);
  EXPECT_EQ(set.error, "simulated network down");
}

TEST(FetchRemoteVariables, AMalformedBodyIsNotAnAnswerEither) {
  FakeTransport transport;
  transport.ScriptGet(GatewayOk("{not json"));
  const auto set = FetchRemoteVariables(transport, GatewayConfig("g"), "id");
  EXPECT_FALSE(set.fetched);
  EXPECT_EQ(set.status, 200);
  EXPECT_FALSE(set.error.empty());
  EXPECT_TRUE(set.variables.empty());
}
