// ParsePropertyScalar/ParsePropertiesJson: the coercion from configuration
// strings and caller-supplied JSON into typed c3d::PropertyValue. See
// core/include/c3d/properties.hpp for the exact rules; every test here is one
// of those rules named as a claim.

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

#include "c3d/properties.hpp"

using namespace c3d;

// ---------------------------------------------------------------------------
// ParsePropertyScalar
// ---------------------------------------------------------------------------

TEST(ParsePropertyScalar, LowercaseTrueBecomesBool) {
  const auto v = ParsePropertyScalar("true");
  ASSERT_EQ(v.kind(), PropertyValue::Kind::kBool);
  EXPECT_TRUE(v.as_bool());
}

TEST(ParsePropertyScalar, LowercaseFalseBecomesBool) {
  const auto v = ParsePropertyScalar("false");
  ASSERT_EQ(v.kind(), PropertyValue::Kind::kBool);
  EXPECT_FALSE(v.as_bool());
}

TEST(ParsePropertyScalar, CapitalizedBooleanStaysString) {
  // The rule is lowercase only. "True" is not the sentinel.
  const auto v = ParsePropertyScalar("True");
  ASSERT_EQ(v.kind(), PropertyValue::Kind::kString);
  EXPECT_EQ(v.as_string(), "True");
}

TEST(ParsePropertyScalar, PlainIntegerBecomesInt) {
  const auto v = ParsePropertyScalar("7");
  ASSERT_EQ(v.kind(), PropertyValue::Kind::kInt);
  EXPECT_EQ(v.as_int(), 7);
}

TEST(ParsePropertyScalar, NegativeIntegerBecomesInt) {
  const auto v = ParsePropertyScalar("-7");
  ASSERT_EQ(v.kind(), PropertyValue::Kind::kInt);
  EXPECT_EQ(v.as_int(), -7);
}

TEST(ParsePropertyScalar, IntegerWithTrailingDotZeroBecomesDouble) {
  // "7.0" is not "entirely an integer" -- the dot makes it a double, not int.
  const auto v = ParsePropertyScalar("7.0");
  ASSERT_EQ(v.kind(), PropertyValue::Kind::kDouble);
  EXPECT_DOUBLE_EQ(v.as_double(), 7.0);
}

TEST(ParsePropertyScalar, DecimalBecomesDouble) {
  const auto v = ParsePropertyScalar("1.5");
  ASSERT_EQ(v.kind(), PropertyValue::Kind::kDouble);
  EXPECT_DOUBLE_EQ(v.as_double(), 1.5);
}

TEST(ParsePropertyScalar, ScientificNotationBecomesDouble) {
  const auto v = ParsePropertyScalar("-2e3");
  ASSERT_EQ(v.kind(), PropertyValue::Kind::kDouble);
  EXPECT_DOUBLE_EQ(v.as_double(), -2000.0);
}

TEST(ParsePropertyScalar, LeadingZeroStaysString) {
  // c3d.deviceid is commonly a zero-padded serial; "002" must not become 2.
  const auto v = ParsePropertyScalar("002");
  ASSERT_EQ(v.kind(), PropertyValue::Kind::kString);
  EXPECT_EQ(v.as_string(), "002");
}

TEST(ParsePropertyScalar, SingleZeroIsStillAnInteger) {
  const auto v = ParsePropertyScalar("0");
  ASSERT_EQ(v.kind(), PropertyValue::Kind::kInt);
  EXPECT_EQ(v.as_int(), 0);
}

TEST(ParsePropertyScalar, SurroundingWhitespaceStaysString) {
  const auto v = ParsePropertyScalar(" 7 ");
  ASSERT_EQ(v.kind(), PropertyValue::Kind::kString);
  EXPECT_EQ(v.as_string(), " 7 ");
}

TEST(ParsePropertyScalar, TrailingGarbageAfterNumberStaysString) {
  const auto v = ParsePropertyScalar("1.5x");
  ASSERT_EQ(v.kind(), PropertyValue::Kind::kString);
  EXPECT_EQ(v.as_string(), "1.5x");
}

TEST(ParsePropertyScalar, EmbeddedSpaceStaysString) {
  const auto v = ParsePropertyScalar("1 5");
  ASSERT_EQ(v.kind(), PropertyValue::Kind::kString);
  EXPECT_EQ(v.as_string(), "1 5");
}

TEST(ParsePropertyScalar, EmptyStringStaysString) {
  const auto v = ParsePropertyScalar("");
  ASSERT_EQ(v.kind(), PropertyValue::Kind::kString);
  EXPECT_EQ(v.as_string(), "");
}

TEST(ParsePropertyScalar, OrdinaryTextStaysString) {
  const auto v = ParsePropertyScalar("warehouse-3");
  ASSERT_EQ(v.kind(), PropertyValue::Kind::kString);
  EXPECT_EQ(v.as_string(), "warehouse-3");
}

TEST(ParsePropertyScalar, Int64MaxBecomesInt) {
  const auto v = ParsePropertyScalar("9223372036854775807");
  ASSERT_EQ(v.kind(), PropertyValue::Kind::kInt);
  EXPECT_EQ(v.as_int(), std::numeric_limits<std::int64_t>::max());
}

TEST(ParsePropertyScalar, OnePastInt64MaxBecomesDoubleNotWrapped) {
  const auto v = ParsePropertyScalar("9223372036854775808");
  ASSERT_EQ(v.kind(), PropertyValue::Kind::kDouble);
  EXPECT_GT(v.as_double(), 0.0);
  EXPECT_DOUBLE_EQ(v.as_double(), 9223372036854775808.0);
}

TEST(ParsePropertyScalar, NanTextStaysString) {
  const auto v = ParsePropertyScalar("nan");
  ASSERT_EQ(v.kind(), PropertyValue::Kind::kString);
  EXPECT_EQ(v.as_string(), "nan");
}

TEST(ParsePropertyScalar, InfTextStaysString) {
  const auto v = ParsePropertyScalar("inf");
  ASSERT_EQ(v.kind(), PropertyValue::Kind::kString);
  EXPECT_EQ(v.as_string(), "inf");
}

// ---------------------------------------------------------------------------
// ParsePropertiesJson
// ---------------------------------------------------------------------------

TEST(ParsePropertiesJson, HappyObjectAllFourTypes) {
  Properties out;
  std::string error;
  const bool ok = ParsePropertiesJson(
      R"({"site": "warehouse-3", "payload_kg": 12.5, "run_index": 7, "teleop": true})", &out,
      &error);
  ASSERT_TRUE(ok) << error;
  ASSERT_EQ(out.size(), 4u);
  EXPECT_EQ(out[0].first, "site");
  EXPECT_EQ(out[0].second.kind(), PropertyValue::Kind::kString);
  EXPECT_EQ(out[0].second.as_string(), "warehouse-3");
  EXPECT_EQ(out[1].first, "payload_kg");
  EXPECT_EQ(out[1].second.kind(), PropertyValue::Kind::kDouble);
  EXPECT_DOUBLE_EQ(out[1].second.as_double(), 12.5);
  EXPECT_EQ(out[2].first, "run_index");
  EXPECT_EQ(out[2].second.kind(), PropertyValue::Kind::kInt);
  EXPECT_EQ(out[2].second.as_int(), 7);
  EXPECT_EQ(out[3].first, "teleop");
  EXPECT_EQ(out[3].second.kind(), PropertyValue::Kind::kBool);
  EXPECT_TRUE(out[3].second.as_bool());
}

TEST(ParsePropertiesJson, EmptyInputLeavesOutUntouched) {
  Properties out;
  out.emplace_back("sentinel", std::string("unchanged"));
  std::string error;
  const bool ok = ParsePropertiesJson("", &out, &error);
  EXPECT_TRUE(ok);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].first, "sentinel");
}

TEST(ParsePropertiesJson, WhitespaceOnlyInputLeavesOutUntouched) {
  Properties out;
  out.emplace_back("sentinel", std::string("unchanged"));
  std::string error;
  const bool ok = ParsePropertiesJson("   \n\t  ", &out, &error);
  EXPECT_TRUE(ok);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].first, "sentinel");
}

TEST(ParsePropertiesJson, MalformedJsonIsAnError) {
  Properties out;
  std::string error;
  const bool ok = ParsePropertiesJson("{not json", &out, &error);
  EXPECT_FALSE(ok);
  EXPECT_FALSE(error.empty());
}

TEST(ParsePropertiesJson, TopLevelArrayIsAnError) {
  Properties out;
  std::string error;
  const bool ok = ParsePropertiesJson("[1, 2, 3]", &out, &error);
  EXPECT_FALSE(ok);
  EXPECT_NE(error.find("array"), std::string::npos) << error;
}

TEST(ParsePropertiesJson, NestedObjectValueIsAnError) {
  Properties out;
  std::string error;
  const bool ok = ParsePropertiesJson(R"({"a": {"b": 1}})", &out, &error);
  EXPECT_FALSE(ok);
  EXPECT_NE(error.find("a"), std::string::npos) << error;
}

TEST(ParsePropertiesJson, NestedArrayValueIsAnError) {
  Properties out;
  std::string error;
  const bool ok = ParsePropertiesJson(R"({"a": [1, 2]})", &out, &error);
  EXPECT_FALSE(ok);
  EXPECT_NE(error.find("a"), std::string::npos) << error;
}

TEST(ParsePropertiesJson, EmptyKeyIsAnError) {
  Properties out;
  std::string error;
  const bool ok = ParsePropertiesJson(R"({"": 1})", &out, &error);
  EXPECT_FALSE(ok);
  EXPECT_FALSE(error.empty());
}

TEST(ParsePropertiesJson, PreservesInputOrder) {
  Properties out;
  std::string error;
  const bool ok = ParsePropertiesJson(R"({"z": 1, "a": 2, "m": 3})", &out, &error);
  ASSERT_TRUE(ok) << error;
  ASSERT_EQ(out.size(), 3u);
  EXPECT_EQ(out[0].first, "z");
  EXPECT_EQ(out[1].first, "a");
  EXPECT_EQ(out[2].first, "m");
}

TEST(ParsePropertiesJson, JsonStringLeadingZeroStaysTheStringVerbatim) {
  // A caller that writes {"c3d.deviceid": "002"} has already expressed the
  // type it wants. ParsePropertyScalar must never run on this value.
  Properties out;
  std::string error;
  const bool ok = ParsePropertiesJson(R"({"c3d.deviceid": "002"})", &out, &error);
  ASSERT_TRUE(ok) << error;
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].second.kind(), PropertyValue::Kind::kString);
  EXPECT_EQ(out[0].second.as_string(), "002");
}

TEST(ParsePropertiesJson, JsonStringOfPlainDigitsStaysStringNotCoerced) {
  // Unlike "002", a plain digit string is not shielded by the leading-zero
  // rule, so this is the case that actually fails if ParsePropertyScalar ever
  // runs on a JSON string value: "7" would become the integer 7.
  Properties out;
  std::string error;
  const bool ok = ParsePropertiesJson(R"({"count": "7"})", &out, &error);
  ASSERT_TRUE(ok) << error;
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].second.kind(), PropertyValue::Kind::kString);
  EXPECT_EQ(out[0].second.as_string(), "7");
}

TEST(ConfiguredProperty, TheTextualKeyExceptionHasOneOwner) {
  // c3d.deviceid is commonly a zero-padded serial, and coercing "002" to 2
  // changes which robot the session belongs to. The exception lives beside the
  // coercion so a caller reading configuration cannot apply one without the
  // other, as a second copy in the caller could.
  EXPECT_TRUE(PropertyKeyMustStayTextual("c3d.deviceid"));
  EXPECT_FALSE(PropertyKeyMustStayTextual("c3d.app.version"));
  EXPECT_FALSE(PropertyKeyMustStayTextual("site"));

  // A plain digit string is the case the leading-zero rule does NOT cover, so it
  // is the one that proves the exception is doing work.
  EXPECT_EQ(ParseConfiguredProperty("c3d.deviceid", "7").kind(), PropertyValue::Kind::kString);
  EXPECT_EQ(ParseConfiguredProperty("c3d.deviceid", "7").as_string(), "7");
  EXPECT_EQ(ParseConfiguredProperty("robots_seen", "7").kind(), PropertyValue::Kind::kInt);
  EXPECT_EQ(ParseConfiguredProperty("robots_seen", "7").as_int(), 7);

  EXPECT_EQ(ParseConfiguredProperty("c3d.deviceid", "002").as_string(), "002");
  EXPECT_EQ(ParseConfiguredProperty("battery_full", "true").kind(), PropertyValue::Kind::kBool);
}
