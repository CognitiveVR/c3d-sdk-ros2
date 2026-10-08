#include "c3d/properties.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdint>

namespace c3d {
namespace {

bool IsBlank(const std::string& text) {
  return std::all_of(text.begin(), text.end(),
                     [](unsigned char c) { return std::isspace(c) != 0; });
}

// A digit run (past an optional leading '-') that starts with '0' followed by
// another digit: "002", not the bare "0" or a leading "0." before a fraction.
bool HasPaddingLeadingZero(const std::string& text) {
  const std::size_t i = (!text.empty() && text[0] == '-') ? 1 : 0;
  return i + 1 < text.size() && text[i] == '0' &&
        std::isdigit(static_cast<unsigned char>(text[i + 1])) != 0;
}

bool ParseWholeInt(const std::string& text, std::int64_t* out) {
  if (text.empty()) return false;
  const char* begin = text.data();
  const char* end = text.data() + text.size();
  const auto result = std::from_chars(begin, end, *out);
  return result.ec == std::errc() && result.ptr == end;
}

bool ParseWholeDouble(const std::string& text, double* out) {
  if (text.empty()) return false;
  const char* begin = text.data();
  const char* end = text.data() + text.size();
  const auto result = std::from_chars(begin, end, *out);
  return result.ec == std::errc() && result.ptr == end;
}

}  // namespace

PropertyValue ParsePropertyScalar(const std::string& text) {
  if (text == "true") return PropertyValue(true);
  if (text == "false") return PropertyValue(false);

  if (!HasPaddingLeadingZero(text)) {
    std::int64_t i = 0;
    if (ParseWholeInt(text, &i)) return PropertyValue(i);
    double d = 0.0;
    // isfinite rejects "nan" and "inf", both of which from_chars parses as a
    // complete, successful match.
    if (ParseWholeDouble(text, &d) && std::isfinite(d)) return PropertyValue(d);
  }
  return PropertyValue(text);
}

bool ParsePropertiesJson(const std::string& json_text, Properties* out, std::string* error) {
  if (IsBlank(json_text)) return true;

  try {
    const auto parsed = nlohmann::ordered_json::parse(json_text);
    if (!parsed.is_object()) {
      if (error) {
        *error = "properties JSON must be an object, got " + std::string(parsed.type_name());
      }
      return false;
    }

    Properties props;
    props.reserve(parsed.size());
    for (const auto& item : parsed.items()) {
      const std::string& key = item.key();
      if (key.empty()) {
        if (error) *error = "properties JSON has an empty key";
        return false;
      }
      const auto& value = item.value();
      if (value.is_boolean()) {
        props.emplace_back(key, value.get<bool>());
      } else if (value.is_number_integer()) {
        props.emplace_back(key, value.get<std::int64_t>());
      } else if (value.is_number_float()) {
        props.emplace_back(key, value.get<double>());
      } else if (value.is_string()) {
        props.emplace_back(key, value.get<std::string>());
      } else {
        if (error) {
          *error =
              "property '" + key + "' is not a scalar (" + std::string(value.type_name()) + ")";
        }
        return false;
      }
    }
    *out = std::move(props);
    return true;
  } catch (const std::exception& e) {
    if (error) *error = std::string("could not parse properties JSON: ") + e.what();
    return false;
  }
}

bool PropertyKeyMustStayTextual(const std::string& key) {
  return key == "c3d.deviceid";
}

PropertyValue ParseConfiguredProperty(const std::string& key, const std::string& text) {
  return PropertyKeyMustStayTextual(key) ? PropertyValue(text) : ParsePropertyScalar(text);
}

}  // namespace c3d
