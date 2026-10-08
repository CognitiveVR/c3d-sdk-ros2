#pragma once

#include <string>

#include "c3d/types.hpp"

namespace c3d {

// A configuration scalar as its narrowest faithful PropertyValue.
//
// Order matters and is the whole design: bool, then integer, then double, then
// string. A caller that already knows a key must stay textual (a
// `keep_as_string` allow-list, e.g. `c3d.deviceid`) should skip this function
// for that key rather than rely on it to guess.
PropertyValue ParsePropertyScalar(const std::string& text);

// True for a property key whose value must stay textual whatever it looks like.
//
// `c3d.deviceid` is commonly a zero-padded serial, and coercing "002" to 2
// changes which robot the session belongs to. This lives here, beside the
// coercion, because it is a fact about the platform's schema rather than about
// any one caller -- and because a second copy in a caller is a second copy to
// forget.
bool PropertyKeyMustStayTextual(const std::string& key);

// `text` as the right PropertyValue for `key`: the coercion above, unless the key
// must stay textual. This is what a caller reading configuration should use.
PropertyValue ParseConfiguredProperty(const std::string& key, const std::string& text);

// A JSON OBJECT of properties, with each value taking its JSON type. Returns
// false and sets *error on anything that is not a flat JSON object of scalars.
// Empty or whitespace-only input is not an error: it leaves *out untouched.
// Never throws.
bool ParsePropertiesJson(const std::string& json_text, Properties* out, std::string* error);

}  // namespace c3d
