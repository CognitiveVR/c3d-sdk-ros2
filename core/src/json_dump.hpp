#pragma once

#include <nlohmann/json.hpp>

#include <string>

namespace c3d {

// Every JSON document core writes goes through this. A string that is not valid UTF-8 -- a
// Latin-1 byte in a driver's diagnostic, say -- has each bad byte replaced with U+FFFD, where the
// default dump() throws, and a throw on the worker thread ends the process.
inline std::string DumpJson(const nlohmann::ordered_json& document) {
  return document.dump(-1, ' ', false, nlohmann::ordered_json::error_handler_t::replace);
}

}  // namespace c3d
