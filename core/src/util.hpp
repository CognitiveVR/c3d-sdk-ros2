#pragma once
#include <cstdint>
#include <string>
namespace c3d {
// Epoch wall-clock seconds. Deliberately system_clock: the C3D session id and
// every payload timestamp are Unix epoch seconds.
double EpochSeconds();
// Monotonic seconds, for measuring our own durations and backoff. Immune to NTP
// steps, which system_clock is not.
double SteadySeconds();
// Round to 5 decimal places, matching the Unity SDK's wire precision. We match
// the value, not the byte representation; see docs/data-format.md.
double Round5(double v);
std::string PadNumber(std::uint64_t n, int width);

// The scene version as the NUMBER the binary payloads carry and the JSON URLs
// spell out. Digits only, no sign and no whitespace, and it must fit UInt32.
// `out` may be null when only the verdict is wanted.
bool SceneVersionNumber(const std::string& text, std::uint32_t* out);

// Whether a byte string is well-formed UTF-8.
//
// This is a SPOOL question, not a text question. A JSON string holds only valid
// UTF-8, so the chunk envelope cannot carry a packed Cap'n Proto body in its
// `body` field byte for byte. The spool asks this and base64s what fails.
bool IsValidUtf8(const std::string& s);

// Standard base64, padded, no line breaks.
std::string Base64Encode(const std::string& bytes);
bool Base64Decode(const std::string& text, std::string* out);
}  // namespace c3d
