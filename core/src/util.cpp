#include "util.hpp"
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <sstream>

namespace c3d {

double EpochSeconds() {
  using namespace std::chrono;
  return duration<double>(system_clock::now().time_since_epoch()).count();
}

double SteadySeconds() {
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

double Round5(double v) {
  if (!std::isfinite(v)) return 0.0;
  return std::round(v * 100000.0) / 100000.0;
}

std::string PadNumber(std::uint64_t n, int width) {
  std::ostringstream os;
  os << std::setw(width) << std::setfill('0') << n;
  return os.str();
}

bool SceneVersionNumber(const std::string& text, std::uint32_t* out) {
  if (text.empty()) return false;
  std::uint64_t value = 0;
  for (char c : text) {
    if (c < '0' || c > '9') return false;
    value = value * 10 + static_cast<std::uint64_t>(c - '0');
    if (value > 0xFFFFFFFFull) return false;
  }
  if (out) *out = static_cast<std::uint32_t>(value);
  return true;
}

bool IsValidUtf8(const std::string& s) {
  std::size_t i = 0;
  while (i < s.size()) {
    const unsigned char lead = static_cast<unsigned char>(s[i]);
    std::size_t length = 0;
    std::uint32_t code = 0;
    if (lead < 0x80) {
      ++i;
      continue;
    } else if ((lead & 0xE0) == 0xC0) {
      length = 2;
      code = lead & 0x1Fu;
    } else if ((lead & 0xF0) == 0xE0) {
      length = 3;
      code = lead & 0x0Fu;
    } else if ((lead & 0xF8) == 0xF0) {
      length = 4;
      code = lead & 0x07u;
    } else {
      return false;
    }
    if (i + length > s.size()) return false;
    for (std::size_t k = 1; k < length; ++k) {
      const unsigned char continuation = static_cast<unsigned char>(s[i + k]);
      if ((continuation & 0xC0) != 0x80) return false;
      code = (code << 6) | (continuation & 0x3Fu);
    }
    // Overlong forms, surrogates and anything past U+10FFFF are rejected, because
    // they are exactly what nlohmann rejects: a validator that accepted them
    // would hand the JSON writer a string it then rewrites.
    if (length == 2 && code < 0x80) return false;
    if (length == 3 && code < 0x800) return false;
    if (length == 4 && code < 0x10000) return false;
    if (code > 0x10FFFF) return false;
    if (code >= 0xD800 && code <= 0xDFFF) return false;
    i += length;
  }
  return true;
}

namespace {
constexpr const char* kBase64Alphabet =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
}  // namespace

std::string Base64Encode(const std::string& bytes) {
  std::string out;
  out.reserve(((bytes.size() + 2) / 3) * 4);
  std::size_t i = 0;
  while (i + 2 < bytes.size()) {
    const std::uint32_t triple = (static_cast<unsigned char>(bytes[i]) << 16) |
                                 (static_cast<unsigned char>(bytes[i + 1]) << 8) |
                                 static_cast<unsigned char>(bytes[i + 2]);
    out.push_back(kBase64Alphabet[(triple >> 18) & 0x3F]);
    out.push_back(kBase64Alphabet[(triple >> 12) & 0x3F]);
    out.push_back(kBase64Alphabet[(triple >> 6) & 0x3F]);
    out.push_back(kBase64Alphabet[triple & 0x3F]);
    i += 3;
  }
  if (i < bytes.size()) {
    std::uint32_t triple = static_cast<unsigned char>(bytes[i]) << 16;
    const bool two = i + 1 < bytes.size();
    if (two) triple |= static_cast<unsigned char>(bytes[i + 1]) << 8;
    out.push_back(kBase64Alphabet[(triple >> 18) & 0x3F]);
    out.push_back(kBase64Alphabet[(triple >> 12) & 0x3F]);
    out.push_back(two ? kBase64Alphabet[(triple >> 6) & 0x3F] : '=');
    out.push_back('=');
  }
  return out;
}

bool Base64Decode(const std::string& text, std::string* out) {
  if (out == nullptr) return false;
  out->clear();
  out->reserve(text.size() / 4 * 3);
  std::uint32_t accumulator = 0;
  int bits = 0;
  std::size_t padding = 0;
  for (char c : text) {
    if (c == '=') {
      ++padding;
      continue;
    }
    // Padding is terminal: a value after it means the text is not one base64
    // document, and decoding on regardless would silently return a prefix.
    if (padding > 0) return false;
    const char* found = std::strchr(kBase64Alphabet, c);
    if (found == nullptr || c == '\0') return false;
    accumulator = (accumulator << 6) | static_cast<std::uint32_t>(found - kBase64Alphabet);
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      out->push_back(static_cast<char>((accumulator >> bits) & 0xFF));
    }
  }
  return padding <= 2;
}

}  // namespace c3d
