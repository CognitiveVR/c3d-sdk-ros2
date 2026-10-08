#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace c3d {

// SHA-256 (FIPS 180-4), implemented here so core takes no cryptography dependency. Used to
// derive identifiers, never for anything secret.
std::array<std::uint8_t, 32> Sha256(const std::string& bytes);

// The digest as 64 lowercase hex characters.
std::string Sha256Hex(const std::string& bytes);

}  // namespace c3d
