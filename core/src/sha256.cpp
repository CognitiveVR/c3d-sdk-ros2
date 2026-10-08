#include "sha256.hpp"

namespace c3d {
namespace {

constexpr std::uint32_t kRoundConstants[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4,
    0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe,
    0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f,
    0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
    0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc,
    0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
    0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116,
    0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
    0xc67178f2,
};

std::uint32_t RotateRight(std::uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }

void Compress(std::uint32_t state[8], const std::uint8_t block[64]) {
  std::uint32_t w[64];
  for (int i = 0; i < 16; ++i) {
    w[i] = (static_cast<std::uint32_t>(block[4 * i]) << 24) |
           (static_cast<std::uint32_t>(block[4 * i + 1]) << 16) |
           (static_cast<std::uint32_t>(block[4 * i + 2]) << 8) |
           static_cast<std::uint32_t>(block[4 * i + 3]);
  }
  for (int i = 16; i < 64; ++i) {
    const std::uint32_t s0 = RotateRight(w[i - 15], 7) ^ RotateRight(w[i - 15], 18) ^ (w[i - 15] >> 3);
    const std::uint32_t s1 = RotateRight(w[i - 2], 17) ^ RotateRight(w[i - 2], 19) ^ (w[i - 2] >> 10);
    w[i] = w[i - 16] + s0 + w[i - 7] + s1;
  }
  std::uint32_t a = state[0], b = state[1], c = state[2], d = state[3];
  std::uint32_t e = state[4], f = state[5], g = state[6], h = state[7];
  for (int i = 0; i < 64; ++i) {
    const std::uint32_t s1 = RotateRight(e, 6) ^ RotateRight(e, 11) ^ RotateRight(e, 25);
    const std::uint32_t choice = (e & f) ^ (~e & g);
    const std::uint32_t t1 = h + s1 + choice + kRoundConstants[i] + w[i];
    const std::uint32_t s0 = RotateRight(a, 2) ^ RotateRight(a, 13) ^ RotateRight(a, 22);
    const std::uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
    const std::uint32_t t2 = s0 + majority;
    h = g;
    g = f;
    f = e;
    e = d + t1;
    d = c;
    c = b;
    b = a;
    a = t1 + t2;
  }
  state[0] += a;
  state[1] += b;
  state[2] += c;
  state[3] += d;
  state[4] += e;
  state[5] += f;
  state[6] += g;
  state[7] += h;
}

}  // namespace

std::array<std::uint8_t, 32> Sha256(const std::string& bytes) {
  std::uint32_t state[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                            0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
  const std::size_t length = bytes.size();
  std::size_t offset = 0;
  for (; offset + 64 <= length; offset += 64) {
    Compress(state, reinterpret_cast<const std::uint8_t*>(bytes.data()) + offset);
  }

  // The tail, the 0x80 terminator and the 64-bit big-endian bit length, padded to one block or
  // two: two whenever fewer than nine bytes of the first remain after the tail.
  std::uint8_t tail[128] = {0};
  const std::size_t remaining = length - offset;
  for (std::size_t i = 0; i < remaining; ++i) tail[i] = static_cast<std::uint8_t>(bytes[offset + i]);
  tail[remaining] = 0x80;
  const std::size_t tail_blocks = remaining + 9 <= 64 ? 1 : 2;
  const std::uint64_t bit_length = static_cast<std::uint64_t>(length) * 8;
  for (int i = 0; i < 8; ++i) {
    tail[tail_blocks * 64 - 1 - i] = static_cast<std::uint8_t>(bit_length >> (8 * i));
  }
  for (std::size_t block = 0; block < tail_blocks; ++block) Compress(state, tail + 64 * block);

  std::array<std::uint8_t, 32> digest{};
  for (int i = 0; i < 8; ++i) {
    digest[4 * i] = static_cast<std::uint8_t>(state[i] >> 24);
    digest[4 * i + 1] = static_cast<std::uint8_t>(state[i] >> 16);
    digest[4 * i + 2] = static_cast<std::uint8_t>(state[i] >> 8);
    digest[4 * i + 3] = static_cast<std::uint8_t>(state[i]);
  }
  return digest;
}

std::string Sha256Hex(const std::string& bytes) {
  static constexpr char kHex[] = "0123456789abcdef";
  const auto digest = Sha256(bytes);
  std::string out;
  out.reserve(64);
  for (const std::uint8_t byte : digest) {
    out.push_back(kHex[byte >> 4]);
    out.push_back(kHex[byte & 0x0F]);
  }
  return out;
}

}  // namespace c3d
