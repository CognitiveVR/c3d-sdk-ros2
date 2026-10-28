// Encodes one pose conformance vector's input with c3d::pose::BlockEncoder, so that the
// platform's reference checker can judge this encoder (tools/README.md has the command).
//
//   c3d_pose_encode_vector < input.json > blocks
//
// Input: the `input.json` of a c3d-stream-formatter pose vector (pose_vector_input.hpp).
// Output: every block in close order, each as a 4-byte little-endian length and then its
// bytes. Exit status 1, with the reason on stderr, when the input is malformed or the encoder
// refuses it.

#include <nlohmann/json.hpp>

#include <cstdint>
#include <cstdio>
#include <exception>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "pose_vector_input.hpp"

int main() {
  std::vector<c3d::pose::EncodedBlock> blocks;
  std::string error;
  try {
    std::ostringstream text;
    text << std::cin.rdbuf();
    const auto in = nlohmann::json::parse(text.str());
    if (!c3d::pose::EncodeVectorInput(in, &blocks, &error)) {
      std::cerr << "c3d_pose_encode_vector: " << error << "\n";
      return 1;
    }
  } catch (const std::exception& e) {
    std::cerr << "c3d_pose_encode_vector: input is not JSON: " << e.what() << "\n";
    return 1;
  }
  for (const auto& b : blocks) {
    const auto n = static_cast<std::uint32_t>(b.data.size());
    const unsigned char len[4] = {static_cast<unsigned char>(n & 0xFF),
                                  static_cast<unsigned char>((n >> 8) & 0xFF),
                                  static_cast<unsigned char>((n >> 16) & 0xFF),
                                  static_cast<unsigned char>((n >> 24) & 0xFF)};
    if (std::fwrite(len, 1, 4, stdout) != 4 ||
        std::fwrite(b.data.data(), 1, b.data.size(), stdout) != b.data.size()) {
      return 1;
    }
  }
  return std::fflush(stdout) == 0 ? 0 : 1;
}
