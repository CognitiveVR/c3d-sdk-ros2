// Feeds a c3d-stream-formatter pose vector's input.json through c3d::pose::BlockEncoder, the
// way the formatter's reference encoder reads it. Shared by c3d_pose_encode_vector and the
// core tests, so that the command the reference checker drives and the gtest run the same code.
#pragma once

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

#include "c3d/pose.hpp"

namespace c3d {
namespace pose {

// The input (spec §7): `joint_count`, `steps` {pos, rot, scale}, `interval_us`, `reset_at`
// (sample indices where a manifest is sent; the encoder's creation is the send at 0), `t_us`,
// `events` [[sample, 0|1]], and `groups` {"j,g": [[x, y, z(, w)], ...]}, g one of p, r, s.
// Values are float32 written as decimals and are read as float32, then widened.
//
// The blocks in close order, or false with the reason in *error: malformed input, refused
// options, a refused sample, or a block the encoder could not encode.
bool EncodeVectorInput(const nlohmann::json& input, std::vector<EncodedBlock>* blocks,
                       std::string* error);

}  // namespace pose
}  // namespace c3d
