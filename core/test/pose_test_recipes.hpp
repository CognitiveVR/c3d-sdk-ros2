// The formatter's six `generated/` pose vectors, rebuilt from their recipes (spec §7; the
// GENERATED table in c3d-stream-formatter's pose/vectors.py). They are too big to store, and
// they need no vector files, so the unit tests run them in every build and the vector test
// checks them against the formatter's generated/expected.json when it is present.
//
// Every recipe uses the default steps, a 10 s interval, one manifest send at sample 0 and one
// event, (0, enabled).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "c3d/pose.hpp"
#include "pose_test_decoder.hpp"

namespace c3d {
namespace pose_test {

struct Recipe {
  std::vector<std::uint8_t> joint_groups;
  std::vector<std::int64_t> t_us;
  std::vector<std::vector<pose::JointPose>> samples;  // [sample][joint]
  std::map<std::size_t, bool> enabled{{0, true}};      // sample -> the value set there
};

// x ^= x << 13; x ^= x >> 17; x ^= x << 5, all mod 2^32; one output per step.
inline std::vector<std::int64_t> Xorshift32(std::uint32_t seed, std::size_t count) {
  std::vector<std::int64_t> out(count);
  std::uint32_t x = seed;
  for (auto& v : out) {
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    v = x;
  }
  return out;
}

// float32((u mod 2000000001 - 1000000000) * 1e-4), the product in double.
inline double RandomValue(std::int64_t u) {
  return static_cast<double>(
      static_cast<float>(static_cast<double>(u % 2'000'000'001 - 1'000'000'000) * 1e-4));
}

// n_max and n_max_small: every joint at p = 0, r = identity, s = 1 in every sample.
inline Recipe Constant(int joints, std::size_t n, std::int64_t dt, bool root_scale) {
  Recipe r;
  for (int j = 0; j < joints; ++j) {
    r.joint_groups.push_back(static_cast<std::uint8_t>(
        pose::kPosition | pose::kRotation | ((j > 0 || root_scale) ? pose::kScale : 0)));
  }
  for (std::size_t i = 0; i < n; ++i) r.t_us.push_back(dt * static_cast<std::int64_t>(i));
  pose::JointPose jp;
  jp.position = {0, 0, 0};
  jp.rotation = {0, 0, 0, 1};
  jp.scale = {1, 1, 1};
  r.samples.assign(n, std::vector<pose::JointPose>(static_cast<std::size_t>(joints), jp));
  return r;
}

// split and split_manifest: 40 joints with p only, 3,800 near-incompressible samples at 2 kHz.
inline Recipe Split(std::int64_t t0) {
  const std::size_t joints = 40, n = 3'800;
  Recipe r;
  r.joint_groups.assign(joints, pose::kPosition);
  const auto u = Xorshift32(1, n * joints * 3);
  std::size_t at = 0;
  for (std::size_t i = 0; i < n; ++i) {
    r.t_us.push_back(t0 + 500 * static_cast<std::int64_t>(i));
    std::vector<pose::JointPose> s(joints);
    for (auto& jp : s) {
      jp.position.x = RandomValue(u[at++]);
      jp.position.y = RandomValue(u[at++]);
      jp.position.z = RandomValue(u[at++]);
    }
    r.samples.push_back(std::move(s));
  }
  return r;
}

// split_deep and split_period: 80 joints with p only; a 5-sample block from t1, then 2,150
// samples at 1 kHz from t2. Joint 0 sits at 2 steps, then at 1 step all through block 2;
// joints 1..79 are near-incompressible.
inline Recipe SplitDeep(std::int64_t t1, std::int64_t t2) {
  const std::size_t joints = 80, n1 = 5, n2 = 2'150, n = n1 + n2;
  Recipe r;
  r.joint_groups.assign(joints, pose::kPosition);
  const auto u = Xorshift32(1, n * (joints - 1) * 3);
  std::size_t at = 0;
  for (std::size_t i = 0; i < n; ++i) {
    r.t_us.push_back(i < n1 ? t1 + 100'000 * static_cast<std::int64_t>(i)
                            : t2 + 1'000 * static_cast<std::int64_t>(i - n1));
    std::vector<pose::JointPose> s(joints);
    s[0].position = {i < n1 ? 0.0002 : 0.0001, 0, 0};
    for (std::size_t j = 1; j < joints; ++j) {
      s[j].position.x = RandomValue(u[at++]);
      s[j].position.y = RandomValue(u[at++]);
      s[j].position.z = RandomValue(u[at++]);
    }
    r.samples.push_back(std::move(s));
  }
  return r;
}

// By the formatter's names.
inline const std::map<std::string, std::function<Recipe()>>& Recipes() {
  static const std::map<std::string, std::function<Recipe()>> r = {
      {"n_max", [] { return Constant(240, 900, 11'111, true); }},
      {"n_max_small", [] { return Constant(11, 10'000, 1'000, false); }},
      {"split", [] { return Split(29'100'000); }},
      {"split_manifest", [] { return Split(1'000'000); }},
      {"split_deep", [] { return SplitDeep(1'000'000, 11'000'000); }},
      {"split_period", [] { return SplitDeep(25'000'000, 35'000'000); }},
  };
  return r;
}

struct RecipeResult {
  std::vector<pose::EncodedBlock> blocks;
  std::vector<DecodedBlock> decoded;
  std::uint64_t encode_failures = 0;
  std::string error;  // a refused sample or a block the test decoder rejected
  // The largest |decoded - input| over every value of every sample, in steps.
  double worst_steps = 0;
};

inline RecipeResult RunRecipe(const Recipe& recipe) {
  RecipeResult out;
  pose::EncoderOptions options;
  options.joint_groups = recipe.joint_groups;
  options.block_interval_us = 10'000'000;
  auto encoder = pose::BlockEncoder::Create(options, &out.error);
  if (!encoder) return out;
  for (std::size_t i = 0; i < recipe.t_us.size(); ++i) {
    pose::PoseSample s;
    s.t_us = recipe.t_us[i];
    s.joints = recipe.samples[i];
    const auto ev = recipe.enabled.find(i);
    if (ev != recipe.enabled.end()) s.enabled = ev->second;
    if (!encoder->AddSample(s, &out.blocks, &out.error)) return out;
  }
  for (auto& b : encoder->Close()) out.blocks.push_back(std::move(b));
  out.encode_failures = encoder->encode_failures();

  Decoder decoder(static_cast<std::int32_t>(recipe.joint_groups.size()));
  std::size_t off = 0;
  for (const auto& b : out.blocks) {
    out.decoded.push_back(decoder.Decode(b.data));
    const DecodedBlock& d = out.decoded.back();
    if (!d.error.empty()) {
      out.error = "block " + std::to_string(out.decoded.size() - 1) + " rejected: " + d.error;
      return out;
    }
    for (const auto& [key, rows] : d.ints) {
      for (std::size_t i = 0; i < rows.size(); ++i) {
        const pose::JointPose& jp = recipe.samples[off + i][static_cast<std::size_t>(key.first)];
        double src[4] = {jp.rotation.x, jp.rotation.y, jp.rotation.z, jp.rotation.w};
        if (key.second == 'p') src[0] = jp.position.x, src[1] = jp.position.y, src[2] = jp.position.z;
        if (key.second == 's') src[0] = jp.scale.x, src[1] = jp.scale.y, src[2] = jp.scale.z;
        const int channels = key.second == 'r' ? 4 : 3;
        for (int c = 0; c < channels; ++c) {
          // Every recipe step is the default 100000, 1e-4 units; widened before multiplying.
          const double value = static_cast<double>(rows[i][c]) * 100'000 / 1e9;
          out.worst_steps = std::max(out.worst_steps, std::abs(value - src[c]) / 1e-4);
        }
      }
    }
    off += static_cast<std::size_t>(d.n);
  }
  if (off != recipe.t_us.size()) out.error = "the blocks hold " + std::to_string(off) + " samples";
  return out;
}

}  // namespace pose_test
}  // namespace c3d
