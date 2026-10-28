#include "pose_vector_input.hpp"

#include <cstdint>
#include <exception>
#include <map>
#include <set>
#include <utility>

namespace c3d {
namespace pose {
namespace {

double F32(const nlohmann::json& v) {
  return static_cast<double>(static_cast<float>(v.get<double>()));
}

bool Fail(std::string* error, const std::string& why) {
  if (error) *error = why;
  return false;
}

}  // namespace

bool EncodeVectorInput(const nlohmann::json& in, std::vector<EncodedBlock>* blocks,
                       std::string* error) {
  try {
    const int joints = in.at("joint_count").get<int>();
    if (joints < 1) return Fail(error, "joint_count must be >= 1");
    EncoderOptions options;
    options.joint_groups.assign(static_cast<std::size_t>(joints), 0);
    options.steps.pos_step = in.at("steps").at("pos").get<std::int32_t>();
    options.steps.rot_step = in.at("steps").at("rot").get<std::int32_t>();
    options.steps.scale_step = in.at("steps").at("scale").get<std::int32_t>();
    options.block_interval_us = in.at("interval_us").get<std::int64_t>();

    const auto t_us = in.at("t_us").get<std::vector<std::int64_t>>();
    const std::size_t n = t_us.size();
    std::map<std::pair<int, char>, const nlohmann::json*> groups;
    for (const auto& item : in.at("groups").items()) {
      const std::string& key = item.key();
      const auto comma = key.find(',');
      if (comma == std::string::npos || comma + 2 != key.size()) {
        return Fail(error, "bad group key " + key);
      }
      const int j = std::stoi(key.substr(0, comma));
      const char g = key[comma + 1];
      if (j < 0 || j >= joints) return Fail(error, "group key " + key + " names no joint");
      if (item.value().size() != n) {
        return Fail(error, "group " + key + " does not have one row per sample");
      }
      std::uint8_t bit = 0;
      if (g == 'p') {
        bit = kPosition;
      } else if (g == 'r') {
        bit = kRotation;
      } else if (g == 's') {
        bit = kScale;
      } else {
        return Fail(error, "group key " + key + " names no group");
      }
      options.joint_groups[static_cast<std::size_t>(j)] |= bit;
      groups[{j, g}] = &item.value();
    }

    std::map<std::size_t, bool> events;
    for (const auto& e : in.at("events")) {
      events[e.at(0).get<std::size_t>()] = e.at(1).get<int>() != 0;
    }
    std::set<std::size_t> resets;
    for (const auto& r : in.value("reset_at", nlohmann::json::array({0}))) {
      resets.insert(r.get<std::size_t>());
    }

    std::string why;
    auto encoder = BlockEncoder::Create(options, &why);
    if (!encoder) return Fail(error, "encoder refused the options: " + why);

    for (std::size_t i = 0; i < n; ++i) {
      if (i != 0 && resets.count(i)) {
        for (auto& b : encoder->ManifestSent()) blocks->push_back(std::move(b));
      }
      PoseSample s;
      s.t_us = t_us[i];
      s.joints.resize(static_cast<std::size_t>(joints));
      for (const auto& [jg, rows] : groups) {
        const nlohmann::json& row = (*rows)[i];
        JointPose& jp = s.joints[static_cast<std::size_t>(jg.first)];
        if (jg.second == 'p') jp.position = {F32(row.at(0)), F32(row.at(1)), F32(row.at(2))};
        if (jg.second == 's') jp.scale = {F32(row.at(0)), F32(row.at(1)), F32(row.at(2))};
        if (jg.second == 'r') {
          jp.rotation = {F32(row.at(0)), F32(row.at(1)), F32(row.at(2)), F32(row.at(3))};
        }
      }
      const auto ev = events.find(i);
      if (ev != events.end()) s.enabled = ev->second;
      if (!encoder->AddSample(s, blocks, &why)) {
        return Fail(error, "sample " + std::to_string(i) + " refused: " + why);
      }
    }
    for (auto& b : encoder->Close()) blocks->push_back(std::move(b));
    if (encoder->encode_failures() != 0) {
      return Fail(error, std::to_string(encoder->encode_failures()) +
                             " block(s) could not be encoded");
    }
    return true;
  } catch (const std::exception& e) {
    return Fail(error, std::string("malformed input: ") + e.what());
  }
}

}  // namespace pose
}  // namespace c3d
