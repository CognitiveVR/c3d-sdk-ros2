// A pose block decoder for the tests, written from the spec's §4 decoding rules and not from
// the encoder: the pose tests decode what the encoder wrote rather than asking it, which is
// the only arrangement in which a bug the two would share is visible. It is checked itself
// against the formatter's `bad/` vectors (test_pose_vectors.cpp).
//
// Codecs: zstd, and none when allowed. lz4 and gzip are not linked into core/, and this SDK
// writes neither, so a block in either is refused with `unsupported_codec`.
#pragma once

#include <zstd.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace c3d {
namespace pose_test {

using Vals = std::array<std::int64_t, 4>;
using GroupKey = std::pair<int, char>;  // (joint, 'p' | 'r' | 's')

struct DecodedBlock {
  std::string error;  // empty when accepted; else the reference decoder's BlockError code
  bool keyframe = false;
  int codec = -1;
  std::int32_t n = 0;
  std::int32_t j = 0;
  std::int64_t start_us = 0;
  std::int32_t pos_step = 0, rot_step = 0, scale_step = 0;
  std::vector<std::int64_t> t_us;
  std::vector<std::uint8_t> masks;
  std::map<GroupKey, std::vector<Vals>> ints;  // written or carried over
  std::map<GroupKey, bool> written;
  std::vector<std::pair<std::int32_t, std::int32_t>> events;
  std::string body;  // decompressed
};

// One object's decoder: the carried-over values persist from block to block.
class Decoder {
 public:
  explicit Decoder(std::int32_t joint_count, bool allow_none = false)
      : joint_count_(joint_count), allow_none_(allow_none) {}

  void Reset() { last_.clear(); }

  DecodedBlock Decode(const std::string& blob) {
    DecodedBlock out;
    out.error = DecodeInto(blob, &out);
    return out;
  }

 private:
  static constexpr std::size_t kMaxBody = 8u << 20;

  static std::int32_t I32(const std::string& b, std::size_t o) {
    std::uint32_t u = 0;
    for (int i = 0; i < 4; ++i) u |= static_cast<std::uint32_t>(static_cast<unsigned char>(b[o + i])) << (8 * i);
    return static_cast<std::int32_t>(u);
  }
  static std::int64_t I64(const std::string& b, std::size_t o) {
    std::uint64_t u = 0;
    for (int i = 0; i < 8; ++i) u |= static_cast<std::uint64_t>(static_cast<unsigned char>(b[o + i])) << (8 * i);
    return static_cast<std::int64_t>(u);
  }

  static std::string Unzstd(const std::string& payload, std::string* body) {
    // Frame header: 4-byte magic, then the descriptor, whose bit 2 is the content checksum flag.
    if (payload.size() < 5 || static_cast<unsigned char>(payload[0]) != 0x28 ||
        static_cast<unsigned char>(payload[1]) != 0xB5 || static_cast<unsigned char>(payload[2]) != 0x2F ||
        static_cast<unsigned char>(payload[3]) != 0xFD) {
      return "bad_frame";
    }
    if (!(static_cast<unsigned char>(payload[4]) & 0x04)) return "missing_checksum";
    const unsigned long long size = ZSTD_getFrameContentSize(payload.data(), payload.size());
    if (size == ZSTD_CONTENTSIZE_ERROR) return "bad_frame";
    if (size == ZSTD_CONTENTSIZE_UNKNOWN) return "missing_content_size";
    if (size > kMaxBody) return "declared_size_over_cap";
    const std::size_t frame = ZSTD_findFrameCompressedSize(payload.data(), payload.size());
    if (ZSTD_isError(frame)) return "truncated_payload";
    body->assign(static_cast<std::size_t>(size), '\0');
    const std::size_t got = ZSTD_decompress(body->empty() ? nullptr : &(*body)[0], body->size(),
                                            payload.data(), frame);
    if (ZSTD_isError(got) || got != size) return "corrupt_payload";
    if (frame != payload.size()) return "trailing_payload";
    return "";
  }

  std::string DecodeInto(const std::string& blob, DecodedBlock* out) {
    if (blob.size() < 4) return "truncated_prefix";
    if (static_cast<unsigned char>(blob[0]) != 0xC3) return "bad_magic";
    if (blob[1] != 1) return "unknown_version";
    const int codec = static_cast<unsigned char>(blob[2]);
    if (codec > 3) return "unknown_codec";
    const int flags = static_cast<unsigned char>(blob[3]);
    if (flags & ~1) return "unknown_flags";
    out->codec = codec;
    out->keyframe = flags & 1;
    const std::string payload = blob.substr(4);
    std::string& body = out->body;
    if (codec == 0) {
      if (!allow_none_) return "codec_not_allowed";
      if (payload.size() > kMaxBody) return "declared_size_over_cap";
      body = payload;
    } else if (codec == 1) {
      const std::string e = Unzstd(payload, &body);
      if (!e.empty()) return e;
    } else {
      return "unsupported_codec";
    }

    if (body.size() < 28) return "truncated_body";
    out->n = I32(body, 0);
    out->j = I32(body, 4);
    out->start_us = I64(body, 8);
    out->pos_step = I32(body, 16);
    out->rot_step = I32(body, 20);
    out->scale_step = I32(body, 24);
    const std::int32_t n = out->n;
    if (n < 1) return "bad_sample_count";
    if (out->j < 1) return "bad_joint_count";
    if (out->j != joint_count_) return "joint_count_mismatch";
    if (out->pos_step <= 0 || out->rot_step <= 0 || out->scale_step <= 0 ||
        out->rot_step > 100'000'000) {
      return "bad_step";
    }
    std::size_t o = 28;
    auto need = [&](std::size_t bytes) { return o + bytes <= body.size(); };

    if (!need(4u * static_cast<std::size_t>(n))) return "truncated_body";
    std::vector<std::int64_t> times(static_cast<std::size_t>(n));
    for (std::int32_t i = 0; i < n; ++i, o += 4) times[static_cast<std::size_t>(i)] = I32(body, o);
    if (times[0] != 0) return "bad_first_time";
    __extension__ typedef __int128 Wide;
    Wide sum = 0;
    for (std::int32_t i = 1; i < n; ++i) {
      if (times[static_cast<std::size_t>(i)] < 0) return "negative_time_delta";
    }
    for (auto t : times) sum += t;
    if (static_cast<Wide>(out->start_us) + sum > std::numeric_limits<std::int64_t>::max()) {
      return "time_overflow";
    }
    std::int64_t t = out->start_us;
    for (auto d : times) out->t_us.push_back(t += d);

    if (!need(static_cast<std::size_t>(out->j))) return "truncated_body";
    for (std::int32_t j = 0; j < out->j; ++j) out->masks.push_back(static_cast<std::uint8_t>(body[o++]) & 7);

    for (std::int32_t j = 0; j < out->j; ++j) {
      for (auto [g, bit, c] : {std::tuple<char, int, int>{'p', 1, 3}, {'r', 2, 4}, {'s', 4, 3}}) {
        const GroupKey key{j, g};
        if (out->masks[static_cast<std::size_t>(j)] & bit) {
          if (!need(4u * static_cast<std::size_t>(c) * static_cast<std::size_t>(n))) return "truncated_body";
          std::vector<Vals> v(static_cast<std::size_t>(n), Vals{0, 0, 0, 0});
          for (int ch = 0; ch < c; ++ch) {
            std::int64_t acc = 0;
            for (std::int32_t i = 0; i < n; ++i, o += 4) v[static_cast<std::size_t>(i)][ch] = acc += I32(body, o);
          }
          if (g == 'r') {
            for (const auto& q : v) {
              if (q == Vals{0, 0, 0, 0}) return "zero_quaternion";
            }
          }
          out->ints[key] = std::move(v);
          out->written[key] = true;
        } else if (last_.count(key) && !out->keyframe) {
          out->ints[key] = std::vector<Vals>(static_cast<std::size_t>(n), last_[key]);
          out->written[key] = false;
        }
      }
    }

    if (!need(4)) return "truncated_body";
    const std::int32_t n_events = I32(body, o);
    o += 4;
    if (n_events < 0) return "bad_event_count";
    if (!need(8u * static_cast<std::size_t>(n_events))) return "truncated_body";
    std::int32_t prev = -1;
    for (std::int32_t e = 0; e < n_events; ++e, o += 8) {
      const std::int32_t idx = I32(body, o), val = I32(body, o + 4);
      if (idx < 0 || idx >= n) return "bad_event_index";
      if (idx <= prev) return "events_out_of_order";
      if (val != 0 && val != 1) return "bad_enabled_value";
      out->events.emplace_back(idx, val);
      prev = idx;
    }
    if (o != body.size()) return "trailing_body";
    if (out->keyframe) {
      if (out->events.empty() || out->events.front().first != 0) return "keyframe_without_enabled";
      for (const auto& kv : last_) {
        if (!out->written.count(kv.first) || !out->written[kv.first]) return "keyframe_missing_group";
      }
      last_.clear();
    }
    for (const auto& kv : out->ints) last_[kv.first] = kv.second.back();
    return "";
  }

  std::int32_t joint_count_;
  bool allow_none_;
  std::map<GroupKey, Vals> last_;
};

}  // namespace pose_test
}  // namespace c3d
