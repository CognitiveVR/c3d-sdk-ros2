// The pose block encoder. The rule numbers below are §3 of c3d-stream-formatter's
// docs/pose/spec.md; the reference implementation is its c3d_stream_formatter/pose/encode.py,
// and this file decides every block exactly as that one does, so that the vectors' expected
// bodies hold byte for byte.

#include "c3d/pose.hpp"

#include <zstd.h>

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <utility>

namespace c3d {
namespace pose {
namespace {

constexpr std::uint8_t kMagic = 0xC3;
constexpr std::uint8_t kVersion = 1;
constexpr std::uint8_t kCodecZstd = 1;
constexpr std::uint8_t kFlagKeyframe = 0x01;
constexpr int kZstdLevel = 3;
constexpr std::int64_t kHeaderBytes = 28;
constexpr std::int32_t kMaxRotStep = 100'000'000;
constexpr double kNano = 1e-9;
constexpr std::int64_t kInt32Min = std::numeric_limits<std::int32_t>::min();
constexpr std::int64_t kInt32Max = std::numeric_limits<std::int32_t>::max();

bool FitsInt32(std::int64_t v) { return v >= kInt32Min && v <= kInt32Max; }

// Python's floor division, which the spec's floor(start_us / K) is: C++ truncates toward 0.
std::int64_t FloorDiv(std::int64_t a, std::int64_t b) {
  std::int64_t q = a / b;
  if ((a % b != 0) && ((a < 0) != (b < 0))) --q;
  return q;
}

// Rule 4: in double, never float, with the step itself computed in double. The bound is
// checked on the double so an out-of-range value is refused rather than converted with
// undefined behaviour.
bool Quantize(double x, double step, std::int64_t* out) {
  if (!std::isfinite(x)) return false;
  const double q = std::floor(x / step + 0.5);
  if (!(q >= static_cast<double>(kInt32Min) && q <= static_cast<double>(kInt32Max))) return false;
  *out = static_cast<std::int64_t>(q);
  return true;
}

void PutI32(std::string* s, std::int64_t v) {
  const auto u = static_cast<std::uint32_t>(static_cast<std::int32_t>(v));
  s->push_back(static_cast<char>(u & 0xFF));
  s->push_back(static_cast<char>((u >> 8) & 0xFF));
  s->push_back(static_cast<char>((u >> 16) & 0xFF));
  s->push_back(static_cast<char>((u >> 24) & 0xFF));
}

void PutI64(std::string* s, std::int64_t v) {
  const auto u = static_cast<std::uint64_t>(v);
  for (int i = 0; i < 8; ++i) s->push_back(static_cast<char>((u >> (8 * i)) & 0xFF));
}

using Vals = std::array<std::int64_t, 4>;

// Rule 5, sample 0: the sign that makes w positive; with w = 0, the first non-zero of
// (x, y, z). All zeros keeps the captured sign (it never reaches here: zeros are repaired).
int CanonicalSign(const Vals& q) {
  for (int c : {3, 0, 1, 2}) {
    if (q[c] != 0) return q[c] > 0 ? 1 : -1;
  }
  return 1;
}

// Rule 5, later samples: negate when the integer dot product with the previous chosen
// sample is negative; a tie keeps the captured sign. 128-bit, because four products of
// int32 values can pass int64.
bool DotIsNegative(const Vals& a, const Vals& b) {
  __extension__ typedef __int128 Wide;
  Wide d = 0;
  for (int c = 0; c < 4; ++c) d += static_cast<Wide>(a[c]) * static_cast<Wide>(b[c]);
  return d < 0;
}

Vals Negated(const Vals& q) { return {-q[0], -q[1], -q[2], -q[3]}; }

struct Slot {
  std::int32_t joint;
  std::uint8_t bit;  // kPosition, kRotation or kScale
  int channels;      // 3, 4, 3
  double step;       // in the group's units
};

}  // namespace

std::int32_t MaxSamplesPerBlock(std::int32_t joint_count, std::int64_t channel_count) {
  const std::int64_t n =
      (kMaxEncoderBodyBytes - kHeaderBytes - joint_count - 4) / (12 + 4 * channel_count);
  return static_cast<std::int32_t>(n < 1 ? 1 : n);
}

struct BlockEncoder::Impl {
  std::int32_t joint_count = 0;
  Steps steps;
  std::int64_t interval_us = 0;
  std::int32_t n_max = 0;
  // Every (joint, group) the object has, in the order a block writes them: joint by joint,
  // p, r, s within a joint.
  std::vector<Slot> slots;

  // The open block. Values are quantised as captured (zero rotations already repaired),
  // before the sign rule: the sign rule runs per block, from that block's own sample 0, and
  // a split piece is a block of its own.
  std::vector<std::int64_t> t_us;
  std::vector<std::vector<Vals>> vals;           // [slot][sample]
  std::vector<std::pair<std::int32_t, int>> events;  // (sample index in the open block, 0|1)
  int enabled_before_open = 1;
  // The previous chosen-sign rotation per slot in the open block, to bound the deltas a
  // sample would write before accepting it.
  std::vector<Vals> chosen_rot;

  // Across blocks.
  std::optional<std::int64_t> last_t_us;
  int enabled_in_effect = 1;
  std::vector<std::optional<Vals>> last_rot;   // per slot: the previous captured rotation
  std::vector<std::optional<Vals>> table;      // rule 6: per slot, the last value sent
  std::optional<std::int64_t> prev_period;     // rule 11: the previous block's period
  bool first_after_send = true;
  std::int64_t next_seq = 0;
  std::int64_t first_seq = 0;
  std::uint64_t repaired = 0;
  std::uint64_t failures = 0;

  ZSTD_CCtx* cctx = nullptr;

  ~Impl() { ZSTD_freeCCtx(cctx); }

  std::size_t open_n() const { return t_us.size(); }

  void ClearOpen() {
    t_us.clear();
    for (auto& v : vals) v.clear();
    events.clear();
  }

  // One piece [lo, hi) of the open block, encoded but not committed: the table entries it
  // would leave are returned in *entries.
  bool EncodePiece(std::size_t lo, std::size_t hi, bool keyframe, std::string* blob,
                   std::vector<std::optional<Vals>>* entries) {
    const std::size_t n = hi - lo;
    std::string body;
    PutI32(&body, static_cast<std::int64_t>(n));
    PutI32(&body, joint_count);
    PutI64(&body, t_us[lo]);
    PutI32(&body, steps.pos_step);
    PutI32(&body, steps.rot_step);
    PutI32(&body, steps.scale_step);

    // Rule 2. A sample joins a block only within the interval of its first sample, so a
    // delta is under 10 s and fits; checked anyway, because an overflow here is a wrong time.
    PutI32(&body, 0);
    for (std::size_t i = lo + 1; i < hi; ++i) {
      const std::int64_t d = t_us[i] - t_us[i - 1];
      if (!FitsInt32(d)) return false;
      PutI32(&body, d);
    }

    std::string masks(static_cast<std::size_t>(joint_count), '\0');
    std::string groups;
    *entries = std::vector<std::optional<Vals>>(slots.size());
    for (std::size_t k = 0; k < slots.size(); ++k) {
      const Slot& slot = slots[k];
      std::vector<Vals> q(vals[k].begin() + static_cast<std::ptrdiff_t>(lo),
                          vals[k].begin() + static_cast<std::ptrdiff_t>(hi));
      if (slot.bit == kRotation) {
        if (CanonicalSign(q[0]) < 0) q[0] = Negated(q[0]);
        for (std::size_t i = 1; i < n; ++i) {
          if (DotIsNegative(q[i], q[i - 1])) q[i] = Negated(q[i]);
        }
      }
      // Rule 6, and rule 11's keyframe: the table is replaced, not consulted.
      if (!keyframe && table[k]) {
        bool same = true;
        for (std::size_t i = 0; i < n && same; ++i) {
          for (int c = 0; c < slot.channels; ++c) same = same && q[i][c] == (*table[k])[c];
        }
        if (same) continue;
      }
      masks[static_cast<std::size_t>(slot.joint)] =
          static_cast<char>(masks[static_cast<std::size_t>(slot.joint)] | slot.bit);
      // Channel-major: all N of channel 0, then channel 1; the first absolute, then deltas.
      for (int c = 0; c < slot.channels; ++c) {
        PutI32(&groups, q[0][c]);
        for (std::size_t i = 1; i < n; ++i) {
          const std::int64_t d = q[i][c] - q[i - 1][c];
          if (!FitsInt32(d)) return false;
          PutI32(&groups, d);
        }
      }
      (*entries)[k] = q[n - 1];
    }
    body += masks;
    body += groups;

    // Rule 8 and rule 11: the events inside the piece, and on a keyframe the value in effect
    // at sample 0 when no event of its own is there.
    std::vector<std::pair<std::int32_t, int>> ev;
    int in_effect = enabled_before_open;
    for (const auto& e : events) {
      if (static_cast<std::size_t>(e.first) < lo) in_effect = e.second;
      if (static_cast<std::size_t>(e.first) >= lo && static_cast<std::size_t>(e.first) < hi) {
        ev.emplace_back(static_cast<std::int32_t>(e.first - static_cast<std::int32_t>(lo)),
                        e.second);
      }
    }
    if (keyframe && (ev.empty() || ev.front().first != 0)) ev.insert(ev.begin(), {0, in_effect});
    PutI32(&body, static_cast<std::int64_t>(ev.size()));
    for (const auto& e : ev) {
      PutI32(&body, e.first);
      PutI32(&body, e.second);
    }

    // Rule 10: one zstd frame, level 3, content checksum and content size both on. The
    // one-shot ZSTD_compress writes no checksum, and decoders reject a frame without one.
    std::string out;
    out.resize(4 + ZSTD_compressBound(body.size()));
    out[0] = static_cast<char>(kMagic);
    out[1] = static_cast<char>(kVersion);
    out[2] = static_cast<char>(kCodecZstd);
    out[3] = static_cast<char>(keyframe ? kFlagKeyframe : 0);
    ZSTD_CCtx_reset(cctx, ZSTD_reset_session_and_parameters);
    if (ZSTD_isError(ZSTD_CCtx_setParameter(cctx, ZSTD_c_compressionLevel, kZstdLevel)) ||
        ZSTD_isError(ZSTD_CCtx_setParameter(cctx, ZSTD_c_checksumFlag, 1)) ||
        ZSTD_isError(ZSTD_CCtx_setParameter(cctx, ZSTD_c_contentSizeFlag, 1))) {
      return false;
    }
    const std::size_t written =
        ZSTD_compress2(cctx, &out[4], out.size() - 4, body.data(), body.size());
    if (ZSTD_isError(written)) return false;
    out.resize(4 + written);
    *blob = std::move(out);
    return true;
  }

  // Closes the open block: encodes it, splitting at the midpoint while a piece is over
  // kMaxBlockBytes (rule 1), and commits each piece's state only once it is final (rule 9).
  std::vector<EncodedBlock> CloseOpen() {
    std::vector<EncodedBlock> out;
    if (t_us.empty()) return out;
    std::vector<std::pair<std::size_t, std::size_t>> todo{{0, open_n()}};
    while (!todo.empty()) {
      const auto [lo, hi] = todo.back();
      todo.pop_back();
      // Rule 11, decided on this piece's own start: the first block after a manifest send,
      // the object's first block, or the first in a later period than the previous block.
      const std::int64_t period = FloorDiv(t_us[lo], kKeyframePeriodUs);
      // Only the piece that starts the block can be first after a send: the flag clears once
      // that piece is sent, and the pieces after it are encoded after it.
      const bool after_send = first_after_send;
      const bool keyframe = after_send || !prev_period || period > *prev_period;
      std::string blob;
      std::vector<std::optional<Vals>> entries;
      if (!EncodePiece(lo, hi, keyframe, &blob, &entries)) {
        ++failures;
        break;
      }
      if (static_cast<std::int64_t>(blob.size()) > kMaxBlockBytes) {
        // Not sent, so nothing about it is recorded: the table, the period and the
        // first-after-send flag stay as they were, and each half is decided afresh.
        if (hi - lo == 1) {
          ++failures;
          break;
        }
        const std::size_t mid = lo + (hi - lo) / 2;
        todo.emplace_back(mid, hi);
        todo.emplace_back(lo, mid);
        continue;
      }
      // A keyframe wrote every group, so its entries replace the whole table (rule 11).
      for (std::size_t k = 0; k < slots.size(); ++k) {
        if (entries[k]) table[k] = entries[k];
      }
      prev_period = period;
      if (after_send) first_after_send = false;
      EncodedBlock b;
      b.seq = next_seq++;
      b.start_us = t_us[lo];
      b.last_us = t_us[hi - 1];
      b.sample_count = static_cast<std::int32_t>(hi - lo);
      b.keyframe = keyframe;
      b.reset_before = after_send;
      b.data = std::move(blob);
      out.push_back(std::move(b));
    }
    enabled_before_open = enabled_in_effect;
    ClearOpen();
    return out;
  }
};

BlockEncoder::BlockEncoder(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
BlockEncoder::~BlockEncoder() = default;

std::unique_ptr<BlockEncoder> BlockEncoder::Create(const EncoderOptions& options,
                                                   std::string* error_out) {
  auto fail = [&](const std::string& why) -> std::unique_ptr<BlockEncoder> {
    if (error_out) *error_out = why;
    return nullptr;
  };
  const std::size_t joints = options.joint_groups.size();
  if (joints < 1) return fail("an object needs at least one joint (the root)");
  // J, its masks and the per-sample bytes have to fit a 2 MiB body for even one sample.
  if (joints > 100'000) return fail("more than 100000 joints");
  const Steps& s = options.steps;
  if (s.pos_step <= 0) return fail("pos_step must be > 0");
  if (s.rot_step <= 0 || s.rot_step > kMaxRotStep) return fail("rot_step must be 1 to 100000000");
  if (s.scale_step <= 0) return fail("scale_step must be > 0");
  if (options.block_interval_us < kMinBlockIntervalUs ||
      options.block_interval_us > kMaxBlockIntervalUs) {
    return fail("block_interval_us must be 100000 to 10000000 (0.1 to 10 s)");
  }

  auto impl = std::make_unique<Impl>();
  impl->joint_count = static_cast<std::int32_t>(joints);
  impl->steps = s;
  impl->interval_us = options.block_interval_us;
  std::int64_t channels = 0;
  for (std::size_t j = 0; j < joints; ++j) {
    const std::uint8_t g = options.joint_groups[j];
    if (g & ~(kPosition | kRotation | kScale)) {
      return fail("joint " + std::to_string(j) + " names a group other than position, rotation, scale");
    }
    const auto joint = static_cast<std::int32_t>(j);
    if (g & kPosition) impl->slots.push_back({joint, kPosition, 3, s.pos_step * kNano});
    if (g & kRotation) impl->slots.push_back({joint, kRotation, 4, s.rot_step * kNano});
    if (g & kScale) impl->slots.push_back({joint, kScale, 3, s.scale_step * kNano});
  }
  for (const Slot& slot : impl->slots) channels += slot.channels;
  impl->n_max = MaxSamplesPerBlock(impl->joint_count, channels);
  impl->vals.resize(impl->slots.size());
  impl->chosen_rot.resize(impl->slots.size());
  impl->last_rot.resize(impl->slots.size());
  impl->table.resize(impl->slots.size());
  impl->cctx = ZSTD_createCCtx();
  if (!impl->cctx) return fail("could not create a zstd context");
  return std::unique_ptr<BlockEncoder>(new BlockEncoder(std::move(impl)));
}

bool BlockEncoder::AddSample(const PoseSample& sample, std::vector<EncodedBlock>* closed,
                             std::string* error_out) {
  Impl& m = *impl_;
  auto refuse = [&](const std::string& why) {
    if (error_out) *error_out = why;
    return false;
  };
  if (sample.joints.size() != static_cast<std::size_t>(m.joint_count)) {
    return refuse("sample has " + std::to_string(sample.joints.size()) + " joints, the object " +
                  std::to_string(m.joint_count));
  }
  if (m.last_t_us && sample.t_us < *m.last_t_us) {
    return refuse("sample time " + std::to_string(sample.t_us) + " us is before the previous " +
                  std::to_string(*m.last_t_us) + " us");
  }

  // Quantise everything first; nothing is committed until every value is known to fit.
  std::vector<Vals> q(m.slots.size(), Vals{0, 0, 0, 0});
  std::uint64_t repairs = 0;
  for (std::size_t k = 0; k < m.slots.size(); ++k) {
    const Slot& slot = m.slots[k];
    const JointPose& jp = sample.joints[static_cast<std::size_t>(slot.joint)];
    const Vec3& v = slot.bit == kPosition ? jp.position : jp.scale;
    const double in[4] = {slot.bit == kRotation ? jp.rotation.x : v.x,
                          slot.bit == kRotation ? jp.rotation.y : v.y,
                          slot.bit == kRotation ? jp.rotation.z : v.z, jp.rotation.w};
    for (int c = 0; c < slot.channels; ++c) {
      if (!Quantize(in[c], slot.step, &q[k][c])) {
        return refuse("joint " + std::to_string(slot.joint) + " " +
                      (slot.bit == kPosition ? "position" : slot.bit == kRotation ? "rotation" : "scale") +
                      " value " + std::to_string(in[c]) +
                      " is not finite or does not fit int32 at the configured step");
      }
    }
    if (slot.bit == kRotation && q[k] == Vals{0, 0, 0, 0}) {
      // Rule 5: a rotation that quantises to all zeros (C#'s default(Quaternion)) is not a
      // rotation, and decoders reject one.
      if (m.last_rot[k]) {
        q[k] = *m.last_rot[k];
      } else {
        std::int64_t w = 0;
        Quantize(1.0, slot.step, &w);
        q[k] = Vals{0, 0, 0, w};
      }
      ++repairs;
    }
  }

  // Rule 1: joins the open block while within its interval and under N_max.
  const bool joins = !m.t_us.empty() &&
                     static_cast<std::uint64_t>(sample.t_us) - static_cast<std::uint64_t>(m.t_us.front()) <
                         static_cast<std::uint64_t>(m.interval_us) &&
                     m.open_n() < static_cast<std::size_t>(m.n_max);

  // The deltas this sample would write, so that a jump the format cannot carry is refused
  // here instead of failing the whole block later. Rotations are compared after the sign
  // rule, as they will be written.
  std::vector<Vals> chosen(m.slots.size());
  for (std::size_t k = 0; k < m.slots.size(); ++k) {
    const Slot& slot = m.slots[k];
    if (slot.bit == kRotation) {
      if (!joins) {
        chosen[k] = CanonicalSign(q[k]) < 0 ? Negated(q[k]) : q[k];
      } else {
        chosen[k] = DotIsNegative(q[k], m.chosen_rot[k]) ? Negated(q[k]) : q[k];
      }
    } else {
      chosen[k] = q[k];
    }
    if (!joins) continue;
    const Vals& prev = slot.bit == kRotation ? m.chosen_rot[k] : m.vals[k].back();
    for (int c = 0; c < slot.channels; ++c) {
      if (!FitsInt32(chosen[k][c] - prev[c])) {
        return refuse("joint " + std::to_string(slot.joint) +
                      " moved further in one sample than an int32 delta carries at the configured step");
      }
    }
  }

  if (!joins && !m.t_us.empty()) {
    std::vector<EncodedBlock> done = m.CloseOpen();
    if (closed) {
      for (auto& b : done) closed->push_back(std::move(b));
    }
  }
  if (m.t_us.empty()) m.enabled_before_open = m.enabled_in_effect;
  const auto index = static_cast<std::int32_t>(m.t_us.size());
  m.t_us.push_back(sample.t_us);
  for (std::size_t k = 0; k < m.slots.size(); ++k) {
    m.vals[k].push_back(q[k]);
    if (m.slots[k].bit == kRotation) {
      m.chosen_rot[k] = chosen[k];
      m.last_rot[k] = q[k];
    }
  }
  if (sample.enabled) {
    m.events.emplace_back(index, *sample.enabled ? 1 : 0);
    m.enabled_in_effect = *sample.enabled ? 1 : 0;
  }
  m.last_t_us = sample.t_us;
  m.repaired += repairs;
  return true;
}

std::vector<EncodedBlock> BlockEncoder::Close() { return impl_->CloseOpen(); }

std::vector<EncodedBlock> BlockEncoder::ManifestSent() {
  std::vector<EncodedBlock> out = impl_->CloseOpen();
  impl_->first_after_send = true;
  impl_->first_seq = impl_->next_seq;
  return out;
}

std::int64_t BlockEncoder::first_seq() const { return impl_->first_seq; }
bool BlockEncoder::has_open_block() const { return !impl_->t_us.empty(); }
std::int32_t BlockEncoder::max_samples_per_block() const { return impl_->n_max; }
std::uint64_t BlockEncoder::repaired_rotations() const { return impl_->repaired; }
std::uint64_t BlockEncoder::encode_failures() const { return impl_->failures; }

}  // namespace pose
}  // namespace c3d
