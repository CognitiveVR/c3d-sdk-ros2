// The pose stream's block encoder: one rigged object's samples in, version 1 pose blocks out.
//
// The format is the platform's, not this SDK's: "Pose block format, version 1" in
// c3d-stream-formatter's docs/pose/spec.md, whose §2 is the layout and §3 the encoding rules
// implemented here. Its conformance vectors are the acceptance test; tools/README.md says how
// to run them against this encoder.
//
// ROS-free and clock-free, like the rest of `core/`. Sample times are taken as given, in
// integer microseconds since the Unix epoch: which clock stamps a sample, and converting it
// to an integer exactly once, are the caller's. So is the spec's host-clock close (rule 1:
// a block closes once its interval has passed since its first sample was TAKEN, measured on
// a clock that never steps): the caller watches that clock and calls Close().
//
// No Cap'n Proto here either. A block is opaque bytes, and the upload envelope that carries
// it is built elsewhere.
#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "c3d/types.hpp"

namespace c3d {
namespace pose {

// A joint's groups, as the block's per-joint mask byte spells them.
constexpr std::uint8_t kPosition = 0x01;
constexpr std::uint8_t kRotation = 0x02;
constexpr std::uint8_t kScale = 0x04;

// Block intervals the spec allows, and its default. The interval is fixed per session and
// travels as the session property `c3d.stream.pose.blockIntervalMs`, never in a block.
constexpr std::int64_t kMinBlockIntervalUs = 100'000;
constexpr std::int64_t kMaxBlockIntervalUs = 10'000'000;
constexpr std::int64_t kDefaultBlockIntervalUs = 5'000'000;

// Keyframe periods are fixed to the epoch clock: a block's period is floor(start_us / K).
constexpr std::int64_t kKeyframePeriodUs = 30'000'000;

// The worst-case uncompressed body an encoder may produce; MaxSamplesPerBlock keeps every
// block under it.
constexpr std::int64_t kMaxEncoderBodyBytes = 2'097'152;
// A finished block (prefix + compressed body) at most this long: the ingest gateway's
// per-block limit. A longer one is split at its midpoint until every piece fits.
constexpr std::int64_t kMaxBlockBytes = 983'616;

// Quantisation steps, as the body header stores them: integer counts of 1e-9 units, so
// every language computes the same step (a float 0.0001 is inexact in binary).
//
// `pos_step` is in nanometres. Every quantised value must fit int32, so the default 0.1 mm
// covers +-214.7 km from the reference frame's origin; a map in UTM-scale coordinates needs
// a coarser step, and every decoder takes any step the header names.
struct Steps {
  std::int32_t pos_step = 100'000;    // > 0
  std::int32_t rot_step = 100'000;    // 1 .. 100'000'000 (0.1, so a unit quaternion is never all zeros)
  std::int32_t scale_step = 100'000;  // > 0
};

struct EncoderOptions {
  // One entry per joint, in manifest order, index 0 the root: the groups that joint carries
  // (kPosition | kRotation | kScale). Its size is the block's J, which must equal the
  // manifest's joint count. Fixed for the encoder's life: a rig whose joints change is a new
  // manifest and a new encoder.
  std::vector<std::uint8_t> joint_groups;
  Steps steps;
  std::int64_t block_interval_us = kDefaultBlockIntervalUs;
};

// One joint in one sample. Only the groups the joint carries are read. Index 0 is in the
// world (the session's reference frame); every other joint is local to its parent.
struct JointPose {
  Vec3 position;
  Quat rotation;
  Vec3 scale{1.0, 1.0, 1.0};
};

// One sample of the whole object: every joint, dense, in manifest order.
struct PoseSample {
  // Microseconds since the Unix epoch. Never decreasing from one sample to the next.
  std::int64_t t_us = 0;
  std::vector<JointPose> joints;
  // Set on the sample at which the object's visibility CHANGES, with its new value; every
  // set value is written as an event. Before the first one the object counts as visible.
  // The encoder restates the value in effect wherever the spec requires it (at the first
  // sample after a manifest send and at sample 0 of every keyframe), so a caller never
  // repeats it.
  std::optional<bool> enabled;
};

// One finished block, ready for the upload envelope.
struct EncodedBlock {
  // 0, 1, 2, ... per object, split pieces included: the envelope's `seq`.
  std::int64_t seq = 0;
  std::int64_t start_us = 0;  // sample 0's time: the envelope's `startUs`
  std::int64_t last_us = 0;   // the last sample's time, for the part's `endUs`
  std::int32_t sample_count = 0;
  // Prefix flag bit 0: decodes with no prior state. The first block after every manifest
  // send is one, and so is the object's first block in each 30 s period.
  bool keyframe = false;
  // The first block after a manifest send: seq == that send's firstSeq. The envelope's
  // `resetBefore`.
  bool reset_before = false;
  // Prefix (C3 01 01 flags) + one zstd frame, level 3, with content checksum and size.
  std::string data;
};

// The largest sample count whose body cannot pass kMaxEncoderBodyBytes, for J joints whose
// groups total `channel_count` channels (3 per position or scale, 4 per rotation), with
// every group written and an event on every sample:
//
//   floor((2,097,152 - 28 - J - 4) / (12 + 4 * channel_count)), at least 1.
//
// A count, never a measured body size: an encoder that closes on its real body size cuts
// blocks differently, and readers depend on where blocks are cut only through the vectors
// that pin it.
std::int32_t MaxSamplesPerBlock(std::int32_t joint_count, std::int64_t channel_count);

// One object's encode pipeline: the open block, the "last value sent" table, the keyframe
// period, the repaired-rotation count and the `seq` counter. Blocks of one object depend
// on each other through all of these, so one object has exactly one encoder and its blocks
// are encoded one at a time, in close order. Not thread-safe: the caller serialises calls.
//
// Creating the encoder is the object's first manifest send: its first block is a keyframe
// with seq 0, and first_seq() is 0.
//
// Closing is synchronous: every call that closes a block returns it, encoded, before it
// returns, so first_seq() is always known when a manifest is about to be sent.
class BlockEncoder {
 public:
  // Null, with the reason in *error_out, for options the format cannot carry: no joints, a
  // group bit beyond scale, a step out of range, an interval outside 0.1 to 10 s.
  static std::unique_ptr<BlockEncoder> Create(const EncoderOptions& options,
                                              std::string* error_out = nullptr);
  ~BlockEncoder();
  BlockEncoder(const BlockEncoder&) = delete;
  BlockEncoder& operator=(const BlockEncoder&) = delete;

  // Adds a sample to the open block, first closing that block if the sample is at or past
  // its interval from the block's first sample, or would take it past MaxSamplesPerBlock.
  // Blocks that closed are appended to *closed.
  //
  // Refused, with the reason in *error_out and nothing changed (no block closed, nothing
  // counted), when the sample cannot be encoded: a joint count other than J, a time before
  // the previous sample's, a value that is not finite, or a value or a delta from the
  // previous sample that does not fit int32 at the configured step (+-214.7 km at the
  // default pos_step). A value is never wrapped.
  //
  // A rotation that quantises to all zeros is not a rotation; it is replaced by the joint's
  // previous rotation, or identity if there is none, and counted in repaired_rotations().
  bool AddSample(const PoseSample& sample, std::vector<EncodedBlock>* closed,
                 std::string* error_out = nullptr);

  // Closes and encodes the open block, if there is one. Call it on session end, scene
  // change, pause, quit and object removal, and when the block's interval has passed on the
  // caller's monotonic clock since its first sample was taken.
  std::vector<EncodedBlock> Close();

  // Call when this object's manifest is sent again. Closes the open block (returned, still
  // under the previous send), then makes the next block a keyframe that writes every group
  // and restates `enabled`. first_seq() is that block's seq.
  std::vector<EncodedBlock> ManifestSent();

  // The seq of the first block after the latest manifest send: the send's `firstSeq`.
  std::int64_t first_seq() const;
  bool has_open_block() const;
  std::int32_t max_samples_per_block() const;
  // All-zero rotations replaced since creation: the session property
  // `c3d.stream.pose.repairedRotations` sums them over objects. The spec asks for one
  // warning per object, so a caller logs when this first leaves 0.
  std::uint64_t repaired_rotations() const;
  // Blocks that could not be encoded and were dropped: compression failed, or a single
  // sample came out over kMaxBlockBytes. Not reachable with sane inputs; counted so that
  // it is visible if it ever happens. A dropped block changes no encoder state, so the
  // blocks after it still decode.
  std::uint64_t encode_failures() const;

 private:
  struct Impl;
  explicit BlockEncoder(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace pose
}  // namespace c3d
