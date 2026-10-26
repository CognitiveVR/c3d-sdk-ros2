// The pose block encoder, rule by rule (c3d-stream-formatter docs/pose/spec.md §2 and §3).
// Blocks are read back with pose_test_decoder.hpp, which is written from the spec's decoding
// rules and shares no code with the encoder. Every expected integer was computed outside this
// repository (Python, in float64 as the spec requires), never by the code under test.
//
// None of these needs the formatter's vectors: test_pose_vectors.cpp runs those when present.

#include <gtest/gtest.h>
#include <zstd.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "c3d/pose.hpp"
#include "pose_test_decoder.hpp"
#include "pose_test_recipes.hpp"

using c3d::pose::BlockEncoder;
using c3d::pose::EncodedBlock;
using c3d::pose::EncoderOptions;
using c3d::pose::JointPose;
using c3d::pose::PoseSample;
using c3d::pose::kPosition;
using c3d::pose::kRotation;
using c3d::pose::kScale;
using c3d::pose_test::DecodedBlock;
using c3d::pose_test::Decoder;
using c3d::pose_test::Vals;

namespace {

// 2026-09-21T14:13:20Z in microseconds: a real epoch time, past 2^32 in any unit.
constexpr std::int64_t kT0 = 1'790'000'000'000'000;
constexpr std::int64_t kSecond = 1'000'000;

EncoderOptions Options(std::vector<std::uint8_t> groups, std::int64_t interval_us = kSecond) {
  EncoderOptions o;
  o.joint_groups = std::move(groups);
  o.block_interval_us = interval_us;
  return o;
}

std::unique_ptr<BlockEncoder> Make(const EncoderOptions& o) {
  std::string error;
  auto e = BlockEncoder::Create(o, &error);
  EXPECT_TRUE(e) << error;
  return e;
}

JointPose P(double x, double y, double z) {
  JointPose j;
  j.position = {x, y, z};
  return j;
}

JointPose R(double x, double y, double z, double w) {
  JointPose j;
  j.rotation = {x, y, z, w};
  return j;
}

PoseSample S(std::int64_t t, std::vector<JointPose> joints, std::optional<bool> enabled = {}) {
  PoseSample s;
  s.t_us = t;
  s.joints = std::move(joints);
  s.enabled = enabled;
  return s;
}

void Add(BlockEncoder& e, const PoseSample& s, std::vector<EncodedBlock>* out) {
  std::string error;
  ASSERT_TRUE(e.AddSample(s, out, &error)) << error;
}

std::vector<EncodedBlock> Finish(BlockEncoder& e, std::vector<EncodedBlock> blocks) {
  for (auto& b : e.Close()) blocks.push_back(std::move(b));
  return blocks;
}

std::vector<DecodedBlock> DecodeAll(const std::vector<EncodedBlock>& blocks, std::int32_t joints) {
  Decoder d(joints);
  std::vector<DecodedBlock> out;
  for (const auto& b : blocks) {
    out.push_back(d.Decode(b.data));
    // Thrown, not only reported: the callers index into what a rejected block leaves empty,
    // and gtest turns the exception into this test's failure instead of a crash.
    if (!out.back().error.empty()) {
      throw std::runtime_error("block seq " + std::to_string(b.seq) + " rejected: " + out.back().error);
    }
  }
  return out;
}

std::int32_t BodyI32(const std::string& body, std::size_t offset) {
  std::uint32_t u = 0;
  for (int i = 0; i < 4; ++i) {
    u |= static_cast<std::uint32_t>(static_cast<unsigned char>(body[offset + i])) << (8 * i);
  }
  return static_cast<std::int32_t>(u);
}

Vals Q(std::int64_t x, std::int64_t y, std::int64_t z, std::int64_t w = 0) { return {x, y, z, w}; }

// --- §2: layout -------------------------------------------------------------------------

TEST(PoseBlockFormat, ThePrefixIsMagicVersionZstdAndTheKeyframeFlag) {
  auto e = Make(Options({kPosition}));
  std::vector<EncodedBlock> blocks;
  Add(*e, S(kT0, {P(1, 2, 3)}, true), &blocks);
  Add(*e, S(kT0 + kSecond, {P(1, 2, 4)}), &blocks);
  blocks = Finish(*e, blocks);
  ASSERT_EQ(blocks.size(), 2u);
  EXPECT_EQ(blocks[0].data.substr(0, 4), std::string("\xC3\x01\x01\x01", 4));
  EXPECT_EQ(blocks[1].data.substr(0, 4), std::string("\xC3\x01\x01\x00", 4));
}

TEST(PoseBlockFormat, EveryBlockIsOneZstdFrameWithAChecksumAndAContentSize) {
  auto e = Make(Options({kPosition | kRotation}));
  std::vector<EncodedBlock> blocks;
  for (int i = 0; i < 20; ++i) Add(*e, S(kT0 + i * 10'000, {P(i * 0.01, 0, 0)}), &blocks);
  blocks = Finish(*e, blocks);
  ASSERT_EQ(blocks.size(), 1u);
  const std::string payload = blocks[0].data.substr(4);
  ASSERT_GE(payload.size(), 5u);
  EXPECT_EQ(payload.substr(0, 4), std::string("\x28\xB5\x2F\xFD", 4));
  EXPECT_TRUE(static_cast<unsigned char>(payload[4]) & 0x04) << "no content checksum";
  const auto size = ZSTD_getFrameContentSize(payload.data(), payload.size());
  EXPECT_NE(size, ZSTD_CONTENTSIZE_UNKNOWN) << "no content size";
  EXPECT_NE(size, ZSTD_CONTENTSIZE_ERROR);
  EXPECT_EQ(ZSTD_findFrameCompressedSize(payload.data(), payload.size()), payload.size());
  // N 20, J 1, times 20, masks 1, root p and r, one event.
  EXPECT_EQ(size, 28u + 4 * 20 + 1 + 4 * 7 * 20 + 4 + 8);
}

TEST(PoseBlockFormat, TheHeaderCarriesNJAnInt64StartAndTheSteps) {
  EncoderOptions o = Options({kPosition, kRotation | kScale});
  o.steps = {250'000, 350'000, 120'000};
  auto e = Make(o);
  std::vector<EncodedBlock> blocks;
  Add(*e, S(kT0 + 123, {P(0, 0, 0), R(0, 0, 0, 1)}), &blocks);
  Add(*e, S(kT0 + 456, {P(0, 0, 0), R(0, 0, 0, 1)}), &blocks);
  blocks = Finish(*e, blocks);
  ASSERT_EQ(blocks.size(), 1u);
  const auto d = DecodeAll(blocks, 2);
  ASSERT_EQ(d[0].error, "");
  const std::string& body = d[0].body;
  EXPECT_EQ(BodyI32(body, 0), 2);
  EXPECT_EQ(BodyI32(body, 4), 2);
  EXPECT_EQ(d[0].start_us, kT0 + 123);
  EXPECT_EQ(BodyI32(body, 16), 250'000);
  EXPECT_EQ(BodyI32(body, 20), 350'000);
  EXPECT_EQ(BodyI32(body, 24), 120'000);
  EXPECT_EQ(blocks[0].start_us, kT0 + 123);
  EXPECT_EQ(blocks[0].last_us, kT0 + 456);
  EXPECT_EQ(blocks[0].sample_count, 2);
}

TEST(PoseBlockFormat, TimesAreZeroThenMicrosecondDeltas) {
  auto e = Make(Options({kPosition}));
  std::vector<EncodedBlock> blocks;
  for (std::int64_t t : {kT0, kT0 + 10'000, kT0 + 10'000, kT0 + 35'000}) Add(*e, S(t, {P(0, 0, 0)}), &blocks);
  blocks = Finish(*e, blocks);
  const auto d = DecodeAll(blocks, 1);
  ASSERT_EQ(d.size(), 1u);
  ASSERT_EQ(d[0].error, "");
  EXPECT_EQ(BodyI32(d[0].body, 28), 0);
  EXPECT_EQ(BodyI32(d[0].body, 32), 10'000);
  EXPECT_EQ(BodyI32(d[0].body, 36), 0);
  EXPECT_EQ(BodyI32(d[0].body, 40), 25'000);
}

TEST(PoseBlockFormat, EachGroupIsChannelMajorFirstAbsoluteThenDeltas) {
  auto e = Make(Options({kPosition}));
  std::vector<EncodedBlock> blocks;
  Add(*e, S(kT0, {P(0.0001, 0.0002, 0.0003)}), &blocks);
  Add(*e, S(kT0 + 1, {P(0.0003, 0.0002, 0.0001)}), &blocks);
  blocks = Finish(*e, blocks);
  const auto d = DecodeAll(blocks, 1);
  ASSERT_EQ(d[0].error, "");
  const std::string& body = d[0].body;
  const std::size_t groups = 28 + 4 * 2 + 1;
  EXPECT_EQ(static_cast<unsigned char>(body[groups - 1]), kPosition);
  const std::vector<std::int32_t> want = {1, 2, 2, 0, 3, -2};  // x: 1, +2; y: 2, +0; z: 3, -2
  for (std::size_t i = 0; i < want.size(); ++i) EXPECT_EQ(BodyI32(body, groups + 4 * i), want[i]) << i;
  EXPECT_EQ(BodyI32(body, groups + 24), 1);  // n_events
  EXPECT_EQ(body.size(), groups + 24 + 4 + 8);
}

TEST(PoseBlockFormat, MasksNameOnlyTheGroupsEachJointCarries) {
  // A JointPose always has a scale; only a joint configured with kScale writes one.
  auto e = Make(Options({kPosition | kRotation, kRotation, kPosition | kRotation | kScale}));
  std::vector<EncodedBlock> blocks;
  Add(*e, S(kT0, {P(1, 0, 0), R(0, 0, 0, 1), P(0.01, 0.02, 0.03)}), &blocks);
  blocks = Finish(*e, blocks);
  const auto d = DecodeAll(blocks, 3);
  EXPECT_EQ(d[0].masks, (std::vector<std::uint8_t>{3, 2, 7}));
  EXPECT_EQ(d[0].ints.count({0, 's'}), 0u);
  EXPECT_EQ(d[0].ints.at({2, 's'})[0], Q(10'000, 10'000, 10'000));
}

// --- §3 rule 4: quantisation ---------------------------------------------------------------

TEST(PoseBlockEncoder, QuantisesInDoubleWithTheStepInDouble) {
  // 0.00015 / 0.0001 is 1.4999999999999998 in double, so q = 1; float32 arithmetic gives 2.
  auto e = Make(Options({kPosition}));
  std::vector<EncodedBlock> blocks;
  Add(*e, S(kT0, {P(0.00015, -0.00015, 0.6)}), &blocks);
  Add(*e, S(kT0 + 1, {P(214748.36, -214748.36, 1e-5)}), &blocks);
  blocks = Finish(*e, blocks);
  const auto d = DecodeAll(blocks, 1);
  EXPECT_EQ(d[0].ints.at({0, 'p'})[0], Q(1, -1, 6'000));
  EXPECT_EQ(d[0].ints.at({0, 'p'})[1], Q(2'147'483'600, -2'147'483'600, 0));
}

TEST(PoseBlockEncoder, RefusesAValueOutsideInt32AtTheStepAndChangesNothing) {
  // 214,748.37 m is 2,147,483,700 steps of 0.1 mm: past int32. Refused, never wrapped.
  auto e = Make(Options({kPosition}));
  std::vector<EncodedBlock> blocks;
  Add(*e, S(kT0, {P(1, 2, 3)}), &blocks);
  std::string error;
  EXPECT_FALSE(e->AddSample(S(kT0 + 1, {P(214748.37, 0, 0)}), &blocks, &error));
  EXPECT_NE(error.find("int32"), std::string::npos) << error;
  EXPECT_FALSE(e->AddSample(S(kT0 + 1, {P(0, -214748.37, 0)}), &blocks, &error));
  EXPECT_FALSE(e->AddSample(S(kT0 + 1, {P(0, 0, std::nan(""))}), &blocks, &error));
  EXPECT_FALSE(e->AddSample(S(kT0 + 1, {P(std::numeric_limits<double>::infinity(), 0, 0)}), &blocks, &error));
  // Nothing was added: the block still holds its one sample.
  blocks = Finish(*e, blocks);
  ASSERT_EQ(blocks.size(), 1u);
  EXPECT_EQ(blocks[0].sample_count, 1);

  // At a 1 mm step the same coordinate fits.
  EncoderOptions coarse = Options({kPosition});
  coarse.steps.pos_step = 1'000'000;
  auto e2 = Make(coarse);
  std::vector<EncodedBlock> b2;
  Add(*e2, S(kT0, {P(214748.37, 0, 0)}), &b2);
  b2 = Finish(*e2, b2);
  EXPECT_EQ(DecodeAll(b2, 1)[0].ints.at({0, 'p'})[0], Q(214'748'370, 0, 0));
}

TEST(PoseBlockEncoder, RefusesADeltaOutsideInt32ButNotTheSameJumpInANewBlock) {
  // -200 km and +200 km both fit int32 at 0.1 mm (-2e9, 2e9); the 4e9 delta does not.
  auto e = Make(Options({kPosition}));
  std::vector<EncodedBlock> blocks;
  Add(*e, S(kT0, {P(-200'000, 0, 0)}), &blocks);
  std::string error;
  EXPECT_FALSE(e->AddSample(S(kT0 + 1, {P(200'000, 0, 0)}), &blocks, &error));
  EXPECT_TRUE(blocks.empty()) << "a refused sample closed a block";
  // A block's first value is absolute, so the jump is fine across a block boundary.
  Add(*e, S(kT0 + kSecond, {P(200'000, 0, 0)}), &blocks);
  blocks = Finish(*e, blocks);
  const auto d = DecodeAll(blocks, 1);
  ASSERT_EQ(d.size(), 2u);
  EXPECT_EQ(d[0].ints.at({0, 'p'})[0], Q(-2'000'000'000, 0, 0));
  EXPECT_EQ(d[1].ints.at({0, 'p'})[0], Q(2'000'000'000, 0, 0));
}

TEST(PoseBlockEncoder, RefusesASampleOlderThanThePreviousOne) {
  auto e = Make(Options({kPosition}));
  std::vector<EncodedBlock> blocks;
  Add(*e, S(kT0 + 1'000, {P(0, 0, 0)}), &blocks);
  std::string error;
  EXPECT_FALSE(e->AddSample(S(kT0 + 999, {P(0, 0, 0)}), &blocks, &error));
  EXPECT_NE(error.find("before"), std::string::npos) << error;
  Add(*e, S(kT0 + 1'000, {P(0, 0, 0)}), &blocks);  // equal is fine: a delta of 0
  blocks = Finish(*e, blocks);
  ASSERT_EQ(blocks.size(), 1u);
  EXPECT_EQ(blocks[0].sample_count, 2);
}

TEST(PoseBlockEncoder, RefusesASampleWithTheWrongJointCount) {
  auto e = Make(Options({kPosition, kRotation}));
  std::vector<EncodedBlock> blocks;
  std::string error;
  EXPECT_FALSE(e->AddSample(S(kT0, {P(0, 0, 0)}), &blocks, &error));
  EXPECT_FALSE(e->has_open_block());
}

// --- §3 rule 5: quaternion sign and zero rotations ---------------------------------------

std::vector<Vals> Rotations(const std::vector<std::vector<double>>& quats, std::int64_t dt = 1) {
  auto e = Make(Options({kRotation}, 10 * kSecond));
  std::vector<EncodedBlock> blocks;
  std::int64_t t = kT0;
  for (const auto& q : quats) {
    Add(*e, S(t, {R(q[0], q[1], q[2], q[3])}), &blocks);
    t += dt;
  }
  blocks = Finish(*e, blocks);
  std::vector<Vals> out;
  for (const auto& d : DecodeAll(blocks, 1)) {
    for (const auto& v : d.ints.at({0, 'r'})) out.push_back(v);
  }
  return out;
}

TEST(PoseBlockEncoder, SampleZeroTakesTheSignThatMakesWPositive) {
  EXPECT_EQ(Rotations({{0, 0, -0.6, -0.8}})[0], Q(0, 0, 6'000, 8'000));
}

TEST(PoseBlockEncoder, WithWZeroTheFirstNonZeroOfXYZIsMadePositive) {
  EXPECT_EQ(Rotations({{-0.6, 0.8, 0, 0}})[0], Q(6'000, -8'000, 0, 0));
  EXPECT_EQ(Rotations({{0, -0.6, 0.8, 0}})[0], Q(0, 6'000, -8'000, 0));
  EXPECT_EQ(Rotations({{0, 0, -1, 0}})[0], Q(0, 0, 10'000, 0));
}

TEST(PoseBlockEncoder, ALaterSampleTakesTheSignNearerThePreviousChosenOne) {
  const auto r = Rotations({{0, 0, 0.6, 0.8}, {0, 0, -0.61, -0.79}, {0, 0, 0.62, 0.78}});
  EXPECT_EQ(r[0], Q(0, 0, 6'000, 8'000));
  EXPECT_EQ(r[1], Q(0, 0, 6'100, 7'900));
  EXPECT_EQ(r[2], Q(0, 0, 6'200, 7'800));
}

TEST(PoseBlockEncoder, ATieKeepsTheCapturedSign) {
  // (0,0,0,1) . (-1,0,0,0) = 0: neither sign is nearer, so the captured one stays.
  const auto r = Rotations({{0, 0, 0, 1}, {-1, 0, 0, 0}});
  EXPECT_EQ(r[1], Q(-10'000, 0, 0, 0));
}

TEST(PoseBlockEncoder, EachBlockChoosesSampleZerosSignAfresh) {
  // Block 1's only sample is block 0's last rotation negated: its own sample 0 is made
  // canonical, so it equals the table entry and is not written.
  auto e = Make(Options({kRotation}));
  std::vector<EncodedBlock> blocks;
  Add(*e, S(kT0, {R(0, 0, 0.6, 0.8)}), &blocks);
  Add(*e, S(kT0 + kSecond, {R(0, 0, -0.6, -0.8)}), &blocks);
  blocks = Finish(*e, blocks);
  const auto d = DecodeAll(blocks, 1);
  ASSERT_EQ(d.size(), 2u);
  EXPECT_FALSE(d[1].keyframe);
  EXPECT_EQ(d[1].masks[0], 0);
  EXPECT_EQ(d[1].ints.at({0, 'r'})[0], Q(0, 0, 6'000, 8'000));
}

TEST(PoseBlockEncoder, AZeroRotationTakesTheJointsPreviousRotation) {
  auto e = Make(Options({kRotation}));
  std::vector<EncodedBlock> blocks;
  Add(*e, S(kT0, {R(0, 0, -0.6, -0.8)}), &blocks);
  Add(*e, S(kT0 + 1, {R(0, 0, 0, 0)}), &blocks);
  // Quantises to all zeros at the default step (1e-5 is 0.1 of a step): also not a rotation.
  Add(*e, S(kT0 + 2, {R(1e-5, 0, 0, 0)}), &blocks);
  blocks = Finish(*e, blocks);
  const auto d = DecodeAll(blocks, 1);
  EXPECT_EQ(d[0].ints.at({0, 'r'})[1], Q(0, 0, 6'000, 8'000));
  EXPECT_EQ(d[0].ints.at({0, 'r'})[2], Q(0, 0, 6'000, 8'000));
  EXPECT_EQ(e->repaired_rotations(), 2u);
}

TEST(PoseBlockEncoder, AZeroRotationWithNoPreviousOneIsIdentityAtTheStep) {
  EncoderOptions o = Options({kRotation});
  o.steps.rot_step = 350'000;  // identity w = floor(1 / 0.00035 + 0.5) = 2857
  auto e = Make(o);
  std::vector<EncodedBlock> blocks;
  Add(*e, S(kT0, {R(0, 0, 0, 0)}), &blocks);
  blocks = Finish(*e, blocks);
  EXPECT_EQ(DecodeAll(blocks, 1)[0].ints.at({0, 'r'})[0], Q(0, 0, 0, 2'857));
  EXPECT_EQ(e->repaired_rotations(), 1u);
}

TEST(PoseBlockEncoder, AZeroRotationAtABlocksFirstSampleIsRepairedFromThePreviousBlock) {
  auto e = Make(Options({kRotation}));
  std::vector<EncodedBlock> blocks;
  Add(*e, S(kT0, {R(0, 0, 0.6, 0.8)}), &blocks);
  Add(*e, S(kT0 + kSecond, {R(0, 0, 0, 0)}), &blocks);
  blocks = Finish(*e, blocks);
  const auto d = DecodeAll(blocks, 1);
  ASSERT_EQ(d.size(), 2u);
  EXPECT_EQ(d[1].ints.at({0, 'r'})[0], Q(0, 0, 6'000, 8'000));
  EXPECT_EQ(e->repaired_rotations(), 1u);
}

TEST(PoseBlockEncoder, ARefusedSampleCountsNoRepair) {
  // The zero rotation comes first, so it is repaired before the position is refused.
  auto e = Make(Options({kRotation, kPosition}));
  std::vector<EncodedBlock> blocks;
  std::string error;
  EXPECT_FALSE(e->AddSample(S(kT0, {R(0, 0, 0, 0), P(1e9, 0, 0)}), &blocks, &error));
  EXPECT_EQ(e->repaired_rotations(), 0u);
}

// --- §3 rule 6: masks and the "last value sent" table -----------------------------------

TEST(PoseBlockEncoder, AGroupUnchangedForAWholeBlockIsNotWrittenAndCarriesOver) {
  auto e = Make(Options({kPosition | kRotation, kRotation}));
  std::vector<EncodedBlock> blocks;
  Add(*e, S(kT0, {P(1, 0, 0), R(0, 0, 0.6, 0.8)}), &blocks);
  Add(*e, S(kT0 + kSecond, {P(1, 0, 0), R(0, 0, 0.6, 0.8)}), &blocks);
  Add(*e, S(kT0 + kSecond + 1, {P(1.0001, 0, 0), R(0, 0, 0.6, 0.8)}), &blocks);
  blocks = Finish(*e, blocks);
  const auto d = DecodeAll(blocks, 2);
  ASSERT_EQ(d.size(), 2u);
  EXPECT_FALSE(d[1].keyframe);
  // Root position changed in one sample of the block, so all of it is written; root rotation
  // and joint 1 did not move.
  EXPECT_EQ(d[1].masks, (std::vector<std::uint8_t>{kPosition, 0}));
  EXPECT_EQ(d[1].ints.at({0, 'p'}), (std::vector<Vals>{Q(10'000, 0, 0), Q(10'001, 0, 0)}));
  EXPECT_EQ(d[1].ints.at({1, 'r'})[1], Q(0, 0, 6'000, 8'000));
}

TEST(PoseBlockEncoder, TheTableFollowsTheLastSampleOfEveryWrittenGroup) {
  // Block 1 moves and comes back; block 2 holds still at the block-1 end value, so only a
  // table that took block 1's LAST sample skips it.
  auto e = Make(Options({kPosition}));
  std::vector<EncodedBlock> blocks;
  Add(*e, S(kT0, {P(1, 0, 0)}), &blocks);
  Add(*e, S(kT0 + kSecond, {P(2, 0, 0)}), &blocks);
  Add(*e, S(kT0 + kSecond + 1, {P(3, 0, 0)}), &blocks);
  Add(*e, S(kT0 + 2 * kSecond, {P(3, 0, 0)}), &blocks);
  blocks = Finish(*e, blocks);
  const auto d = DecodeAll(blocks, 1);
  ASSERT_EQ(d.size(), 3u);
  EXPECT_EQ(d[1].masks[0], kPosition);
  EXPECT_EQ(d[2].masks[0], 0);
  EXPECT_EQ(d[2].ints.at({0, 'p'})[0], Q(30'000, 0, 0));
}

// --- §3 rule 1: windows and N_max -------------------------------------------------------

TEST(PoseBlockEncoder, ASampleJoinsWhileItIsWithinTheIntervalOfTheBlocksFirst) {
  auto e = Make(Options({kPosition}, 500'000));
  std::vector<EncodedBlock> blocks;
  for (std::int64_t t : {0, 499'999, 500'000, 999'999, 1'000'000}) Add(*e, S(kT0 + t, {P(0, 0, 0)}), &blocks);
  blocks = Finish(*e, blocks);
  ASSERT_EQ(blocks.size(), 3u);
  EXPECT_EQ(blocks[0].sample_count, 2);
  EXPECT_EQ(blocks[1].sample_count, 2);
  EXPECT_EQ(blocks[2].sample_count, 1);
  EXPECT_EQ(blocks[1].start_us, kT0 + 500'000);
}

TEST(PoseBlockEncoder, MaxSamplesPerBlockIsAFormulaOfJAndTheChannelCount) {
  EXPECT_EQ(c3d::pose::MaxSamplesPerBlock(240, 2'400), 218);
  EXPECT_EQ(c3d::pose::MaxSamplesPerBlock(11, 107), 4'766);
  EXPECT_EQ(c3d::pose::MaxSamplesPerBlock(1, 0), 174'759);
  EXPECT_EQ(c3d::pose::MaxSamplesPerBlock(60'000, 600'000), 1);
  auto e = Make(Options({kPosition | kRotation, kPosition | kRotation | kScale}));
  EXPECT_EQ(e->max_samples_per_block(), c3d::pose::MaxSamplesPerBlock(2, 17));
}

TEST(PoseBlockEncoder, ABlockClosesBeforeItsSampleCountPassesNMax) {
  // 240 joints with p, r, s: N_max = 218, so 219 samples well inside one interval cut 218 + 1.
  const auto recipe = c3d::pose_test::Constant(240, 219, 1'000, true);
  auto e = Make(Options(recipe.joint_groups, 10 * kSecond));
  std::vector<EncodedBlock> blocks;
  for (std::size_t i = 0; i < recipe.t_us.size(); ++i) Add(*e, S(recipe.t_us[i], recipe.samples[i]), &blocks);
  EXPECT_EQ(blocks.size(), 1u) << "the 219th sample closes the first block";
  blocks = Finish(*e, blocks);
  ASSERT_EQ(blocks.size(), 2u);
  EXPECT_EQ(blocks[0].sample_count, 218);
  EXPECT_EQ(blocks[1].sample_count, 1);
}

TEST(PoseBlockEncoder, CloseEncodesTheOpenBlockAndTheNextSampleStartsAnother) {
  auto e = Make(Options({kPosition}, 10 * kSecond));
  std::vector<EncodedBlock> blocks;
  Add(*e, S(kT0, {P(0, 0, 0)}), &blocks);
  Add(*e, S(kT0 + 1, {P(0, 0, 0)}), &blocks);
  EXPECT_TRUE(e->has_open_block());
  auto closed = e->Close();
  ASSERT_EQ(closed.size(), 1u);
  EXPECT_EQ(closed[0].sample_count, 2);
  EXPECT_FALSE(e->has_open_block());
  EXPECT_TRUE(e->Close().empty());
  Add(*e, S(kT0 + 2, {P(0, 0, 0)}), &blocks);
  blocks = Finish(*e, blocks);
  ASSERT_EQ(blocks.size(), 1u);
  EXPECT_EQ(blocks[0].seq, 1);
  EXPECT_EQ(blocks[0].start_us, kT0 + 2);
  EXPECT_FALSE(blocks[0].keyframe);
}

// --- §3 rules 1, 9 and 11: splitting an oversized block, with its state undone ----------

struct Cut {
  std::vector<std::int32_t> n;
  std::vector<bool> keyframes;
};

Cut CutOf(const c3d::pose_test::RecipeResult& r) {
  Cut c;
  for (const auto& b : r.blocks) {
    c.n.push_back(b.sample_count);
    c.keyframes.push_back(b.keyframe);
  }
  return c;
}

void ExpectSeqRunsFromZero(const std::vector<EncodedBlock>& blocks) {
  for (std::size_t k = 0; k < blocks.size(); ++k) {
    EXPECT_EQ(blocks[k].seq, static_cast<std::int64_t>(k));
    EXPECT_EQ(blocks[k].reset_before, k == 0);
    EXPECT_LE(static_cast<std::int64_t>(blocks[k].data.size()), c3d::pose::kMaxBlockBytes);
  }
}

TEST(PoseBlockSplit, AnOversizedBlockSplitsAtItsMidpointAndEachHalfDecidesItsKeyframe) {
  // spec §7 generated/split: 1,900 + 1,900, the second half past the 30 s mark.
  const auto r = c3d::pose_test::RunRecipe(c3d::pose_test::Split(29'100'000));
  ASSERT_EQ(r.error, "");
  EXPECT_EQ(CutOf(r).n, (std::vector<std::int32_t>{1'900, 1'900}));
  EXPECT_EQ(CutOf(r).keyframes, (std::vector<bool>{true, true}));
  EXPECT_LE(r.worst_steps, 0.5 + 1e-6);
  ExpectSeqRunsFromZero(r.blocks);
}

TEST(PoseBlockSplit, EachPieceCarriesItsOwnEventsAndAKeyframePieceRestatesTheValueInEffect) {
  // Hidden in the first half, shown again inside the second, which is a keyframe (past 30 s).
  auto recipe = c3d::pose_test::Split(29'100'000);
  recipe.enabled = {{0, true}, {1'000, false}, {2'000, true}};
  const auto r = c3d::pose_test::RunRecipe(recipe);
  ASSERT_EQ(r.error, "");
  ASSERT_EQ(r.decoded.size(), 2u);
  using Ev = std::vector<std::pair<std::int32_t, std::int32_t>>;
  EXPECT_EQ(r.decoded[0].events, (Ev{{0, 1}, {1'000, 0}}));
  EXPECT_EQ(r.decoded[1].events, (Ev{{0, 0}, {100, 1}}));
}

TEST(PoseBlockSplit, OnlyThePieceRightAfterAManifestSendIsKeyframedForIt) {
  // generated/split_manifest: both halves in one period.
  const auto r = c3d::pose_test::RunRecipe(c3d::pose_test::Split(1'000'000));
  ASSERT_EQ(r.error, "");
  EXPECT_EQ(CutOf(r).n, (std::vector<std::int32_t>{1'900, 1'900}));
  EXPECT_EQ(CutOf(r).keyframes, (std::vector<bool>{true, false}));
}

TEST(PoseBlockSplit, SplitsAgainUntilEveryPieceFitsAndUndoesEachOversizedAttempt) {
  // generated/split_deep: 5, then 2,150 whose halves are still too big. Joint 0 sits at 1 step
  // all through block 2: an encoder that kept the oversized attempt's table entry would skip
  // it in the first quarter, and it would decode at block 1's 2 steps.
  const auto r = c3d::pose_test::RunRecipe(c3d::pose_test::SplitDeep(1'000'000, 11'000'000));
  ASSERT_EQ(r.error, "");
  EXPECT_EQ(CutOf(r).n, (std::vector<std::int32_t>{5, 537, 538, 537, 538}));
  EXPECT_EQ(CutOf(r).keyframes, (std::vector<bool>{true, false, false, false, false}));
  ASSERT_EQ(r.decoded.size(), 5u);
  EXPECT_TRUE(r.decoded[1].written.at({0, 'p'}));
  EXPECT_EQ(r.decoded[1].ints.at({0, 'p'})[0], Q(1, 0, 0));
  EXPECT_LE(r.worst_steps, 0.5 + 1e-6);
  ExpectSeqRunsFromZero(r.blocks);
}

TEST(PoseBlockSplit, ThePeriodIsRecordedOnlyForPiecesThatWereSent) {
  // generated/split_period: block 2 starts in a new period, so its first quarter is a keyframe
  // and the other three are not.
  const auto r = c3d::pose_test::RunRecipe(c3d::pose_test::SplitDeep(25'000'000, 35'000'000));
  ASSERT_EQ(r.error, "");
  EXPECT_EQ(CutOf(r).n, (std::vector<std::int32_t>{5, 537, 538, 537, 538}));
  EXPECT_EQ(CutOf(r).keyframes, (std::vector<bool>{true, true, false, false, false}));
}

TEST(PoseBlockSplit, NMaxCutsTheGeneratedRecipesAsTheSpecSays) {
  const auto big = c3d::pose_test::RunRecipe(c3d::pose_test::Constant(240, 900, 11'111, true));
  ASSERT_EQ(big.error, "");
  EXPECT_EQ(CutOf(big).n, (std::vector<std::int32_t>{218, 218, 218, 218, 28}));
  EXPECT_LE(big.worst_steps, 0.5 + 1e-6);
  const auto small = c3d::pose_test::RunRecipe(c3d::pose_test::Constant(11, 10'000, 1'000, false));
  ASSERT_EQ(small.error, "");
  EXPECT_EQ(CutOf(small).n, (std::vector<std::int32_t>{4'766, 4'766, 468}));
}

// --- §3 rules 8 and 11: events and keyframes ---------------------------------------------

std::vector<std::vector<std::pair<std::int32_t, std::int32_t>>> EventsOf(const std::vector<DecodedBlock>& d) {
  std::vector<std::vector<std::pair<std::int32_t, std::int32_t>>> out;
  for (const auto& b : d) out.push_back(b.events);
  return out;
}

TEST(PoseBlockEncoder, EnabledIsWrittenWhereSetAndRestatedAtEveryKeyframe) {
  auto e = Make(Options({kPosition}));
  std::vector<EncodedBlock> blocks;
  const std::int64_t t0 = 30 * kSecond * 59'666'666;  // 1789999980 s: a 30 s mark
  Add(*e, S(t0, {P(0, 0, 0)}), &blocks);                          // block 0: keyframe, restates 1
  Add(*e, S(t0 + 500'000, {P(0, 0, 0)}, false), &blocks);         //   hidden at its sample 1
  Add(*e, S(t0 + kSecond, {P(0, 0, 0)}), &blocks);                // block 1: same period, no event
  Add(*e, S(t0 + 2 * kSecond, {P(0, 0, 0)}, true), &blocks);      // block 2: shown again
  Add(*e, S(t0 + 2 * kSecond + 1, {P(0, 0, 0)}, false), &blocks); //   and hidden
  Add(*e, S(t0 + 31 * kSecond, {P(0, 0, 0)}), &blocks);           // block 3: next period, restates 0
  Add(*e, S(t0 + 61 * kSecond, {P(0, 0, 0)}, true), &blocks);     // block 4: keyframe with its own event
  blocks = Finish(*e, blocks);
  const auto d = DecodeAll(blocks, 1);
  using Ev = std::vector<std::pair<std::int32_t, std::int32_t>>;
  EXPECT_EQ(EventsOf(d), (std::vector<Ev>{{{0, 1}, {1, 0}}, {}, {{0, 1}, {1, 0}}, {{0, 0}}, {{0, 1}}}));
  EXPECT_EQ(d[3].keyframe, true);
  EXPECT_EQ(d[4].keyframe, true);
}

TEST(PoseBlockEncoder, TheFirstBlockInALaterPeriodIsAKeyframeThatWritesEveryGroup) {
  auto e = Make(Options({kPosition | kRotation, kRotation}));
  std::vector<EncodedBlock> blocks;
  const std::int64_t mark = 30 * kSecond * 59'666'667;
  for (std::int64_t t : {mark - 2 * kSecond, mark - kSecond, mark + 100'000, mark + kSecond + 100'000}) {
    Add(*e, S(t, {P(1, 2, 3), R(0, 0, 0.6, 0.8)}), &blocks);
  }
  blocks = Finish(*e, blocks);
  const auto d = DecodeAll(blocks, 2);
  ASSERT_EQ(d.size(), 4u);
  EXPECT_EQ((std::vector<bool>{d[0].keyframe, d[1].keyframe, d[2].keyframe, d[3].keyframe}),
            (std::vector<bool>{true, false, true, false}));
  EXPECT_EQ(d[1].masks, (std::vector<std::uint8_t>{0, 0}));
  EXPECT_EQ(d[2].masks, (std::vector<std::uint8_t>{3, 2})) << "a keyframe writes what did not change";
  EXPECT_EQ(d[3].masks, (std::vector<std::uint8_t>{0, 0}));
  EXPECT_EQ(blocks[2].data[3], '\x01');
}

TEST(PoseBlockEncoder, KeyframePeriodsAreFlooredForTimesBeforeTheEpoch) {
  // floor(-1 s / 30 s) is -1, and 0.5 s is in period 0, so the second block is a keyframe;
  // truncating division would put both in period 0.
  auto e = Make(Options({kPosition}));
  std::vector<EncodedBlock> blocks;
  Add(*e, S(-kSecond, {P(0, 0, 0)}), &blocks);
  Add(*e, S(kSecond / 2, {P(0, 0, 0)}), &blocks);
  blocks = Finish(*e, blocks);
  ASSERT_EQ(blocks.size(), 2u);
  EXPECT_TRUE(blocks[1].keyframe);
}

// --- manifest sends: seq, firstSeq, resetBefore ------------------------------------------

TEST(PoseBlockEncoder, AManifestSendClosesTheBlockAndTheNextOneStartsOverAtFirstSeq) {
  auto e = Make(Options({kPosition, kRotation}, 10 * kSecond));
  EXPECT_EQ(e->first_seq(), 0);
  std::vector<EncodedBlock> blocks;
  Add(*e, S(kT0, {P(1, 0, 0), R(0, 0, 0, 1)}, true), &blocks);
  Add(*e, S(kT0 + 1, {P(1, 0, 0), R(0, 0, 0, 1)}, false), &blocks);
  auto sent = e->ManifestSent();
  ASSERT_EQ(sent.size(), 1u);
  EXPECT_EQ(sent[0].seq, 0);
  EXPECT_TRUE(sent[0].reset_before);
  EXPECT_EQ(e->first_seq(), 1);
  // Nothing moved, but the block after a send writes every group and restates enabled (0).
  Add(*e, S(kT0 + 2, {P(1, 0, 0), R(0, 0, 0, 1)}), &blocks);
  auto mid = e->Close();
  Add(*e, S(kT0 + 3, {P(1, 0, 0), R(0, 0, 0, 1)}), &blocks);
  auto last = e->Close();
  ASSERT_EQ(mid.size(), 1u);
  ASSERT_EQ(last.size(), 1u);
  EXPECT_EQ(mid[0].seq, 1);
  EXPECT_TRUE(mid[0].reset_before);
  EXPECT_TRUE(mid[0].keyframe);
  EXPECT_EQ(last[0].seq, 2);
  EXPECT_FALSE(last[0].reset_before);
  EXPECT_FALSE(last[0].keyframe);

  Decoder d(2);
  ASSERT_EQ(d.Decode(sent[0].data).error, "");
  d.Reset();
  const DecodedBlock after = d.Decode(mid[0].data);
  ASSERT_EQ(after.error, "");
  EXPECT_EQ(after.masks, (std::vector<std::uint8_t>{kPosition, kRotation}));
  EXPECT_EQ(after.events, (std::vector<std::pair<std::int32_t, std::int32_t>>{{0, 0}}));
}

TEST(PoseBlockEncoder, AManifestSendWithNoOpenBlockOnlyMovesFirstSeq) {
  auto e = Make(Options({kPosition}));
  std::vector<EncodedBlock> blocks;
  Add(*e, S(kT0, {P(0, 0, 0)}), &blocks);
  EXPECT_EQ(e->Close().size(), 1u);
  EXPECT_TRUE(e->ManifestSent().empty());
  EXPECT_EQ(e->first_seq(), 1);
  EXPECT_TRUE(e->ManifestSent().empty());
  EXPECT_EQ(e->first_seq(), 1);
}

// --- options ------------------------------------------------------------------------------

TEST(PoseBlockEncoder, RefusesOptionsTheFormatCannotCarry) {
  std::string error;
  EXPECT_FALSE(BlockEncoder::Create(Options({}), &error));
  EXPECT_FALSE(BlockEncoder::Create(Options({kPosition | 0x08}), &error));
  EXPECT_FALSE(BlockEncoder::Create(Options({kPosition}, 99'999), &error));
  EXPECT_FALSE(BlockEncoder::Create(Options({kPosition}, 10'000'001), &error));
  EXPECT_TRUE(BlockEncoder::Create(Options({kPosition}, 100'000), &error)) << error;
  EXPECT_TRUE(BlockEncoder::Create(Options({kPosition}, 10'000'000), &error)) << error;
  for (auto steps : {c3d::pose::Steps{0, 1, 1}, c3d::pose::Steps{1, 0, 1}, c3d::pose::Steps{1, 1, 0},
                     c3d::pose::Steps{1, 100'000'001, 1}, c3d::pose::Steps{-5, 1, 1}}) {
    EncoderOptions o = Options({kPosition});
    o.steps = steps;
    EXPECT_FALSE(BlockEncoder::Create(o, &error)) << steps.pos_step << " " << steps.rot_step;
  }
  EncoderOptions o = Options({kRotation});
  o.steps.rot_step = 100'000'000;
  EXPECT_TRUE(BlockEncoder::Create(o, &error)) << error;
  EXPECT_EQ(c3d::pose::kDefaultBlockIntervalUs, 5'000'000);
  EXPECT_EQ(EncoderOptions().block_interval_us, 5'000'000);
  EXPECT_EQ(EncoderOptions().steps.pos_step, 100'000);
}

}  // namespace
