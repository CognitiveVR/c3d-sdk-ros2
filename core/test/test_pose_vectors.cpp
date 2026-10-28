// The pose encoder against c3d-stream-formatter's conformance vectors (spec §7).
//
// The vectors are the platform's and are not in this repository: these tests read them from
// the directory the C3D_POSE_VECTORS environment variable names, or else the C3D_POSE_VECTORS
// CMake cache variable, and skip when neither names one, so a checkout without the formatter
// stays green. tools/README.md shows the run and the reference checker's own.
//
// Stricter than the spec's encoder pass rule, which compares decoded values within half a
// step: this encoder ports the reference's every decision, so its uncompressed bodies must
// equal the reference's byte for byte. Compressed bytes are never compared.

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <set>
#include <string>
#include <sys/stat.h>
#include <vector>

#include "c3d/pose.hpp"
#include "pose_test_decoder.hpp"
#include "pose_test_recipes.hpp"
#include "pose_vector_input.hpp"

using c3d::pose::EncodedBlock;
using c3d::pose_test::Decoder;
using c3d::pose_test::DecodedBlock;
using c3d::pose_test::Recipes;
using c3d::pose_test::Vals;
using nlohmann::json;

namespace {

std::string VectorsDir() {
  const char* env = std::getenv("C3D_POSE_VECTORS");
  std::string dir = env && *env ? env : C3D_POSE_VECTORS_DEFAULT;
  struct stat st {};
  if (dir.empty() || stat(dir.c_str(), &st) != 0 || !S_ISDIR(st.st_mode)) return "";
  return dir;
}

#define REQUIRE_VECTORS(dir)                                                                  \
  const std::string dir = VectorsDir();                                                       \
  if (dir.empty()) {                                                                          \
    GTEST_SKIP() << "set C3D_POSE_VECTORS to c3d-stream-formatter's vectors/pose to run this"; \
  }

bool IsFile(const std::string& p) {
  struct stat st {};
  return stat(p.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}

std::string ReadFile(const std::string& p) {
  std::ifstream f(p, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
}

json ReadJson(const std::string& p) { return json::parse(ReadFile(p)); }

int FailureCount() {
  const auto* r = ::testing::UnitTest::GetInstance()->current_test_info()->result();
  int n = 0;
  for (int i = 0; i < r->total_part_count(); ++i) n += r->GetTestPartResult(i).failed() ? 1 : 0;
  return n;
}

std::string Hex(const std::string& b) {
  static const char* d = "0123456789abcdef";
  std::string out;
  for (unsigned char c : b) {
    out.push_back(d[c >> 4]);
    out.push_back(d[c & 15]);
  }
  return out;
}

// Every <name>/ with an input.json and an expected.json: the stored vectors, the four cut from
// dev sessions included when present. bad/, codec_variants/, generated/ and queue/ are not
// encoder inputs.
std::vector<std::string> StoredVectors(const std::string& dir) {
  std::vector<std::string> out;
  DIR* d = opendir(dir.c_str());
  if (!d) return out;
  while (dirent* e = readdir(d)) {
    const std::string name = e->d_name;
    if (name.empty() || name[0] == '.') continue;
    if (IsFile(dir + "/" + name + "/input.json") && IsFile(dir + "/" + name + "/expected.json")) {
      out.push_back(name);
    }
  }
  closedir(d);
  std::sort(out.begin(), out.end());
  return out;
}

std::string GroupName(const c3d::pose_test::GroupKey& k) {
  return std::to_string(k.first) + "," + std::string(1, k.second);
}

// One stored vector, checked block by block against its expected.json.
void CheckStoredVector(const std::string& dir, const std::string& name) {
  SCOPED_TRACE(name);
  const json input = ReadJson(dir + "/" + name + "/input.json");
  const json expected = ReadJson(dir + "/" + name + "/expected.json");
  std::vector<EncodedBlock> blocks;
  std::string error;
  ASSERT_TRUE(c3d::pose::EncodeVectorInput(input, &blocks, &error)) << error;
  const json& eblocks = expected.at("blocks");
  ASSERT_EQ(blocks.size(), eblocks.size());

  std::set<std::size_t> resets;
  for (const auto& r : expected.value("resets_at_block", json::array())) resets.insert(r.get<std::size_t>());
  Decoder decoder(input.at("joint_count").get<std::int32_t>());
  std::int64_t first_seq = 0;
  for (std::size_t k = 0; k < blocks.size(); ++k) {
    SCOPED_TRACE("block " + std::to_string(k));
    const EncodedBlock& b = blocks[k];
    const json& eb = eblocks[k];
    if (resets.count(k)) {
      decoder.Reset();
      first_seq = static_cast<std::int64_t>(k);
    }
    // The expected prefix is the reference's codec-none copy: magic, version and flags must
    // match; the codec byte is this encoder's, zstd.
    const std::string want_prefix = eb.at("prefix_hex").get<std::string>();
    EXPECT_EQ(Hex(b.data.substr(0, 2)), want_prefix.substr(0, 4));
    EXPECT_EQ(static_cast<unsigned char>(b.data[2]), 1);
    EXPECT_EQ(Hex(b.data.substr(3, 1)), want_prefix.substr(6, 2));

    const DecodedBlock d = decoder.Decode(b.data);
    ASSERT_EQ(d.error, "");
    EXPECT_EQ(Hex(d.body), eb.at("body_hex").get<std::string>());
    EXPECT_EQ(d.t_us, eb.at("t_us").get<std::vector<std::int64_t>>());
    std::map<std::string, std::vector<std::vector<std::int64_t>>> ints;
    for (const auto& [key, v] : d.ints) {
      const int c = key.second == 'r' ? 4 : 3;
      auto& rows = ints[GroupName(key)];
      for (const Vals& q : v) rows.emplace_back(q.begin(), q.begin() + c);
    }
    EXPECT_EQ(ints, (eb.at("ints").get<std::map<std::string, std::vector<std::vector<std::int64_t>>>>()));
    std::vector<std::vector<std::int32_t>> events;
    for (const auto& e : d.events) events.push_back({e.first, e.second});
    EXPECT_EQ(events, (eb.at("events").get<std::vector<std::vector<std::int32_t>>>()));

    // What the upload envelope will take from the encoder rather than from the bytes.
    EXPECT_EQ(b.seq, static_cast<std::int64_t>(k));
    EXPECT_EQ(b.start_us, eb.at("start_us").get<std::int64_t>());
    EXPECT_EQ(b.sample_count, eb.at("n").get<std::int32_t>());
    EXPECT_EQ(b.last_us, d.t_us.back());
    EXPECT_EQ(b.keyframe, d.keyframe);
    EXPECT_EQ(b.reset_before, b.seq == first_seq);
    EXPECT_LE(static_cast<std::int64_t>(b.data.size()), c3d::pose::kMaxBlockBytes);
  }
}

TEST(PoseVectors, EveryStoredVectorEncodesToTheReferenceBodies) {
  REQUIRE_VECTORS(dir);
  const std::vector<std::string> names = StoredVectors(dir);
  // The synthetic three are always there; the session-derived four are checked when present.
  for (const char* must : {"toy", "edge", "keyframes"}) {
    EXPECT_NE(std::find(names.begin(), names.end(), must), names.end()) << must << " missing from " << dir;
  }
  for (const auto& name : names) {
    const int before = FailureCount();
    CheckStoredVector(dir, name);
    std::cout << "[ pose vector ] " << name << (FailureCount() > before ? " FAILED" : " passed") << "\n";
  }
}

TEST(PoseVectors, EveryGeneratedRecipeCutsTheExpectedBlocks) {
  REQUIRE_VECTORS(dir);
  const json expected = ReadJson(dir + "/generated/expected.json");
  ASSERT_FALSE(expected.empty());
  for (const auto& item : expected.items()) {
    const std::string& name = item.key();
    SCOPED_TRACE("generated/" + name);
    const int before = FailureCount();
    const auto it = Recipes().find(name);
    // A recipe the formatter adds and pose_test_recipes.hpp does not build fails here rather
    // than passing unchecked.
    ASSERT_NE(it, Recipes().end()) << "a recipe this test does not build";
    const auto r = c3d::pose_test::RunRecipe(it->second());
    ASSERT_EQ(r.error, "");
    EXPECT_EQ(r.encode_failures, 0u);
    std::vector<std::int32_t> ns;
    std::vector<bool> keyframes;
    for (const auto& d : r.decoded) {
      ns.push_back(d.n);
      keyframes.push_back(d.keyframe);
    }
    for (const auto& b : r.blocks) EXPECT_LE(static_cast<std::int64_t>(b.data.size()), c3d::pose::kMaxBlockBytes);
    EXPECT_EQ(ns, item.value().at("expected_n_per_block").get<std::vector<std::int32_t>>());
    EXPECT_EQ(keyframes, item.value().at("expected_keyframes").get<std::vector<bool>>());
    EXPECT_LE(r.worst_steps, 0.5 + 1e-6);
    std::cout << "[ pose vector ] generated/" << name << (FailureCount() > before ? " FAILED" : " passed")
              << "\n";
  }
}

// --- bad/ and codec_variants/: decoder vectors, run here against the test decoder that
// judges the encoder above, so that it is known to reject what the platform rejects ------

TEST(PoseVectors, TheTestDecoderRejectsEveryBadBlockItCanRead) {
  REQUIRE_VECTORS(dir);
  const json expected = ReadJson(dir + "/bad/expected.json");
  int checked = 0, skipped = 0;
  for (const auto& item : expected.items()) {
    SCOPED_TRACE("bad/" + item.key());
    const json& e = item.value();
    const std::string blob = ReadFile(dir + "/bad/" + item.key() + ".bin");
    if (blob.size() >= 4 && (blob[2] == 2 || blob[2] == 3)) {
      ++skipped;  // lz4 or gzip: neither is linked into core/, and this SDK writes neither
      continue;
    }
    std::vector<std::string> want;
    if (e.contains("error_any")) {
      want = e.at("error_any").get<std::vector<std::string>>();
    } else {
      want.push_back(e.at("error").get<std::string>());
    }
    Decoder decoder(e.at("joint_count").get<std::int32_t>());
    for (const auto& prior : e.value("after", json::array())) {
      ASSERT_EQ(decoder.Decode(ReadFile(dir + "/bad/" + prior.get<std::string>())).error, "");
    }
    const std::string got = decoder.Decode(blob).error;
    EXPECT_NE(std::find(want.begin(), want.end(), got), want.end()) << "got '" << got << "'";
    ++checked;
  }
  std::cout << "[ pose vector ] bad: " << checked << " cases checked, " << skipped
            << " skipped (lz4 or gzip)\n";
  EXPECT_GT(checked, 0);
}

TEST(PoseVectors, TheTestDecoderReadsEveryCodecVariantItCanRead) {
  REQUIRE_VECTORS(dir);
  const json expected = ReadJson(dir + "/codec_variants/expected.json");
  int checked = 0, skipped = 0;
  for (const auto& item : expected.items()) {
    SCOPED_TRACE("codec_variants/" + item.key());
    const std::string blob = ReadFile(dir + "/codec_variants/" + item.key() + ".bin");
    if (blob.size() >= 4 && (blob[2] == 2 || blob[2] == 3)) {
      ++skipped;
      continue;
    }
    Decoder decoder(item.value().at("joint_count").get<std::int32_t>());
    const DecodedBlock d = decoder.Decode(blob);
    EXPECT_EQ(d.error, "");
    EXPECT_EQ(Hex(d.body), item.value().at("block").at("body_hex").get<std::string>());
    ++checked;
  }
  std::cout << "[ pose vector ] codec_variants: " << checked << " checked, " << skipped
            << " skipped (lz4 or gzip)\n";
  if (checked == 0) GTEST_SKIP() << "every codec variant is lz4 or gzip, which core/ does not link";
}

}  // namespace
