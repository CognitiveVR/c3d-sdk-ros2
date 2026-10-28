// The pose upload queue (spec §10.1 rules 3 to 7), on a clock the test owns.
//
// Two halves. The per-rule tests run in every build. PoseQueueVectors drives the queue through
// c3d-stream-formatter's vectors/pose/queue/ cases, the files every SDK's queue must pass, from
// the directory C3D_POSE_VECTORS names (or the CMake cache variable of that name), and skips
// without one, as the encoder's vectors do: they are not in this repository.

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <iterator>
#include <set>
#include <string>
#include <sys/stat.h>
#include <vector>

#include "pose_queue.hpp"

using c3d::pose::FlushTrigger;
using c3d::pose::QueuedBlock;
using c3d::pose::Send;
using c3d::pose::UploadQueue;
using nlohmann::json;

namespace {

constexpr std::int64_t kS = 1'000'000;
constexpr std::uint64_t kCap = 5'242'880;

QueuedBlock Block(const std::string& obj, std::int64_t seq, std::uint64_t bytes,
                  std::int64_t start_us = 1'790'000'000'000'000, std::int64_t last_us = 0) {
  QueuedBlock b;
  b.manifest_id = obj;
  b.seq = seq;
  b.bytes = bytes;
  b.start_us = start_us;
  b.last_us = last_us ? last_us : start_us + 4 * kS;
  return b;
}

using IdList = std::vector<std::pair<std::string, std::int64_t>>;

IdList Ids(const Send& s) {
  IdList out;
  for (const auto& b : s.blocks) out.emplace_back(b.manifest_id, b.seq);
  return out;
}

void Append(std::vector<Send>* all, std::vector<Send> more) {
  for (auto& s : more) all->push_back(std::move(s));
}

}  // namespace

// Rule 1 and 2 of the vectors' README: due at the first block's entry + 11 s, inclusive, and
// checked on a tick.
TEST(PoseQueue, TheTimerSendsAtElevenSecondsFromTheFirstArrivalInclusive) {
  UploadQueue q;
  EXPECT_TRUE(q.Arrive(100 * kS, Block("a", 0, 10)).empty());
  EXPECT_TRUE(q.Arrive(104 * kS, Block("a", 1, 10)).empty());
  EXPECT_TRUE(q.Tick(111 * kS - 1).empty());
  const auto sends = q.Tick(111 * kS);
  ASSERT_EQ(sends.size(), 1u);
  EXPECT_EQ(sends[0].reason, "timer");
  EXPECT_EQ(sends[0].part, 1u);
  EXPECT_EQ(sends[0].queue_start_us, 100 * kS);
  EXPECT_EQ(sends[0].sent_us, 111 * kS);
  EXPECT_EQ(sends[0].flush, 0);
  EXPECT_EQ(Ids(sends[0]), (IdList{{"a", 0}, {"a", 1}}));
}

// The entry time is the Arrive's clock, not anything the block carries: two blocks with
// identical sample times entering 5 s apart start the deadline at the first ENTRY.
TEST(PoseQueue, TheDeadlineCountsFromArrivalNotFromSampleTimes) {
  UploadQueue q;
  q.Arrive(200 * kS, Block("a", 0, 10, 1 * kS, 2 * kS));
  EXPECT_TRUE(q.Tick(210 * kS).empty());
  EXPECT_EQ(q.Tick(211 * kS).size(), 1u);
}

// Rule 3: on an arrival at or after the deadline, the timer fires first and the arriving
// block starts the next queue.
TEST(PoseQueue, ABlockArrivingAtTheDeadlineStartsTheNextQueue) {
  UploadQueue q;
  q.Arrive(0, Block("a", 0, 10));
  const auto sends = q.Arrive(11 * kS, Block("a", 1, 10));
  ASSERT_EQ(sends.size(), 1u);
  EXPECT_EQ(sends[0].reason, "timer");
  EXPECT_EQ(Ids(sends[0]), (IdList{{"a", 0}}));
  const auto next = q.Tick(22 * kS);
  ASSERT_EQ(next.size(), 1u);
  EXPECT_EQ(next[0].queue_start_us, 11 * kS);
  EXPECT_EQ(Ids(next[0]), (IdList{{"a", 1}}));
}

// Rule 4: exactly the cap fits; one byte more sends the queue first, and the block that did
// not fit starts the next queue at the same clock reading.
TEST(PoseQueue, TheCapIsInclusiveAndSendsBeforeAdding) {
  UploadQueue q;
  q.Arrive(0, Block("a", 0, kCap - 100));
  EXPECT_TRUE(q.Arrive(1 * kS, Block("b", 0, 100)).empty()) << "exactly the cap fits";
  UploadQueue r;
  r.Arrive(0, Block("a", 0, kCap - 100));
  const auto sends = r.Arrive(1 * kS, Block("b", 0, 101));
  ASSERT_EQ(sends.size(), 1u);
  EXPECT_EQ(sends[0].reason, "cap");
  EXPECT_EQ(sends[0].sent_us, 1 * kS);
  EXPECT_EQ(Ids(sends[0]), (IdList{{"a", 0}}));
  const auto next = r.Tick(12 * kS);
  ASSERT_EQ(next.size(), 1u);
  EXPECT_EQ(next[0].queue_start_us, 1 * kS) << "a cap send's reading starts the next queue";
}

// Rule 3: when the timer and the cap would both fire, the send is a timer send, and the
// block that would not have fitted joins nothing it did not fit with.
TEST(PoseQueue, TimerBeforeCap) {
  UploadQueue q;
  q.Arrive(0, Block("a", 0, kCap));
  const auto sends = q.Arrive(11 * kS, Block("a", 1, kCap));
  ASSERT_EQ(sends.size(), 1u);
  EXPECT_EQ(sends[0].reason, "timer");
}

// Rule 5 and the README's rule 5: a manifest part takes the next number and touches nothing,
// not even a due queue.
TEST(PoseQueue, AManifestPartTakesTheNextNumberAndChecksNoTimer) {
  UploadQueue q;
  EXPECT_EQ(q.ManifestPart(0).part, 1u);
  q.Arrive(1 * kS, Block("a", 0, 10));
  const Send m = q.ManifestPart(20 * kS);
  EXPECT_EQ(m.part, 2u);
  EXPECT_EQ(m.kind, Send::Kind::kManifests);
  EXPECT_EQ(m.start_us, 0);
  EXPECT_EQ(m.end_us, 0);
  EXPECT_FALSE(q.empty()) << "a manifest send is not a timer check";
  const auto sends = q.Tick(20 * kS);
  ASSERT_EQ(sends.size(), 1u);
  EXPECT_EQ(sends[0].part, 3u);
}

// Rule 7: the earliest block start and the latest LAST SAMPLE, not the last block's.
TEST(PoseQueue, EnvelopeTimesSpanTheEarliestStartAndTheLatestSample) {
  UploadQueue q;
  q.Arrive(0, Block("a", 0, 10, 5000, 9000));
  q.Arrive(1, Block("b", 0, 10, 4000, 12000));
  q.Arrive(2, Block("c", 0, 10, 6000, 7000));
  const auto sends = q.Tick(11 * kS);
  ASSERT_EQ(sends.size(), 1u);
  EXPECT_EQ(sends[0].start_us, 4000);
  EXPECT_EQ(sends[0].end_us, 12000);
}

// Rule 5: the queue as it is goes first, as a flush part even when it was due; the flush's
// blocks are its own batch, split by the cap into flush parts, the rest at FinishFlush.
TEST(PoseQueue, AFlushSendsTheQueueAsItIsThenItsOwnBlocksInCapPieces) {
  UploadQueue q;
  q.Arrive(0, Block("a", 0, 10));
  std::vector<Send> sends;
  ASSERT_TRUE(q.StartFlush(12 * kS, FlushTrigger::kEnd, 1, &sends));
  ASSERT_EQ(sends.size(), 1u);
  EXPECT_EQ(sends[0].reason, "flush:end") << "a due queue flushed is not a timer send";
  EXPECT_EQ(sends[0].flush, 1);
  EXPECT_TRUE(q.empty());
  Append(&sends, q.Arrive(12 * kS + 1, Block("a", 1, 3'000'000), 1));
  Append(&sends, q.Arrive(12 * kS + 2, Block("b", 0, 3'000'000), 1));
  ASSERT_EQ(sends.size(), 2u);
  EXPECT_EQ(sends[1].reason, "flush:end") << "a piece the cap splits off is still a flush part";
  EXPECT_EQ(sends[1].flush, 1);
  EXPECT_EQ(sends[1].queue_start_us, 12 * kS + 1);
  EXPECT_EQ(Ids(sends[1]), (IdList{{"a", 1}}));
  Append(&sends, q.FinishFlush(12 * kS + 3, 1));
  ASSERT_EQ(sends.size(), 3u);
  EXPECT_EQ(sends[2].queue_start_us, 12 * kS + 2);
  EXPECT_EQ(sends[2].sent_us, 12 * kS + 3);
  EXPECT_EQ(Ids(sends[2]), (IdList{{"b", 0}}));
  EXPECT_EQ(sends[2].part, 3u);
}

TEST(PoseQueue, AnEmptyFlushSendsNothingAndANumberIsNeverReused) {
  UploadQueue q;
  std::vector<Send> sends;
  ASSERT_TRUE(q.StartFlush(0, FlushTrigger::kPause, 1, &sends));
  Append(&sends, q.FinishFlush(1, 1));
  EXPECT_TRUE(sends.empty());
  EXPECT_FALSE(q.StartFlush(2, FlushTrigger::kEnd, 1, &sends));
  EXPECT_FALSE(q.StartFlush(2, FlushTrigger::kEnd, 0, &sends));
  EXPECT_EQ(q.parts_sent(), 0u);
}

// Overlapping flushes: the second's queue-as-is part holds only what was queued when it
// started, never the first flush's blocks, and each finishes on its own.
TEST(PoseQueue, OverlappingFlushesEachSendOnlyTheirOwnBlocks) {
  UploadQueue q;
  std::vector<Send> sends;
  ASSERT_TRUE(q.StartFlush(0, FlushTrigger::kPause, 1, &sends));
  Append(&sends, q.Arrive(1, Block("a", 0, 10), 1));
  Append(&sends, q.Arrive(2, Block("x", 0, 10)));
  ASSERT_TRUE(q.StartFlush(3, FlushTrigger::kEnd, 2, &sends));
  Append(&sends, q.Arrive(4, Block("b", 0, 10), 2));
  Append(&sends, q.FinishFlush(5, 2));
  Append(&sends, q.FinishFlush(6, 1));
  ASSERT_EQ(sends.size(), 3u);
  EXPECT_EQ(Ids(sends[0]), (IdList{{"x", 0}}));
  EXPECT_EQ(sends[0].flush, 2);
  EXPECT_EQ(sends[0].reason, "flush:end");
  EXPECT_EQ(Ids(sends[1]), (IdList{{"b", 0}}));
  EXPECT_EQ(sends[1].flush, 2);
  EXPECT_EQ(Ids(sends[2]), (IdList{{"a", 0}}));
  EXPECT_EQ(sends[2].flush, 1);
  EXPECT_EQ(sends[2].reason, "flush:pause");
}

// ---- The platform's queue vectors ---------------------------------------------------------

namespace {

std::string VectorsDir() {
  const char* env = std::getenv("C3D_POSE_VECTORS");
  std::string dir = env && *env ? env : C3D_POSE_VECTORS_DEFAULT;
  struct stat st {};
  if (dir.empty() || stat(dir.c_str(), &st) != 0 || !S_ISDIR(st.st_mode)) return "";
  return dir;
}

json ReadJson(const std::string& p) {
  std::ifstream f(p, std::ios::binary);
  return json::parse(std::string(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()));
}

FlushTrigger Trigger(const std::string& name) {
  if (name == "scene") return FlushTrigger::kScene;
  if (name == "pause") return FlushTrigger::kPause;
  return FlushTrigger::kEnd;
}

// A send as expected.json spells it, with `blocks` and `objs` as sets: the pass rule.
json Canonical(json send) {
  for (const char* key : {"blocks", "objs"}) {
    if (!send.contains(key)) continue;
    std::multiset<std::string> set;
    for (const auto& v : send[key]) set.insert(v.dump());
    send[key] = json(std::vector<std::string>(set.begin(), set.end()));
  }
  return send;
}

json AsJson(const Send& s, const json& objs) {
  json out;
  out["part"] = s.part;
  if (s.kind == Send::Kind::kManifests) {
    out["kind"] = "manifests";
    out["objs"] = objs;
  } else {
    out["kind"] = "blocks";
    json blocks = json::array();
    for (const auto& b : s.blocks) blocks.push_back(json::array({b.manifest_id, b.seq}));
    out["blocks"] = blocks;
    out["queueStartUs"] = s.queue_start_us;
    out["sentUs"] = s.sent_us;
    out["reason"] = s.reason;
    if (s.flush > 0) out["flush"] = s.flush;
  }
  out["startUs"] = s.start_us;
  out["endUs"] = s.end_us;
  return out;
}

// The README's harness: every call carries its own `t`, and the sends are what the queue
// handed back, in order.
std::vector<json> RunCase(const json& input) {
  UploadQueue q(input.at("timerUs").get<std::int64_t>(), input.at("capBytes").get<std::uint64_t>());
  std::vector<json> out;
  auto take = [&](const std::vector<Send>& sends) {
    for (const Send& s : sends) out.push_back(AsJson(s, json()));
  };
  for (const json& e : input.at("events")) {
    const std::string call = e.at("call");
    const std::int64_t t = e.at("t");
    if (call == "arrive") {
      const json& b = e.at("block");
      QueuedBlock block;
      block.manifest_id = b.at("obj");
      block.seq = b.at("seq");
      block.bytes = b.at("bytes");
      block.sample_count = b.at("sampleCount");
      block.start_us = b.at("startUs");
      block.last_us = b.at("lastUs");
      take(q.Arrive(t, block, e.contains("flush") ? e.at("flush").get<int>() : 0));
    } else if (call == "tick") {
      take(q.Tick(t));
    } else if (call == "manifest") {
      out.push_back(AsJson(q.ManifestPart(t), e.at("objs")));
    } else if (call == "flush") {
      std::vector<Send> sends;
      EXPECT_TRUE(q.StartFlush(t, Trigger(e.at("trigger")), e.at("k"), &sends));
      take(sends);
    } else if (call == "flushDone") {
      take(q.FinishFlush(t, e.at("k")));
    } else {
      ADD_FAILURE() << "unknown call " << call;
    }
  }
  return out;
}

}  // namespace

TEST(PoseQueueVectors, EveryQueueCasePasses) {
  const std::string root = VectorsDir();
  if (root.empty()) {
    GTEST_SKIP() << "set C3D_POSE_VECTORS to c3d-stream-formatter's vectors/pose to run this";
  }
  const std::string dir = root + "/queue";
  DIR* d = opendir(dir.c_str());
  ASSERT_NE(d, nullptr) << dir << " is not a directory";
  std::set<std::string> cases;
  while (const dirent* entry = readdir(d)) {
    const std::string name = entry->d_name;
    struct stat st {};
    if (name[0] != '.' && stat((dir + "/" + name + "/input.json").c_str(), &st) == 0) {
      cases.insert(name);
    }
  }
  closedir(d);
  ASSERT_FALSE(cases.empty()) << "no case under " << dir;
  for (const std::string& name : cases) {
    SCOPED_TRACE(name);
    const std::vector<json> got = RunCase(ReadJson(dir + "/" + name + "/input.json"));
    const json want = ReadJson(dir + "/" + name + "/expected.json").at("sends");
    EXPECT_EQ(got.size(), want.size()) << "sends";
    for (std::size_t i = 0; i < std::min(got.size(), want.size()); ++i) {
      EXPECT_EQ(Canonical(got[i]), Canonical(want[i])) << "send " << i + 1;
    }
  }
  RecordProperty("cases", static_cast<int>(cases.size()));
  std::cout << "[ queue    ] " << cases.size() << " queue vector cases" << std::endl;
}
