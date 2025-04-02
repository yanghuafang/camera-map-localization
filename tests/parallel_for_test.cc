// Index fan-out: coverage, disjointness, and thread-count independence.

#include "cam_loc/core/parallel_for.h"

#include <algorithm>
#include <atomic>
#include <mutex>
#include <numeric>
#include <vector>

#include <gtest/gtest.h>

namespace {

using cam_loc::core::ParallelFor;

std::vector<int> VisitCounts(int count, int threads) {
  std::vector<std::atomic<int>> visits(static_cast<size_t>(count));
  for (auto& v : visits) v.store(0);
  ParallelFor(count, threads, [&](int begin, int end) {
    for (int i = begin; i < end; ++i) ++visits[static_cast<size_t>(i)];
  });
  std::vector<int> out(static_cast<size_t>(count));
  for (size_t i = 0; i < out.size(); ++i) out[i] = visits[i].load();
  return out;
}

TEST(ParallelForTest, EveryIndexIsVisitedExactlyOnce) {
  for (const int threads : {1, 2, 3, 7, 64}) {
    const auto visits = VisitCounts(100, threads);
    for (size_t i = 0; i < visits.size(); ++i) {
      EXPECT_EQ(visits[i], 1)
          << "index " << i << " with " << threads << " threads";
    }
  }
}

TEST(ParallelForTest, MoreThreadsThanWorkIsNotAnError) {
  const auto visits = VisitCounts(3, 32);
  EXPECT_EQ(std::accumulate(visits.begin(), visits.end(), 0), 3);
}

TEST(ParallelForTest, EmptyRangeRunsNothing) {
  int calls = 0;
  ParallelFor(0, 4, [&](int, int) { ++calls; });
  ParallelFor(-5, 4, [&](int, int) { ++calls; });
  EXPECT_EQ(calls, 0);
}

TEST(ParallelForTest, OneThreadRunsInline) {
  // No thread is spawned, which is what keeps a debugger on the caller's stack.
  const auto self = std::this_thread::get_id();
  std::thread::id seen;
  ParallelFor(10, 1, [&](int, int) { seen = std::this_thread::get_id(); });
  EXPECT_EQ(seen, self);
}

TEST(ParallelForTest, BlocksAreContiguousAndOrdered) {
  std::mutex m;
  std::vector<std::pair<int, int>> blocks;
  ParallelFor(10, 3, [&](int begin, int end) {
    const std::lock_guard<std::mutex> lock(m);
    blocks.emplace_back(begin, end);
  });
  std::sort(blocks.begin(), blocks.end());
  for (size_t i = 0; i + 1 < blocks.size(); ++i) {
    EXPECT_EQ(blocks[i].second, blocks[i + 1].first) << "gap or overlap";
  }
}

}  // namespace
