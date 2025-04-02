#ifndef CAM_LOC_CORE_PARALLEL_FOR_H_
#define CAM_LOC_CORE_PARALLEL_FOR_H_

/// Minimal index-range fan-out over std::thread.

#include <algorithm>
#include <functional>
#include <thread>
#include <vector>

namespace cam_loc::core {

/// Split `[0, count)` into contiguous blocks and run @p body on each in
/// parallel, one thread per block, joining before returning.
///
/// Blocks are contiguous and disjoint, so a body that writes only to indices in
/// its own range needs no synchronization and produces bitwise the same result
/// as the serial loop. That is what makes this safe for the pose grid: every
/// hypothesis owns one cell.
///
/// Threads are created per call rather than pooled. At one call per frame
/// against milliseconds of work per block that is not measurable; a pool would
/// be worth it only if this were called per grid cell.
///
/// @param count       Number of indices. Zero or negative does nothing.
/// @param num_threads Threads to use; 0 or less asks the hardware. Falls back
///                    to running inline when only one block results, which
///                    keeps single-threaded builds and debugging free of any
///                    thread at all.
inline void ParallelFor(int count, int num_threads,
                        const std::function<void(int begin, int end)>& body) {
  if (count <= 0) return;

  int threads = num_threads;
  if (threads <= 0) {
    threads = static_cast<int>(std::thread::hardware_concurrency());
  }
  threads = std::clamp(threads, 1, count);

  if (threads == 1) {
    body(0, count);
    return;
  }

  const int block = (count + threads - 1) / threads;
  std::vector<std::thread> workers;
  workers.reserve(static_cast<size_t>(threads) - 1);
  // The caller's thread takes the last block instead of idling.
  for (int t = 0; t + 1 < threads; ++t) {
    const int begin = t * block;
    const int end = std::min(begin + block, count);
    if (begin >= end) break;
    workers.emplace_back([&body, begin, end] { body(begin, end); });
  }
  const int tail_begin = std::min((threads - 1) * block, count);
  if (tail_begin < count) body(tail_begin, count);

  for (auto& w : workers) w.join();
}

}  // namespace cam_loc::core

#endif  // CAM_LOC_CORE_PARALLEL_FOR_H_
