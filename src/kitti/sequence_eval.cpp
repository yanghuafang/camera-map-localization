/// Eval summary rollups: matching quality rates and frame-timing percentiles.
#include <cam_loc/kitti/sequence_eval.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace cam_loc::kitti {

namespace {

double percentile(std::vector<double> values, double p) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const double rank = p * static_cast<double>(values.size() - 1);
  const size_t lo = static_cast<size_t>(std::floor(rank));
  const size_t hi = static_cast<size_t>(std::ceil(rank));
  const double t = rank - static_cast<double>(lo);
  return values[lo] * (1.0 - t) + values[hi] * t;
}

}  // namespace

MapMatchingQuality summarizeMatchingQuality(const std::vector<FrameEvalRecord>& records) {
  MapMatchingQuality q;
  q.num_frames = static_cast<int>(records.size());
  if (records.empty()) return q;

  double sum_cost = 0.0;
  double sum_spread = 0.0;
  int matches = 0;
  int flat = 0;
  int synth = 0;
  for (const auto& r : records) {
    sum_cost += r.min_cost;
    sum_spread += r.cost_spread;
    if (r.sampling_applied) ++matches;
    if (r.cost_map_flat) ++flat;
    if (r.perception_synthesized) ++synth;
  }
  const double n = static_cast<double>(records.size());
  q.mean_min_cost = sum_cost / n;
  q.mean_cost_spread = sum_spread / n;
  q.match_rate = static_cast<double>(matches) / n;
  q.flat_rate = static_cast<double>(flat) / n;
  q.synthesized_rate = static_cast<double>(synth) / n;
  return q;
}

SequenceEvalSummary summarizeEval(const std::vector<FrameEvalRecord>& records) {
  SequenceEvalSummary out;
  std::vector<TrajectoryError> pose_errors;
  pose_errors.reserve(records.size());
  std::vector<double> frame_ms;
  frame_ms.reserve(records.size());
  for (const auto& r : records) {
    pose_errors.push_back(r.pose_error);
    frame_ms.push_back(r.frame_ms);
  }
  out.pose = summarizeErrors(pose_errors);
  out.matching = summarizeMatchingQuality(records);
  if (!frame_ms.empty()) {
    double sum = 0.0;
    for (double t : frame_ms) sum += t;
    out.mean_frame_ms = sum / static_cast<double>(frame_ms.size());
    out.p95_frame_ms = percentile(frame_ms, 0.95);
  }
  return out;
}

}  // namespace cam_loc::kitti
