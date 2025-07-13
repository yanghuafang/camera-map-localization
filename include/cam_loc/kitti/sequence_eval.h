#ifndef CAM_LOC_KITTI_SEQUENCE_EVAL_H_
#define CAM_LOC_KITTI_SEQUENCE_EVAL_H_

/// Sequence evaluation types and summary aggregation.
///
/// Consumed by eval_sequence, eval_perception_compare, and the benchmark suite.

#include <string>
#include <vector>

#include "cam_loc/kitti/eval_metrics.h"
#include "cam_loc/perception/noise.h"
#include "cam_loc/perception/resolve.h"
#include "cam_loc/types/params.h"

namespace cam_loc::kitti {

/// Map-matching health rates aggregated over a sequence.
struct MapMatchingQuality {
  double mean_min_cost = 0.0;
  double mean_cost_spread = 0.0;
  double match_rate = 0.0;
  double flat_rate = 0.0;
  /// Frames where the best hypothesis fit too badly to use.
  double poor_fit_rate = 0.0;
  double synthesized_rate = 0.0;
  int num_frames = 0;
};

/// Per-frame eval record: pose error, cost diagnostics, and timing.
struct FrameEvalRecord {
  int frame = 0;
  TrajectoryError pose_error;
  float min_cost = 0.f;
  float cost_spread = 0.f;
  bool perception_synthesized = false;
  bool cost_map_flat = false;
  bool match_cost_too_high = false;
  bool sampling_applied = false;
  double best_offset_m = 0.0;
  bool loaded_from_file = false;
  bool noise_applied = false;
  double frame_ms = 0.0;
};

/// Rolled-up pose, matching, and latency statistics for one eval run.
struct SequenceEvalSummary {
  ErrorSummary pose;
  MapMatchingQuality matching;
  double mean_frame_ms = 0.0;
  double p95_frame_ms = 0.0;
};

/// CLI / benchmark inputs for RunSequenceEval.
struct SequenceEvalConfig {
  std::string kitti_root = ".";
  std::string perception_root;
  perception::PerceptionSource perception_source =
      perception::PerceptionSource::kAuto;
  perception::PerceptionNoiseParams noise;
  uint32_t noise_seed = 1;
  int sequence = 0;
  int start_frame = 0;
  int max_frames = -1;
  LocalizationParams localization;
};

/// Aggregate map-matching rates from per-frame eval records.
MapMatchingQuality SummarizeMatchingQuality(
    const std::vector<FrameEvalRecord>& records);

/// Combine pose errors, matching quality, and frame timing into one summary.
SequenceEvalSummary SummarizeEval(const std::vector<FrameEvalRecord>& records);

}  // namespace cam_loc::kitti

#endif  // CAM_LOC_KITTI_SEQUENCE_EVAL_H_
