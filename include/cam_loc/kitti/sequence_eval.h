#ifndef CAM_LOC_KITTI_SEQUENCE_EVAL_H_
#define CAM_LOC_KITTI_SEQUENCE_EVAL_H_

/// Sequence evaluation types and summary aggregation.
///
/// Consumed by eval_sequence, eval_perception_compare, and the benchmark suite.

#include <string>
#include <vector>

#include "cam_loc/kitti/egomotion_noise.h"
#include "cam_loc/kitti/eval_metrics.h"
#include "cam_loc/perception/noise.h"
#include "cam_loc/perception/resolve.h"
#include "cam_loc/types/params.h"

namespace cam_loc::kitti {

/// Map-matching health rates aggregated over a sequence.
struct MapMatchingQuality {
  double mean_min_cost = 0.0;
  double mean_cost_spread = 0.0;
  /// Mean gap from the winner to the next separated minimum, DT pixels.
  double mean_mode_margin = 0.0;
  /// Fraction of frames where a second minimum was found at all.
  double multimodal_rate = 0.0;
  /// Mean hypotheses scored per frame.
  double mean_cells_searched = 0.0;
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
  /// Cost gap from the winner to the next separated minimum, DT pixels.
  float mode_margin = 0.f;
  int num_cost_modes = 0;
  /// Hypotheses scored this frame.
  int cells_searched = 0;
  bool loaded_from_file = false;
  bool noise_applied = false;
  double frame_ms = 0.0;
  /// Whether the filter's own covariance explains this frame's pose error.
  FrameConsistency consistency;
};

/// Rolled-up pose, consistency, matching, and latency statistics for one eval
/// run.
struct SequenceEvalSummary {
  ErrorSummary pose;
  ConsistencySummary consistency;
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
  /// Odometry error injected into the prediction step. Off by default: the
  /// ground-truth egomotion every other mode assumes is the reference the
  /// map-matching numbers were measured against.
  EgomotionNoiseParams ego_noise;
  uint32_t ego_noise_seed = 2;
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
