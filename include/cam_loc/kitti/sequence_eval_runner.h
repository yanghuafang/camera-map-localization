#ifndef CAM_LOC_KITTI_SEQUENCE_EVAL_RUNNER_H_
#define CAM_LOC_KITTI_SEQUENCE_EVAL_RUNNER_H_

/// Frame-by-frame sequence evaluation driver.
///
/// Playback flow per frame: BuildEgomotion → ResolvePerception →
/// LocalizationEngine::ProcessFrame → record pose error and map-matching
/// diagnostics.

#include <memory>
#include <vector>

#include "cam_loc/kitti/sequence_eval.h"
#include "cam_loc/kitti/types.h"
#include "cam_loc/map/map_loader.h"
#include "cam_loc/types/status.h"

namespace cam_loc::kitti {

/// Run localization over a pose sequence and collect per-frame eval records.
///
/// @param config Frames `[start_frame, max_frames)` are processed. Note that
///        `max_frames` is an **end index**, not a count: with `start_frame=10`
///        and `max_frames=80` this runs 70 frames, and `max_frames` at or below
///        `start_frame` is an error rather than an empty run.
/// @param out_records Cleared, then one record per processed frame.
/// @return `kInvalidArgument` on an empty frame range, a missing map loader, or
///         any frame the engine rejects.
/// @note Under `PerceptionSource::kOracle` a single frame with no map ahead of
///       the camera fails the whole run — which is what happens on the last few
///       metres of any finite trajectory. The other sources treat an empty
///       frame as a valid "nothing detected".
Status RunSequenceEval(const std::vector<Pose>& poses, const Calibration& calib,
                       const std::shared_ptr<map::IMapLoader>& map_loader,
                       const SequenceEvalConfig& config,
                       std::vector<FrameEvalRecord>& out_records);

}  // namespace cam_loc::kitti

#endif  // CAM_LOC_KITTI_SEQUENCE_EVAL_RUNNER_H_
