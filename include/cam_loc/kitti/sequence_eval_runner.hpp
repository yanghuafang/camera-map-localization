#pragma once

/// Frame-by-frame sequence evaluation driver.
///
/// Playback flow per frame: buildEgomotion → resolvePerception → LocalizationEngine::processFrame
/// → record pose error and map-matching diagnostics.

#include <cam_loc/kitti/sequence_eval.hpp>
#include <cam_loc/kitti/types.hpp>
#include <cam_loc/map/map_loader.hpp>
#include <cam_loc/types/status.hpp>

#include <memory>
#include <vector>

namespace cam_loc::kitti {

/// Run localization over a pose sequence and collect per-frame eval records.
Status runSequenceEval(const std::vector<Pose>& poses, const Calibration& calib,
                       const std::shared_ptr<map::IMapLoader>& map_loader,
                       const SequenceEvalConfig& config,
                       std::vector<FrameEvalRecord>& out_records);

}  // namespace cam_loc::kitti
