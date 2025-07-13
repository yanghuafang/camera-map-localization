/// Sequence eval loop: egomotion → perception → engine → FrameEvalRecord per
/// frame.
#include "cam_loc/kitti/sequence_eval_runner.h"

#include <chrono>
#include <memory>

#include "cam_loc/core/localization_engine.h"
#include "cam_loc/kitti/calib_parser.h"
#include "cam_loc/perception/resolve.h"

namespace cam_loc::kitti {

Status RunSequenceEval(const std::vector<Pose>& poses, const Calibration& calib,
                       const std::shared_ptr<map::IMapLoader>& map_loader,
                       const SequenceEvalConfig& config,
                       std::vector<FrameEvalRecord>& out_records) {
  out_records.clear();
  if (poses.empty() || !map_loader) return Status::kInvalidArgument;

  // max_frames is a count from start_frame, not an end index: asking for 80
  // frames from frame 10 should give 80 frames.
  const int available = static_cast<int>(poses.size());
  const int start = std::max(0, config.start_frame);
  if (start >= available) return Status::kInvalidArgument;
  const int count =
      config.max_frames < 0 ? available - start : config.max_frames;
  const int end = std::min(available, start + count);
  if (end <= start) return Status::kInvalidArgument;

  core::LocalizationEngine engine(config.localization);
  engine.set_map_loader(map_loader);
  engine.SetCalibration(calib);

  core::Projection projection(calib);

  out_records.reserve(static_cast<size_t>(end - start));
  // Per-frame eval: egomotion → perception → localize → record
  // errors/diagnostics.
  for (int f = start; f < end; ++f) {
    Egomotion ego;
    if (BuildEgomotion(poses, f, ego) != Status::kOk) {
      return Status::kInvalidArgument;
    }

    const Mat44& T_gt = poses[static_cast<size_t>(f)].T_world_cam0;
    kitti::FramePerception perception;
    perception::PerceptionResolveInfo pinfo;
    // A frame with nothing to see is a valid outcome for every source, not an
    // error: it is what the last few metres of any finite map look like, and
    // what a real detector returns on an empty road. The engine reports it as
    // an unmatched frame, which the match rate already accounts for.
    ResolvePerception(config.perception_source, config.perception_root,
                      config.sequence, f, *map_loader,
                      config.localization.map_query_radius_m, projection, T_gt,
                      config.noise, config.noise_seed, perception, pinfo);

    const auto t0 = std::chrono::steady_clock::now();
    if (engine.ProcessFrame(ego, perception) != Status::kOk) {
      return Status::kInvalidArgument;
    }
    const auto t1 = std::chrono::steady_clock::now();

    FrameEvalRecord rec;
    rec.frame = f;
    rec.frame_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    rec.pose_error = PoseError(engine.result().T_world_rig, T_gt);
    rec.min_cost = engine.result().aggregate_min_cost;
    rec.cost_spread = engine.result().cost_map_spread;
    rec.perception_synthesized = pinfo.synthesized;
    rec.cost_map_flat = engine.result().cost_map_flat;
    rec.match_cost_too_high = engine.result().match_cost_too_high;
    rec.sampling_applied = engine.result().sampling_measurement_applied;
    rec.best_offset_m = engine.result().best_offset_norm_m;
    rec.loaded_from_file = pinfo.loaded_from_file;
    rec.noise_applied = pinfo.noise_applied;
    out_records.push_back(rec);
  }

  return Status::kOk;
}

}  // namespace cam_loc::kitti
