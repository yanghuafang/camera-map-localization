/// Sequence eval loop: egomotion → perception → engine → FrameEvalRecord per frame.
#include <cam_loc/kitti/sequence_eval_runner.hpp>

#include <cam_loc/core/localization_engine.hpp>
#include <cam_loc/kitti/calib_parser.hpp>
#include <cam_loc/perception/resolve.hpp>

#include <chrono>
#include <memory>

namespace cam_loc::kitti {

Status runSequenceEval(const std::vector<Pose>& poses, const Calibration& calib,
                       const std::shared_ptr<map::IMapLoader>& map_loader,
                       const SequenceEvalConfig& config,
                       std::vector<FrameEvalRecord>& out_records) {
  out_records.clear();
  if (poses.empty() || !map_loader) return Status::kInvalidArgument;

  const int total =
      config.max_frames < 0
          ? static_cast<int>(poses.size())
          : std::min(config.max_frames, static_cast<int>(poses.size()));
  const int start = std::min(config.start_frame, total);
  if (start >= total) return Status::kInvalidArgument;

  core::LocalizationEngine engine(config.localization);
  engine.setMapLoader(map_loader);
  engine.setCalibration(calib);

  core::Projection projection(calib);

  out_records.reserve(static_cast<size_t>(total - start));
  // Per-frame eval: egomotion → perception → localize → record errors/diagnostics.
  for (int f = start; f < total; ++f) {
    Egomotion ego;
    if (buildEgomotion(poses, f, ego) != Status::kOk) {
      return Status::kInvalidArgument;
    }

    const Mat44& T_gt = poses[static_cast<size_t>(f)].T_world_cam0;
    kitti::FramePerception perception;
    perception::PerceptionResolveInfo pinfo;
    // Only oracle perception is mandatory; file/auto/noisy tolerate empty frames (missing
    // detections are valid, e.g. no lane markings visible).
    if (resolvePerception(config.perception_source, config.perception_root, config.sequence, f,
                          *map_loader, config.localization.map_query_radius_m, projection, T_gt,
                          config.noise, config.noise_seed, perception,
                          pinfo) != Status::kOk &&
        config.perception_source == perception::PerceptionSource::kOracle) {
      return Status::kInvalidArgument;
    }

    const auto t0 = std::chrono::steady_clock::now();
    if (engine.processFrame(ego, perception) != Status::kOk) {
      return Status::kInvalidArgument;
    }
    const auto t1 = std::chrono::steady_clock::now();

    FrameEvalRecord rec;
    rec.frame = f;
    rec.frame_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    rec.pose_error = poseError(engine.result().T_world_rig, T_gt);
    rec.min_cost = engine.result().aggregate_min_cost;
    rec.cost_spread = engine.result().cost_map_spread;
    rec.perception_synthesized = engine.result().perception_synthesized || pinfo.synthesized;
    rec.cost_map_flat = engine.result().cost_map_flat;
    rec.sampling_applied = engine.result().sampling_measurement_applied;
    rec.best_offset_m = engine.result().best_offset_norm_m;
    rec.loaded_from_file = pinfo.loaded_from_file;
    rec.noise_applied = pinfo.noise_applied;
    out_records.push_back(rec);
  }

  return Status::kOk;
}

}  // namespace cam_loc::kitti
