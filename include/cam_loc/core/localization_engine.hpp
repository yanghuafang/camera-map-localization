#pragma once

#include <cam_loc/core/cost_aggregator.hpp>
#include <cam_loc/core/localization_debug.hpp>
#include <cam_loc/core/localization_kf.hpp>
#include <cam_loc/core/pose_sampler.hpp>
#include <cam_loc/core/projection.hpp>
#include <cam_loc/kitti/types.hpp>
#include <cam_loc/types/params.hpp>
#include <cam_loc/types/status.hpp>

#include <memory>
#include <optional>

namespace cam_loc::map {
class IMapLoader;
}

namespace cam_loc::core {

/// Per-frame localization: EKF predict → map-matching observation → optional global update.
///
/// Observation sources (updates), in order when enabled:
///   - Map matching: 3-DOF (x, y, yaw) pose from aggregated cost-grid argmin
///   - Global prior: full GT pose (debug) or KITTI odometry pose (fallback)
class LocalizationEngine {
 public:
  explicit LocalizationEngine(LocalizationParams params);

  void setMapLoader(std::shared_ptr<map::IMapLoader> loader);
  void setCalibration(const kitti::Calibration& calib);

  Status processFrame(const kitti::Egomotion& ego, const kitti::FramePerception& perception);

  /// When enabled, the last completed map-matching step fills debugSnapshot().
  void setDebugCapture(bool enabled) { debug_capture_ = enabled; }
  bool debugCapture() const { return debug_capture_; }

  const LocalizationResult& result() const { return result_; }
  const LocalizationKF& filter() const { return kf_; }
  const LocalizationDebugSnapshot& debugSnapshot() const { return debug_; }

 private:
  Status runMapMatching(const kitti::Egomotion& ego, const kitti::FramePerception& perception,
                        kitti::MapChunk& local_map, kitti::FramePerception& active_perception);

  Status applySamplingMeasurement(const kitti::Egomotion& ego, const Mat44& T_world_plane,
                                  const CostGrid& aggregated, const CostGrid::ArgMinResult& argmin);

  /// Optional KITTI odometry global pose as a loose measurement when map matching fails.
  void applyGlobalMeasurement(const kitti::Egomotion& ego);
  void writeResult(const kitti::Egomotion& ego);

  Mat44 samplingPlanePose(const kitti::Egomotion& ego) const;
  bool isCostMapFlat(const CostGrid& grid, const CostGrid::ArgMinResult& argmin) const;

  LocalizationParams params_;
  LocalizationKF kf_;
  LocalizationResult result_;
  std::shared_ptr<map::IMapLoader> map_loader_;

  std::optional<Projection> projection_;
  PoseSampler pose_sampler_;
  CostAggregator aggregator_;

  double total_travel_m_ = 0.0;
  bool debug_capture_ = false;
  LocalizationDebugSnapshot debug_;
};

}  // namespace cam_loc::core
