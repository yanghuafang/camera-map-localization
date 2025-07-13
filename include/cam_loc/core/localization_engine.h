#ifndef CAM_LOC_CORE_LOCALIZATION_ENGINE_H_
#define CAM_LOC_CORE_LOCALIZATION_ENGINE_H_

#include <memory>
#include <optional>

#include "cam_loc/core/cost_aggregator.h"
#include "cam_loc/core/localization_debug.h"
#include "cam_loc/core/localization_kf.h"
#include "cam_loc/core/pose_sampler.h"
#include "cam_loc/core/projection.h"
#include "cam_loc/kitti/types.h"
#include "cam_loc/types/params.h"
#include "cam_loc/types/status.h"

namespace cam_loc::map {
class IMapLoader;
}

namespace cam_loc::core {

/// Per-frame localization: EKF predict → map-matching observation → optional
/// global update.
///
/// Observation sources (updates), in order when enabled:
///   - Map matching: 3-DOF (x, y, yaw) pose from aggregated cost-grid argmin
///   - Global prior: full GT pose (debug) or KITTI odometry pose (fallback)
class LocalizationEngine {
 public:
  explicit LocalizationEngine(LocalizationParams params);

  void set_map_loader(std::shared_ptr<map::IMapLoader> loader);
  void SetCalibration(const kitti::Calibration& calib);

  Status ProcessFrame(const kitti::Egomotion& ego,
                      const kitti::FramePerception& perception);

  /// When enabled, the last completed map-matching step fills debug_snapshot().
  void set_debug_capture(bool enabled) { debug_capture_ = enabled; }
  bool debug_capture() const { return debug_capture_; }

  const LocalizationResult& result() const { return result_; }
  const LocalizationKF& filter() const { return kf_; }
  const LocalizationDebugSnapshot& debug_snapshot() const { return debug_; }

 private:
  Status RunMapMatching(const kitti::Egomotion& ego,
                        const kitti::FramePerception& perception,
                        kitti::MapChunk& local_map,
                        kitti::FramePerception& active_perception);

  /// Turn the winning grid cell into a world-frame pose measurement, with a
  /// covariance built on vehicle axes and rotated into the filter's frame.
  Status ApplySamplingMeasurement(const Mat44& T_world_plane,
                                  const CostGrid& aggregated,
                                  const CostGrid::ArgMinResult& argmin);

  /// Optional KITTI odometry global pose as a loose measurement when map
  /// matching fails.
  void ApplyGlobalMeasurement(const kitti::Egomotion& ego);
  void WriteResult(const kitti::Egomotion& ego);

  Mat44 SamplingPlanePose(const kitti::Egomotion& ego) const;
  bool IsCostMapFlat(const CostGrid& grid,
                     const CostGrid::ArgMinResult& argmin) const;

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

#endif  // CAM_LOC_CORE_LOCALIZATION_ENGINE_H_
