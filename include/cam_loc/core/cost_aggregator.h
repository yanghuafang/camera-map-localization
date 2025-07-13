#ifndef CAM_LOC_CORE_COST_AGGREGATOR_H_
#define CAM_LOC_CORE_COST_AGGREGATOR_H_

/// Temporal fusion of per-frame 3-D cost volumes over a sliding pose history.

#include <memory>
#include <vector>

#include "cam_loc/core/cost_grid.h"
#include "cam_loc/types/params.h"
#include "cam_loc/types/status.h"

namespace cam_loc::core {

/// One retained per-frame cost volume and the plane pose it was scored at.
struct HistoryCostFrame {
  std::shared_ptr<CostGrid> costs;
  Mat44 T_world_plane = Mat44::Identity();
  /// Planar distance travelled since the *first* retained frame, not since the
  /// current one — which is what makes the decay below behave as it does.
  double accum_distance_m = 0.0;
  int frame = 0;
};

/// Multi-frame cost aggregation in sampling-plane coordinates.
///
/// What accumulates across frames is evidence about the *pose error*, not about
/// a pose. Every sampling plane is the same drifting estimate seen at a
/// different time, so a grid cell meaning "the estimate is a metre long" means
/// that in every frame; the only thing that changes between two planes is the
/// axes it is expressed in. Each history offset is therefore conjugated by the
/// relative plane motion, `M · T_offset · M⁻¹`, and the weighted history
/// average is fused 50/50 with the current frame.
///
/// Warping the *pose* instead — asking the old grid what it thought of where
/// the vehicle is now — reads the old observation at a pose it never scored as
/// good, and drags the estimate a frame's travel backwards every frame.
///
/// Weights decay with how far the vehicle has travelled *since* a history
/// frame, so the newest frame counts most and a frame older than
/// `1 / distance_decay` metres drops out entirely. History whose plane has
/// moved further than the grid is wide is dropped as well: its warped offset
/// falls off the grid, where SampleContinuous can only return a clamped border
/// value, and averaging that in is adding a constant rather than evidence.
class CostAggregator {
 public:
  explicit CostAggregator(const AggregationParams& params);

  /// Fuse the retained history into @p current, in place.
  ///
  /// @param current            Scored volume for this frame; overwritten with
  ///                           the fused result.
  /// @param T_world_plane_curr Sampling-plane pose the volume was scored at.
  /// @param travel_m           Odometer reading for this frame, metres. Ages
  ///                           are differences against the readings recorded by
  ///                           PushHistory, so both must come from the same
  ///                           odometer.
  /// @param use_gpu            Try the CUDA path first, falling back to CPU.
  /// @return `kOk`. Leaves @p current untouched when no history carries weight,
  ///         rather than blending in an empty average.
  Status Aggregate(CostGrid& current, const Mat44& T_world_plane_curr,
                   double travel_m, bool use_gpu = false);

  /// Retain a copy of @p costs, evicting the oldest past `window_size`.
  ///
  /// @param travel_m Odometer reading for this frame, metres.
  void PushHistory(const CostGrid& costs, const Mat44& T_world_plane, int frame,
                   double travel_m);

  /// Drop all history. Call between sequences.
  void Clear();

 private:
  /// @param age_m Distance travelled since the history frame was recorded.
  /// @return `1 − distance_decay · age_m`, floored at zero.
  float FrameWeight(double age_m) const;

  AggregationParams params_;
  std::vector<HistoryCostFrame> history_;
};

}  // namespace cam_loc::core

#endif  // CAM_LOC_CORE_COST_AGGREGATOR_H_
