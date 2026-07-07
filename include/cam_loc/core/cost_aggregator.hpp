#pragma once

/// Temporal fusion of per-frame 3-D cost volumes over a sliding pose history.

#include <cam_loc/core/cost_grid.hpp>
#include <cam_loc/types/params.hpp>
#include <cam_loc/types/status.hpp>

#include <memory>
#include <vector>

namespace cam_loc::core {

struct HistoryCostFrame {
  std::shared_ptr<CostGrid> costs;
  Mat44 T_world_plane = Mat44::Identity();
  double accum_distance_m = 0.0;
  int frame = 0;
};

/// Multi-frame cost aggregation in sampling-plane coordinates.
///
/// Past frames are re-expressed in the current plane frame: for each cell offset
/// in the current grid, the corresponding offset in a history grid is found via
/// T_offset_prev = inv(T_world_plane_hist) · T_world_plane_curr · T_offset_curr,
/// then sampleContinuous on the stored history volume. Weight decays with
/// accumulated travel distance; result is fused 50/50 with the current frame.
class CostAggregator {
 public:
  explicit CostAggregator(const AggregationParams& params);

  Status aggregate(CostGrid& current, const Mat44& T_world_plane_curr, double frame_travel_m,
                   bool use_gpu = false);

  void pushHistory(const CostGrid& costs, const Mat44& T_world_plane, int frame);

  void clear();

 private:
  float frameWeight(double accum_dist) const;

  AggregationParams params_;
  std::vector<HistoryCostFrame> history_;
};

}  // namespace cam_loc::core
