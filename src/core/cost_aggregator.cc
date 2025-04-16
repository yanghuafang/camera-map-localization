// Temporal cost aggregation: warp history grids into current plane frame and
// fuse.

#include "cam_loc/core/cost_aggregator.h"

#include <algorithm>
#include <cmath>
#include <memory>

#include "cam_loc/core/frames.h"

namespace cam_loc::core {

CostAggregator::CostAggregator(const AggregationParams& params)
    : params_(params) {}

float CostAggregator::FrameWeight(double age_m) const {
  const float w = 1.f - params_.distance_decay * static_cast<float>(age_m);
  return w > 0.f ? w : 0.f;
}

Status CostAggregator::Aggregate(CostGrid& current,
                                 const Mat44& T_world_plane_curr,
                                 double travel_m) {
  effective_frames_ = 1.0;
  if (history_.empty()) {
    return Status::kOk;
  }

  // A history frame whose plane has moved further than the grid's own half
  // extent warps to an offset off the grid, where SampleContinuous returns a
  // clamped border value. Averaging that in adds a constant, not evidence.
  const double reach_m = std::max(current.nx() * current.step_x(),
                                  current.ny() * current.step_y());

  // The current frame's share of the fuse; the history splits the rest.
  constexpr float kFuseAlpha = 0.5f;

  auto usable_weight = [&](const HistoryCostFrame& hist) {
    const float w = FrameWeight(travel_m - hist.accum_distance_m);
    if (w <= 0.f) return 0.f;
    const double moved = (T_world_plane_curr.block<3, 1>(0, 3) -
                          hist.T_world_plane.block<3, 1>(0, 3))
                             .norm();
    return moved > reach_m ? 0.f : w;
  };

  std::vector<float> weights;
  weights.reserve(history_.size());
  for (const auto& hist : history_) {
    const float w = usable_weight(hist);
    if (w > 0.f) weights.push_back(w);
  }
  effective_frames_ = EffectiveFrames(weights, kFuseAlpha);

  CostGrid aggregated(current);
  aggregated.Fill(0.f);

  float sum_w = 0.f;
  // Re-express each current-cell offset in each history plane, sample past cost
  for (const auto& hist : history_) {
    const float w = usable_weight(hist);
    if (w <= 0.f) continue;

    const Mat44 plane_motion =
        hist.T_world_plane.inverse() * T_world_plane_curr;
    const Mat44 plane_motion_inv = plane_motion.inverse();

    for (int iw = 0; iw < current.DimW(); ++iw) {
      for (int iy = 0; iy < current.DimY(); ++iy) {
        for (int ix = 0; ix < current.DimX(); ++ix) {
          const Vec3 offset_curr = current.IndexToOffset(ix, iy, iw);
          const Mat44 T_offset_curr = Frames::OffsetToCam0Transform(
              offset_curr.x(), offset_curr.y(), offset_curr.z());
          // Carry the *error*, not the pose. Both planes are the same drifting
          // estimate seen at two times, so the offset that means "the estimate
          // is one metre long" means the same thing in both -- rotated into the
          // older plane's axes, which is what conjugating by the relative plane
          // motion does.
          const Mat44 T_offset_prev =
              plane_motion * T_offset_curr * plane_motion_inv;
          const Vec3 offset_prev = Frames::Cam0TransformToOffset(T_offset_prev);

          const float past_cost = hist.costs->SampleContinuous(
              offset_prev.x(), offset_prev.y(), offset_prev.z());
          aggregated.At(ix, iy, iw) += w * past_cost;
        }
      }
    }
    sum_w += w;
  }

  // No history carried weight: leave the current frame alone. Normalizing by a
  // zero sum and blending the all-zero volume in anyway would scale the whole
  // cost surface by the fuse factor -- the argmin survives that, but the spread
  // does not, and the spread is what the flat gate and the measurement
  // covariance are both read from.
  if (sum_w <= 1e-6f) {
    effective_frames_ = 1.0;
    return Status::kOk;
  }
  for (float& c : aggregated.data()) {
    c /= sum_w;
  }

  // Equal blend of current-frame cost and distance-weighted history average
  for (size_t i = 0; i < current.data().size(); ++i) {
    current.data()[i] = kFuseAlpha * current.data()[i] +
                        (1.f - kFuseAlpha) * aggregated.data()[i];
  }

  return Status::kOk;
}

double CostAggregator::EffectiveFrames(
    const std::vector<float>& history_weights, float fuse_alpha) {
  double sum = 0.0;
  for (const float w : history_weights) sum += w;
  if (sum <= 1e-6) return 1.0;

  double sum_squares = static_cast<double>(fuse_alpha) * fuse_alpha;
  for (const float w : history_weights) {
    const double p = (1.0 - fuse_alpha) * w / sum;
    sum_squares += p * p;
  }
  return 1.0 / sum_squares;
}

void CostAggregator::PushHistory(const CostGrid& costs,
                                 const Mat44& T_world_plane, int frame,
                                 double travel_m) {
  HistoryCostFrame hf;
  hf.costs = std::make_shared<CostGrid>(costs);
  hf.T_world_plane = T_world_plane;
  hf.frame = frame;
  hf.accum_distance_m = travel_m;

  history_.push_back(std::move(hf));
  while (static_cast<int>(history_.size()) > params_.window_size) {
    history_.erase(history_.begin());
  }
}

}  // namespace cam_loc::core
