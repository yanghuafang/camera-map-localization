// Temporal cost aggregation: warp history grids into current plane frame and
// fuse.

#include "cam_loc/core/cost_aggregator.h"

#include "cam_loc/core/frames.h"

#ifdef CAMLOC_CUDA_ENABLED
#include "cam_loc/cuda/distance_transform.h"
#endif

#include <algorithm>
#include <cmath>
#include <memory>

namespace cam_loc::core {

namespace {

#ifdef CAMLOC_CUDA_ENABLED
// Flattens a pose into the row-major float[16] the kernel launchers take. Only
// the GPU path needs it, and it is compiled only there: left visible in a
// CPU-only build it is an unused function, which -Wall reports.
void PackMat44RowMajor(const Mat44& T, float out[16]) {
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      out[r * 4 + c] = static_cast<float>(T(r, c));
    }
  }
}
#endif

}  // namespace

CostAggregator::CostAggregator(const AggregationParams& params)
    : params_(params) {}

float CostAggregator::FrameWeight(double age_m) const {
  const float w = 1.f - params_.distance_decay * static_cast<float>(age_m);
  return w > 0.f ? w : 0.f;
}

Status CostAggregator::Aggregate(CostGrid& current,
                                 const Mat44& T_world_plane_curr,
                                 double travel_m, bool use_gpu) {
  if (history_.empty()) {
    return Status::kOk;
  }

  // A history frame whose plane has moved further than the grid's own half
  // extent warps to an offset off the grid, where SampleContinuous returns a
  // clamped border value. Averaging that in adds a constant, not evidence.
  const double reach_m = std::max(current.nx() * current.step_x(),
                                  current.ny() * current.step_y());

  auto usable_weight = [&](const HistoryCostFrame& hist) {
    const float w = FrameWeight(travel_m - hist.accum_distance_m);
    if (w <= 0.f) return 0.f;
    const double moved = (T_world_plane_curr.block<3, 1>(0, 3) -
                          hist.T_world_plane.block<3, 1>(0, 3))
                             .norm();
    return moved > reach_m ? 0.f : w;
  };

#ifdef CAMLOC_CUDA_ENABLED
  if (use_gpu && cuda::IsAvailable()) {
    std::vector<float> hist_inv_T;
    std::vector<float> hist_weights;
    std::vector<float> hist_costs;
    hist_inv_T.reserve(history_.size() * 16);
    hist_weights.reserve(history_.size());
    const int num_cells = current.DimX() * current.DimY() * current.DimW();
    hist_costs.reserve(history_.size() * static_cast<size_t>(num_cells));

    for (const auto& hist : history_) {
      const float w = usable_weight(hist);
      if (w <= 0.f) continue;
      const Mat44 inv_T = hist.T_world_plane.inverse();
      float packed[16];
      PackMat44RowMajor(inv_T, packed);
      hist_inv_T.insert(hist_inv_T.end(), packed, packed + 16);
      hist_weights.push_back(w);
      hist_costs.insert(hist_costs.end(), hist.costs->data().begin(),
                        hist.costs->data().end());
    }

    if (!hist_weights.empty()) {
      float T_curr[16];
      PackMat44RowMajor(T_world_plane_curr, T_curr);
      cuda::CostAggregateGpuParams gp;
      gp.dim_x = current.DimX();
      gp.dim_y = current.DimY();
      gp.dim_w = current.DimW();
      gp.nx = (gp.dim_x - 1) / 2;
      gp.ny = (gp.dim_y - 1) / 2;
      gp.nw = (gp.dim_w - 1) / 2;
      gp.step_x = static_cast<float>(current.step_x());
      gp.step_y = static_cast<float>(current.step_y());
      gp.step_yaw = static_cast<float>(current.step_yaw());
      gp.fuse_alpha = 0.5f;
      if (cuda::AggregateCostsGpu(current.data(), T_curr, hist_inv_T,
                                  hist_weights, hist_costs,
                                  gp) == Status::kOk) {
        return Status::kOk;
      }
    }
  }
#endif

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
    return Status::kOk;
  }
  for (float& c : aggregated.data()) {
    c /= sum_w;
  }

  const float alpha = 0.5f;
  // Equal blend of current-frame cost and distance-weighted history average
  for (size_t i = 0; i < current.data().size(); ++i) {
    current.data()[i] =
        alpha * current.data()[i] + (1.f - alpha) * aggregated.data()[i];
  }

  // use_gpu is read only by the CUDA block above, so it is unused in a CPU-only
  // build.
  (void)use_gpu;
  return Status::kOk;
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

void CostAggregator::Clear() { history_.clear(); }

}  // namespace cam_loc::core
