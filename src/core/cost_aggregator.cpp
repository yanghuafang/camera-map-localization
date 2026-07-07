// Temporal cost aggregation: warp history grids into current plane frame and fuse.

#include <cam_loc/core/cost_aggregator.hpp>

#include <cam_loc/core/projection.hpp>

#ifdef CAMLOC_CUDA_ENABLED
#include <cam_loc/cuda/distance_transform.hpp>
#endif

#include <cmath>
#include <memory>

namespace cam_loc::core {

namespace {

void packMat44RowMajor(const Mat44& T, float out[16]) {
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      out[r * 4 + c] = static_cast<float>(T(r, c));
    }
  }
}

}  // namespace

CostAggregator::CostAggregator(const AggregationParams& params) : params_(params) {}

float CostAggregator::frameWeight(double accum_dist) const {
  const float w = 1.f - params_.distance_decay * static_cast<float>(accum_dist);
  return w > 0.f ? w : 0.f;
}

Status CostAggregator::aggregate(CostGrid& current, const Mat44& T_world_plane_curr,
                                 double frame_travel_m, bool use_gpu) {
  if (history_.empty()) {
    return Status::kOk;
  }

#ifdef CAMLOC_CUDA_ENABLED
  if (use_gpu && cuda::isAvailable()) {
    std::vector<float> hist_inv_T;
    std::vector<float> hist_weights;
    std::vector<float> hist_costs;
    hist_inv_T.reserve(history_.size() * 16);
    hist_weights.reserve(history_.size());
    const int num_cells = current.dimX() * current.dimY() * current.dimW();
    hist_costs.reserve(history_.size() * static_cast<size_t>(num_cells));

    for (const auto& hist : history_) {
      const float w = frameWeight(hist.accum_distance_m);
      if (w <= 0.f) continue;
      const Mat44 inv_T = hist.T_world_plane.inverse();
      float packed[16];
      packMat44RowMajor(inv_T, packed);
      hist_inv_T.insert(hist_inv_T.end(), packed, packed + 16);
      hist_weights.push_back(w);
      hist_costs.insert(hist_costs.end(), hist.costs->data().begin(), hist.costs->data().end());
    }

    if (!hist_weights.empty()) {
      float T_curr[16];
      packMat44RowMajor(T_world_plane_curr, T_curr);
      cuda::CostAggregateGpuParams gp;
      gp.dim_x = current.dimX();
      gp.dim_y = current.dimY();
      gp.dim_w = current.dimW();
      gp.nx = (gp.dim_x - 1) / 2;
      gp.ny = (gp.dim_y - 1) / 2;
      gp.nw = (gp.dim_w - 1) / 2;
      gp.step_x = static_cast<float>(current.stepX());
      gp.step_y = static_cast<float>(current.stepY());
      gp.step_yaw = static_cast<float>(current.stepYaw());
      gp.fuse_alpha = 0.5f;
      if (cuda::aggregateCostsGpu(current.data(), T_curr, hist_inv_T, hist_weights, hist_costs,
                                  gp) == Status::kOk) {
        (void)frame_travel_m;
        return Status::kOk;
      }
    }
  }
#endif

  CostGrid aggregated(current);
  aggregated.fill(0.f);

  float sum_w = 0.f;
  // Re-express each current-cell offset in each history plane, sample past cost
  for (const auto& hist : history_) {
    const float w = frameWeight(hist.accum_distance_m);
    if (w <= 0.f) continue;

    for (int iw = 0; iw < current.dimW(); ++iw) {
      for (int iy = 0; iy < current.dimY(); ++iy) {
        for (int ix = 0; ix < current.dimX(); ++ix) {
          const Vec3 offset_curr = current.indexToOffset(ix, iy, iw);
          const Mat44 T_offset_curr =
              Projection::offsetToTransform(offset_curr.x(), offset_curr.y(), offset_curr.z());
          const Mat44 T_world_hyp = T_world_plane_curr * T_offset_curr;
          const Mat44 T_offset_prev = hist.T_world_plane.inverse() * T_world_hyp;
          const Vec3 offset_prev = Projection::transformToOffset(T_offset_prev);

          const float past_cost = hist.costs->sampleContinuous(offset_prev.x(), offset_prev.y(),
                                                             offset_prev.z());
          aggregated.at(ix, iy, iw) += w * past_cost;
        }
      }
    }
    sum_w += w;
  }

  if (sum_w > 1e-6f) {
    for (float& c : aggregated.data()) {
      c /= sum_w;
    }
  }

  const float alpha = 0.5f;
  // Equal blend of current-frame cost and distance-weighted history average
  for (size_t i = 0; i < current.data().size(); ++i) {
    current.data()[i] = alpha * current.data()[i] + (1.f - alpha) * aggregated.data()[i];
  }

  (void)frame_travel_m;
  return Status::kOk;
}

void CostAggregator::pushHistory(const CostGrid& costs, const Mat44& T_world_plane, int frame) {
  HistoryCostFrame hf;
  hf.costs = std::make_shared<CostGrid>(costs);
  hf.T_world_plane = T_world_plane;
  hf.frame = frame;

  if (!history_.empty()) {
    // Accumulated planar travel since first history frame (XY norm of plane translation delta)
    hf.accum_distance_m = history_.back().accum_distance_m +
                          (T_world_plane.block<3, 1>(0, 3) -
                           history_.back().T_world_plane.block<3, 1>(0, 3))
                              .head<2>()
                              .norm();
  }

  history_.push_back(std::move(hf));
  while (static_cast<int>(history_.size()) > params_.window_size) {
    history_.erase(history_.begin());
  }
}

void CostAggregator::clear() { history_.clear(); }

}  // namespace cam_loc::core
