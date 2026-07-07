// CostGrid: 3-DOF pose-offset volume with yaw-major linear indexing and trilinear sampling.

#include <cam_loc/core/cost_grid.hpp>

#ifdef CAMLOC_CUDA_ENABLED
#include <cam_loc/cuda/distance_transform.hpp>
#endif

#include <algorithm>
#include <cmath>
#include <limits>

namespace cam_loc::core {

namespace {

int clampIndex(int v, int lo, int hi) { return std::max(lo, std::min(v, hi)); }

}  // namespace

// The grid is centered on the anchor pose: cell (nx_, ny_, nw_) is the zero offset and cells
// fan out symmetrically by ±half-extent. Dimension counts are expected to be odd so the center
// is a real cell; even counts truncate to a slightly asymmetric range.
CostGrid::CostGrid(const SamplingGridParams& params)
    : nx_((params.num_x - 1) / 2),
      ny_((params.num_y - 1) / 2),
      nw_((params.num_yaw - 1) / 2),
      step_x_(params.step_x_m),
      step_y_(params.step_y_m),
      step_yaw_(params.step_yaw_deg * M_PI / 180.0),
      costs_(static_cast<size_t>(params.totalHypotheses()), std::numeric_limits<float>::max()) {}

float& CostGrid::at(int ix, int iy, int iw) {
  return costs_[static_cast<size_t>(linearIndex(ix, iy, iw))];
}

float CostGrid::at(int ix, int iy, int iw) const {
  return costs_[static_cast<size_t>(linearIndex(ix, iy, iw))];
}

int CostGrid::linearIndex(int ix, int iy, int iw) const {
  // yaw-major: iw slowest, ix fastest (matches GPU cost kernels)
  return iw * dimX() * dimY() + iy * dimX() + ix;
}

Vec3 CostGrid::indexToOffset(int ix, int iy, int iw) const {
  return Vec3((ix - nx_) * step_x_, (iy - ny_) * step_y_, (iw - nw_) * step_yaw_);
}

void CostGrid::offsetToNearestIndex(const Vec3& offset, int& ix, int& iy, int& iw) const {
  ix = clampIndex(static_cast<int>(std::lround(offset.x() / step_x_ + nx_)), 0, dimX() - 1);
  iy = clampIndex(static_cast<int>(std::lround(offset.y() / step_y_ + ny_)), 0, dimY() - 1);
  iw = clampIndex(static_cast<int>(std::lround(offset.z() / step_yaw_ + nw_)), 0, dimW() - 1);
}

float CostGrid::sampleContinuous(double x_m, double y_m, double yaw_rad) const {
  // Trilinear read at a fractional offset. Indices are clamped to the border, so offsets outside
  // the sampled range hold the edge value instead of extrapolating — this is what lets temporal
  // aggregation sample a history cell whose warped offset falls off the current grid.
  const double fx = x_m / step_x_ + nx_;
  const double fy = y_m / step_y_ + ny_;
  const double fw = yaw_rad / step_yaw_ + nw_;

  const int x0 = clampIndex(static_cast<int>(std::floor(fx)), 0, dimX() - 1);
  const int y0 = clampIndex(static_cast<int>(std::floor(fy)), 0, dimY() - 1);
  const int w0 = clampIndex(static_cast<int>(std::floor(fw)), 0, dimW() - 1);
  const int x1 = clampIndex(x0 + 1, 0, dimX() - 1);
  const int y1 = clampIndex(y0 + 1, 0, dimY() - 1);
  const int w1 = clampIndex(w0 + 1, 0, dimW() - 1);

  const double tx = fx - x0;
  const double ty = fy - y0;
  const double tw = fw - w0;

  auto sample = [&](int ix, int iy, int iw) { return at(ix, iy, iw); };

  const double c000 = sample(x0, y0, w0);
  const double c100 = sample(x1, y0, w0);
  const double c010 = sample(x0, y1, w0);
  const double c110 = sample(x1, y1, w0);
  const double c001 = sample(x0, y0, w1);
  const double c101 = sample(x1, y0, w1);
  const double c011 = sample(x0, y1, w1);
  const double c111 = sample(x1, y1, w1);

  const double c00 = c000 * (1 - tx) + c100 * tx;
  const double c10 = c010 * (1 - tx) + c110 * tx;
  const double c01 = c001 * (1 - tx) + c101 * tx;
  const double c11 = c011 * (1 - tx) + c111 * tx;
  const double c0 = c00 * (1 - ty) + c10 * ty;
  const double c1 = c01 * (1 - ty) + c11 * ty;
  const double cxy = c0 * (1 - tw) + c1 * tw;
  return static_cast<float>(cxy);
}

void CostGrid::fill(float value) {
  std::fill(costs_.begin(), costs_.end(), value);
}

CostGrid::ArgMinResult CostGrid::argmin(bool use_gpu) const {
  ArgMinResult best;
  best.cost = std::numeric_limits<float>::max();

#ifdef CAMLOC_CUDA_ENABLED
  if (use_gpu && cuda::isAvailable()) {
    int idx = 0;
    float vmin = 0.f;
    if (cuda::argminGpu(costs_, idx, vmin) == Status::kOk) {
      best.cost = vmin;
      // Decode yaw-major linear index back to (ix, iy, iw)
      const int dim_xy = dimX() * dimY();
      best.iw = idx / dim_xy;
      const int rem = idx % dim_xy;
      best.iy = rem / dimX();
      best.ix = rem % dimX();
      return best;
    }
  }
#endif

  for (int iw = 0; iw < dimW(); ++iw) {
    for (int iy = 0; iy < dimY(); ++iy) {
      for (int ix = 0; ix < dimX(); ++ix) {
        const float c = at(ix, iy, iw);
        if (c < best.cost) {
          best.cost = c;
          best.ix = ix;
          best.iy = iy;
          best.iw = iw;
        }
      }
    }
  }
  return best;
}

}  // namespace cam_loc::core
