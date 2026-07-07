// CPU Felzenszwalb Euclidean distance transform and polyline rasterization.

#include <cam_loc/core/distance_transform_cpu.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace cam_loc::core {

namespace {

void edt1d(const std::vector<float>& f, std::vector<float>& d, int n) {
  // 1D squared distance transform (Felzenszwalb & Huttenlocher, O(n)). Each sample q defines a
  // parabola (q' - q)^2 + f[q]; the result is their lower envelope sampled at every q'.
  // Phase 1 sweeps left→right building that envelope; phase 2 reads it back:
  //   v[k] = location of the k-th parabola that survives on the envelope,
  //   z[k] = boundary where parabola k-1 hands off to parabola k,
  //   k    = index of the rightmost parabola added so far.
  std::vector<int> v(n);
  std::vector<float> z(n + 1);
  int k = 0;
  v[0] = 0;
  z[0] = -std::numeric_limits<float>::infinity();
  z[1] = std::numeric_limits<float>::infinity();

  // Phase 1: intersect each new parabola with the envelope, popping any it now dominates.
  for (int q = 1; q < n; ++q) {
    float s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);
    while (s <= z[k]) {
      --k;
      s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<float>::infinity();
  }

  // Phase 2: for each position, advance to the parabola whose interval covers it and evaluate.
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) ++k;
    d[q] = (q - v[k]) * (q - v[k]) + f[v[k]];
  }
}

void drawLine(std::vector<uint8_t>& img, int width, int height, Vec2 a, Vec2 b,
              float stroke) {
  const int steps = static_cast<int>(std::max(1.0, (b - a).norm()));
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    const int x = static_cast<int>(std::lround(a.x() * (1 - t) + b.x() * t));
    const int y = static_cast<int>(std::lround(a.y() * (1 - t) + b.y() * t));
    const int r = static_cast<int>(std::ceil(stroke));
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        const int px = x + dx;
        const int py = y + dy;
        if (px >= 0 && px < width && py >= 0 && py < height) {
          img[py * width + px] = 0;
        }
      }
    }
  }
}

}  // namespace

Status DistanceTransformCpu::compute(const std::vector<uint8_t>& binary, int width, int height,
                                     std::vector<float>& out_distance) {
  if (width <= 0 || height <= 0 ||
      static_cast<size_t>(width * height) != binary.size()) {
    return Status::kInvalidArgument;
  }

  const int n = width * height;
  std::vector<float> f(n);
  // Seed the transform: feature pixels (binary == 0) start at distance 0, everything else at
  // a large "infinity" so the two separable passes below propagate real distances into it.
  for (int i = 0; i < n; ++i) {
    f[i] = binary[i] == 0 ? 0.f : 1e8f;
  }

  std::vector<float> tmp(n);
  std::vector<float> grid(n);

  // Pass 1: EDT along columns (store squared distance)
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      grid[y] = f[y * width + x];
    }
    edt1d(grid, tmp, height);
    for (int y = 0; y < height; ++y) {
      f[y * width + x] = tmp[y];
    }
  }

  // Pass 2: EDT along rows, sqrt → pixel Euclidean distance
  out_distance.resize(n);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      grid[x] = f[y * width + x];
    }
    edt1d(grid, tmp, width);
    for (int x = 0; x < width; ++x) {
      out_distance[y * width + x] = std::sqrt(tmp[x]);
    }
  }

  return Status::kOk;
}

Status DistanceTransformCpu::rasterizePolylines(const std::vector<Vec2>& points, int width,
                                                int height, float stroke_width,
                                                std::vector<uint8_t>& out_binary) {
  if (width <= 0 || height <= 0) {
    return Status::kInvalidArgument;
  }
  out_binary.assign(static_cast<size_t>(width * height), 255);
  if (points.size() < 2) {
    return Status::kOk;
  }
  for (size_t i = 1; i < points.size(); ++i) {
    drawLine(out_binary, width, height, points[i - 1], points[i], stroke_width);
  }
  return Status::kOk;
}

}  // namespace cam_loc::core
