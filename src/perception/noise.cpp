// Deterministic perception noise: seeded dropout and pixel jitter on 2-D polylines.

#include <cam_loc/perception/noise.hpp>

#include <cmath>
#include <random>

namespace cam_loc::perception {

namespace {

uint32_t mixSeed(uint32_t seed, int frame) {
  // Per-frame seed: reproducible run-to-run yet decorrelated between frames. 2654435761 is
  // Knuth's multiplicative-hash constant (2^32 / golden ratio), chosen for good bit dispersion.
  return seed ^ static_cast<uint32_t>(frame * 2654435761u);
}

kitti::Polyline2D noisyPolyline(const kitti::Polyline2D& pl, const PerceptionNoiseParams& params,
                                std::mt19937& rng) {
  std::uniform_real_distribution<double> uni(0.0, 1.0);
  std::normal_distribution<double> gauss(0.0, params.pixel_std);

  kitti::Polyline2D out;
  out.type = pl.type;
  for (const auto& p : pl.points) {
    if (params.point_dropout > 0.0 && uni(rng) < params.point_dropout) {
      continue;
    }
    Vec2 q = p;
    q.x() += params.lateral_bias_px;
    if (params.pixel_std > 0.0) {
      q.x() += gauss(rng);
      q.y() += gauss(rng);
    }
    out.points.push_back(q);
  }
  return out;
}

void noisePolylines(std::vector<kitti::Polyline2D>& polylines, const PerceptionNoiseParams& params,
                    std::mt19937& rng) {
  std::uniform_real_distribution<double> uni(0.0, 1.0);
  std::vector<kitti::Polyline2D> kept;
  kept.reserve(polylines.size());
  // Drop whole polylines first, then jitter/drop individual vertices.
  for (const auto& pl : polylines) {
    if (params.polyline_dropout > 0.0 && uni(rng) < params.polyline_dropout) {
      continue;
    }
    auto noisy = noisyPolyline(pl, params, rng);
    if (noisy.points.size() >= 2) {
      kept.push_back(std::move(noisy));
    }
  }
  polylines.swap(kept);
}

}  // namespace

kitti::FramePerception addPerceptionNoise(const kitti::FramePerception& in,
                                          const PerceptionNoiseParams& params, uint32_t seed) {
  if (!params.enabled()) {
    return in;
  }
  kitti::FramePerception out = in;
  std::mt19937 rng(mixSeed(seed, in.frame));
  noisePolylines(out.lane_lines, params, rng);
  noisePolylines(out.road_boundaries, params, rng);
  return out;
}

}  // namespace cam_loc::perception
