// Deterministic perception noise: seeded dropout and pixel jitter on 2-D
// polylines.

#include "cam_loc/perception/noise.h"

#include <cmath>
#include <random>

namespace cam_loc::perception {

namespace {

uint32_t MixSeed(uint32_t seed, int frame) {
  // Per-frame seed: reproducible run-to-run yet decorrelated between frames.
  // 2654435761 is Knuth's multiplicative-hash constant (2^32 / golden ratio),
  // chosen for good bit dispersion.
  return seed ^ static_cast<uint32_t>(frame * 2654435761u);
}

kitti::Polyline2D NoisyPolyline(const kitti::Polyline2D& pl,
                                const PerceptionNoiseParams& params,
                                std::mt19937& rng) {
  std::uniform_real_distribution<double> uni(0.0, 1.0);
  // Constructed only when it will be sampled: std::normal_distribution requires
  // a positive standard deviation, and zero is the default here.
  const bool jitter = params.pixel_std > 0.0;
  std::normal_distribution<double> gauss(0.0, jitter ? params.pixel_std : 1.0);

  kitti::Polyline2D out;
  out.type = pl.type;
  for (const auto& p : pl.points) {
    if (params.point_dropout > 0.0 && uni(rng) < params.point_dropout) {
      continue;
    }
    Vec2 q = p;
    q.x() += params.lateral_bias_px;
    if (jitter) {
      q.x() += gauss(rng);
      q.y() += gauss(rng);
    }
    out.points.push_back(q);
  }
  return out;
}

void NoisePolylines(std::vector<kitti::Polyline2D>& polylines,
                    const PerceptionNoiseParams& params, std::mt19937& rng) {
  std::uniform_real_distribution<double> uni(0.0, 1.0);
  std::vector<kitti::Polyline2D> kept;
  kept.reserve(polylines.size());
  // Drop whole polylines first, then jitter/drop individual vertices.
  for (const auto& pl : polylines) {
    if (params.polyline_dropout > 0.0 && uni(rng) < params.polyline_dropout) {
      continue;
    }
    auto noisy = NoisyPolyline(pl, params, rng);
    if (noisy.points.size() >= 2) {
      kept.push_back(std::move(noisy));
    }
  }
  polylines.swap(kept);
}

}  // namespace

kitti::FramePerception AddPerceptionNoise(const kitti::FramePerception& in,
                                          const PerceptionNoiseParams& params,
                                          uint32_t seed) {
  if (!params.Enabled()) {
    return in;
  }
  kitti::FramePerception out = in;
  std::mt19937 rng(MixSeed(seed, in.frame));
  NoisePolylines(out.features, params, rng);
  return out;
}

}  // namespace cam_loc::perception
