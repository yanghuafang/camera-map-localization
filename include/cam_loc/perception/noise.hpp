#pragma once

#include <cam_loc/kitti/types.hpp>

#include <cmath>

namespace cam_loc::perception {

/// Tunable corruption applied to image-space lane/boundary polylines.
struct PerceptionNoiseParams {
  /// Gaussian jitter on image polyline vertices (pixels).
  double pixel_std = 0.0;
  /// Per-vertex dropout probability in [0, 1].
  double point_dropout = 0.0;
  /// Per-polyline dropout probability in [0, 1].
  double polyline_dropout = 0.0;
  /// Constant lateral shift applied to all vertices (pixels, +u).
  double lateral_bias_px = 0.0;

  bool enabled() const {
    return pixel_std > 0.0 || point_dropout > 0.0 || polyline_dropout > 0.0 ||
           std::abs(lateral_bias_px) > 1e-9;
  }
};

/// Deterministic noise injection for map-matching robustness studies.
kitti::FramePerception addPerceptionNoise(const kitti::FramePerception& in,
                                          const PerceptionNoiseParams& params, uint32_t seed);

}  // namespace cam_loc::perception
