#pragma once

#include <cam_loc/types/status.hpp>

#include <cstdint>
#include <vector>

namespace cam_loc::core {

/// CPU Euclidean distance transform (Felzenszwalb & Huttenlocher, squared distances).
///
/// Two-pass separable EDT: 1D transform along columns (squared dist), then along
/// rows with sqrt to produce pixel-space Euclidean distance to nearest feature.
class DistanceTransformCpu {
 public:
  /// Input: binary image row-major, 0 = feature, 255 = background.
  /// Output: float distance (pixels) to nearest feature pixel.
  static Status compute(const std::vector<uint8_t>& binary, int width, int height,
                        std::vector<float>& out_distance);

  static Status rasterizePolylines(const std::vector<Vec2>& points, int width, int height,
                                   float stroke_width, std::vector<uint8_t>& out_binary);
};

}  // namespace cam_loc::core
