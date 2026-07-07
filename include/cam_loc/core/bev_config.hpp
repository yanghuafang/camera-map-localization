#pragma once

namespace cam_loc::core {

/// Bird's-eye raster aligned with the KITTI rig frame (X forward, Y left, Z up).
///
/// Pixel layout: col maps rig X, row maps rig Y. Origin at (kXMin, kYMin).
/// Width is chosen so aspect ratio matches the X/Y span at kImageHeight rows.
/// DT costs are capped at kDistanceMax (pixels) when sampling map features.
struct BevConfig {
  static constexpr int kImageHeight = 175;
  static constexpr double kXMin = -20.0;
  static constexpr double kXMax = 20.0;
  static constexpr double kYMin = -10.0;
  static constexpr double kYMax = 10.0;
  static constexpr float kDistanceMax = 5.f;

  static constexpr int kImageWidth =
      static_cast<int>((kXMax - kXMin) / (kYMax - kYMin) * kImageHeight) + 1;

  static double metersPerPixelX() { return (kXMax - kXMin) / kImageWidth; }
  static double metersPerPixelY() { return (kYMax - kYMin) / kImageHeight; }
};

}  // namespace cam_loc::core
