#ifndef CAM_LOC_CORE_BEV_CONFIG_H_
#define CAM_LOC_CORE_BEV_CONFIG_H_

namespace cam_loc::core {

/// Bird's-eye raster, in the vehicle frame: X forward, Y left, Z up.
///
/// Pixel layout: column maps vehicle X, row maps vehicle Y, origin at
/// (kForwardMinM, kLeftMinM). Projection::RigToBevPixel converts from cam0
/// first, so this is the only place the vehicle convention appears in a
/// raster.
///
/// The forward window starts at the camera rather than behind it: everything
/// this raster holds comes from inverse-perspective mapping of the image, and
/// a forward-facing camera sees nothing behind. Width follows from the aspect
/// ratio so both axes land on the same metres-per-pixel.
struct BevConfig {
  static constexpr int kImageHeight = 175;
  static constexpr double kForwardMinM = 0.0;
  static constexpr double kForwardMaxM = 40.0;
  static constexpr double kLeftMinM = -10.0;
  static constexpr double kLeftMaxM = 10.0;
  /// DT costs are capped here (pixels) when sampling map features.
  static constexpr float kDistanceMax = 5.f;

  static constexpr int kImageWidth =
      static_cast<int>((kForwardMaxM - kForwardMinM) / (kLeftMaxM - kLeftMinM) *
                       kImageHeight) +
      1;

  static double MetersPerPixelX() {
    return (kForwardMaxM - kForwardMinM) / kImageWidth;
  }
  static double MetersPerPixelY() {
    return (kLeftMaxM - kLeftMinM) / kImageHeight;
  }
};

}  // namespace cam_loc::core

#endif  // CAM_LOC_CORE_BEV_CONFIG_H_
