#ifndef CAM_LOC_CORE_PROJECTION_H_
#define CAM_LOC_CORE_PROJECTION_H_

#include "cam_loc/core/bev_config.h"
#include "cam_loc/core/frames.h"
#include "cam_loc/kitti/types.h"
#include "cam_loc/types/status.h"

namespace cam_loc::core {

/// Pinhole projection and the image ↔ ground ↔ bird's-eye transforms.
///
/// The rig frame is KITTI rectified cam0 — X right, Y down, Z forward — and so
/// is the world frame, because KITTI odometry poses are cam0 poses. The
/// bird's-eye raster and the pose-grid offsets are the two places that reason
/// in the vehicle frame instead; Frames holds that conversion, and both are
/// converted here rather than leaving two conventions in play.
///
/// - rig → image: `u = fx·x/z + cx`, `v = fy·y/z + cy`; points at or behind the
///   camera are rejected rather than projected.
/// - image → ground: the pixel ray is intersected with the road plane, which in
///   cam0 is `y = ground_height_m` because Y points down. Pixels on or above
///   the horizon do not meet it and are rejected.
/// - rig → BEV pixel: the point is taken to the vehicle frame first, so the
///   raster's column axis is forward and its row axis is left.
class Projection {
 public:
  /// KITTI's cam0 height above the road, metres.
  static constexpr double kDefaultGroundHeightM = 1.65;

  /// @param calib Intrinsics are read from `P0` (rectified cam0).
  explicit Projection(const kitti::Calibration& calib);

  /// Height of the camera above the road, metres. KITTI's cam0 sits about
  /// 1.65 m up; the value only affects the inverse-perspective path.
  void set_ground_height_m(double h) { ground_height_m_ = h; }
  double ground_height_m() const { return ground_height_m_; }

  /// Project a rig-frame point to image pixels.
  ///
  /// @param p_rig  Point in the cam0 rig frame, metres.
  /// @param out_uv Pixel coordinates, written only on success.
  /// @return `kInvalidArgument` when the point is at or behind the camera.
  Status ProjectRigToImage(const Vec3& p_rig, Vec2& out_uv) const;

  /// Inverse perspective: intersect a pixel ray with the road plane.
  ///
  /// @param out_rig Point on the road in the cam0 rig frame, metres.
  /// @return `kInvalidArgument` for a pixel on or above the horizon, whose ray
  ///         never meets the plane. This is the ordinary case for the upper
  ///         half of the image, not an error worth logging.
  Status ImageToGroundRig(const Vec2& uv, Vec3& out_rig) const;

  /// Map a rig-frame point to bird's-eye raster indices.
  ///
  /// @return `kInvalidArgument` when the point falls outside the BEV window.
  static Status RigToBevPixel(const Vec3& p_rig, int& out_col, int& out_row);

  // --- Compound world/image/BEV transforms (shared by synthesis, offline viz,
  // ROS markers) ---

  /// Transform a world point into the rig frame of @p T_world_rig (Rᵀ·(p − t)).
  static Vec3 WorldToRig(const Mat44& T_world_rig, const Vec3& p_world);

  /// World point → image pixels at rig pose @p T_world_rig.
  ///
  /// @param min_z_rig Near-plane cutoff, metres. Points closer than this
  ///        project to wildly large pixel coordinates, so they are rejected.
  Status WorldToImage(const Mat44& T_world_rig, const Vec3& p_world,
                      Vec2& out_uv, double min_z_rig = 0.5) const;

  /// World point → BEV pixel indices at rig pose @p T_world_rig.
  static Status WorldToBevPixel(const Mat44& T_world_rig, const Vec3& p_world,
                                int& out_col, int& out_row);

  /// Image pixel → world point on the road plane at @p T_world_rig.
  Status ImageToWorldGround(const Mat44& T_world_rig, const Vec2& uv,
                            Vec3& out_world) const;

  double fx() const { return fx_; }
  double fy() const { return fy_; }
  double cx() const { return cx_; }
  double cy() const { return cy_; }

 private:
  double fx_{718.0};
  double fy_{718.0};
  double cx_{607.0};
  double cy_{185.0};
  double ground_height_m_{kDefaultGroundHeightM};
};

}  // namespace cam_loc::core

#endif  // CAM_LOC_CORE_PROJECTION_H_
