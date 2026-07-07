#pragma once

#include <cam_loc/core/bev_config.hpp>
#include <cam_loc/kitti/types.hpp>
#include <cam_loc/types/status.hpp>

namespace cam_loc::core {

/// Pinhole camera projection utilities.
///
/// Conventions: KITTI rectified cam0 is the rig frame (no extrinsic offset).
///   rig → image: u = fx·x/z + cx,  v = fy·y/z + cy  (points behind camera rejected)
///   image → rig ground: ray through pixel intersects Z = ground_z_rig plane
///   rig → BEV pixel: floor((x - kXMin)/mpp_x), floor((y - kYMin)/mpp_y)
/// Plane offsets (x, y, yaw) compose as SE(2) in the sampling-plane XY frame.
class Projection {
 public:
  explicit Projection(const kitti::Calibration& calib);

  /// Rig/world point (cam0 frame) to image pixels.
  Status projectRigToImage(const Vec3& p_rig, Vec2& out_uv) const;

  /// Image pixel + assumed ground height in rig frame to rig XY (inverse pinhole on Z plane).
  Status imageToGroundRig(const Vec2& uv, double ground_z_rig, Vec3& out_rig) const;

  /// Rig XY (Z=ground_z) to BEV pixel indices.
  static Status rigToBevPixel(const Vec3& p_rig, int& out_col, int& out_row);

  // --- Compound world/image/BEV transforms (shared by synthesis, offline viz, ROS markers) ---

  /// Transform a world point into the rig frame of @p T_world_rig (Rᵀ·(p − t)).
  static Vec3 worldToRig(const Mat44& T_world_rig, const Vec3& p_world);

  /// World point → image pixels at rig pose @p T_world_rig; rejects points with rig z ≤ min_z_rig.
  Status worldToImage(const Mat44& T_world_rig, const Vec3& p_world, Vec2& out_uv,
                      double min_z_rig = 0.5) const;

  /// World point → BEV pixel indices at rig pose @p T_world_rig.
  Status worldToBevPixel(const Mat44& T_world_rig, const Vec3& p_world, int& out_col,
                         int& out_row) const;

  /// Image pixel → world point on the rig ground plane (rig Z = ground_z_rig) at @p T_world_rig.
  Status imageToWorldGround(const Mat44& T_world_rig, const Vec2& uv, Vec3& out_world,
                            double ground_z_rig = 0.0) const;

  static Mat44 offsetToTransform(double x_m, double y_m, double yaw_rad);
  static Vec3 transformToOffset(const Mat44& T);

  double fx() const { return fx_; }
  double fy() const { return fy_; }
  double cx() const { return cx_; }
  double cy() const { return cy_; }

 private:
  double fx_{718.0};
  double fy_{718.0};
  double cx_{607.0};
  double cy_{185.0};
};

}  // namespace cam_loc::core
