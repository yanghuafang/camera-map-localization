// Pinhole projection (cam0 = rig), inverse perspective onto the road plane,
// and the bird's-eye raster mapping.

#include "cam_loc/core/projection.h"

#include <cmath>

namespace cam_loc::core {

Projection::Projection(const kitti::Calibration& calib) {
  const Eigen::Matrix3d K = calib.IntrinsicCam0();
  fx_ = K(0, 0);
  fy_ = K(1, 1);
  cx_ = K(0, 2);
  cy_ = K(1, 2);
}

Status Projection::ProjectRigToImage(const Vec3& p_rig, Vec2& out_uv) const {
  if (p_rig.z() <= 1e-3) {
    return Status::kInvalidArgument;
  }
  out_uv.x() = fx_ * p_rig.x() / p_rig.z() + cx_;
  out_uv.y() = fy_ * p_rig.y() / p_rig.z() + cy_;
  return Status::kOk;
}

Status Projection::ImageToGroundRig(const Vec2& uv, Vec3& out_rig) const {
  // Ray through the pixel, in normalized camera coordinates: (x_n·t, y_n·t, t).
  // The road is the plane y = ground_height_m (cam0 Y points down), so the ray
  // meets it at t = h / y_n. A pixel on or above the horizon has y_n <= 0 and
  // never does -- that is most of the upper image, so it is a rejection rather
  // than a failure.
  const double x_norm = (uv.x() - cx_) / fx_;
  const double y_norm = (uv.y() - cy_) / fy_;
  if (y_norm <= 1e-6) {
    return Status::kInvalidArgument;
  }
  const double z = ground_height_m_ / y_norm;
  out_rig.x() = x_norm * z;
  out_rig.y() = ground_height_m_;
  out_rig.z() = z;
  return Status::kOk;
}

Status Projection::RigToBevPixel(const Vec3& p_rig, int& out_col,
                                 int& out_row) {
  // The raster is a vehicle-frame top-down view: column ↔ forward, row ↔ left.
  const Vec3 p_veh = Frames::ToVehicle(p_rig);
  const double col =
      (p_veh.x() - BevConfig::kForwardMinM) / BevConfig::MetersPerPixelX();
  const double row =
      (p_veh.y() - BevConfig::kLeftMinM) / BevConfig::MetersPerPixelY();
  out_col = static_cast<int>(std::floor(col));
  out_row = static_cast<int>(std::floor(row));
  if (out_col < 0 || out_col >= BevConfig::kImageWidth || out_row < 0 ||
      out_row >= BevConfig::kImageHeight) {
    return Status::kInvalidArgument;
  }
  return Status::kOk;
}

Vec3 Projection::WorldToRig(const Mat44& T_world_rig, const Vec3& p_world) {
  const Eigen::Matrix3d R = T_world_rig.block<3, 3>(0, 0);
  const Vec3 t = T_world_rig.block<3, 1>(0, 3);
  return R.transpose() * (p_world - t);
}

Status Projection::WorldToImage(const Mat44& T_world_rig, const Vec3& p_world,
                                Vec2& out_uv, double min_z_rig) const {
  const Vec3 p_rig = WorldToRig(T_world_rig, p_world);
  if (p_rig.z() <= min_z_rig) {
    return Status::kInvalidArgument;
  }
  return ProjectRigToImage(p_rig, out_uv);
}

Status Projection::WorldToBevPixel(const Mat44& T_world_rig,
                                   const Vec3& p_world, int& out_col,
                                   int& out_row) {
  const Vec3 p_rig = WorldToRig(T_world_rig, p_world);
  return RigToBevPixel(p_rig, out_col, out_row);
}

Status Projection::ImageToWorldGround(const Mat44& T_world_rig, const Vec2& uv,
                                      Vec3& out_world) const {
  Vec3 p_rig;
  const Status st = ImageToGroundRig(uv, p_rig);
  if (st != Status::kOk) return st;
  const Eigen::Matrix3d R = T_world_rig.block<3, 3>(0, 0);
  const Vec3 t = T_world_rig.block<3, 1>(0, 3);
  out_world = R * p_rig + t;
  return Status::kOk;
}

}  // namespace cam_loc::core
