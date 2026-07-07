// Pinhole projection (cam0 = rig) and SE(2) offset ↔ 4×4 transform helpers.

#include <cam_loc/core/projection.hpp>

#include <cmath>

namespace cam_loc::core {

Projection::Projection(const kitti::Calibration& calib) {
  const Eigen::Matrix3d K = calib.intrinsicCam0();
  fx_ = K(0, 0);
  fy_ = K(1, 1);
  cx_ = K(0, 2);
  cy_ = K(1, 2);
}

Status Projection::projectRigToImage(const Vec3& p_rig, Vec2& out_uv) const {
  if (p_rig.z() <= 1e-3) {
    return Status::kInvalidArgument;
  }
  out_uv.x() = fx_ * p_rig.x() / p_rig.z() + cx_;
  out_uv.y() = fy_ * p_rig.y() / p_rig.z() + cy_;
  return Status::kOk;
}

Status Projection::imageToGroundRig(const Vec2& uv, double ground_z_rig, Vec3& out_rig) const {
  const double x_norm = (uv.x() - cx_) / fx_;
  const double y_norm = (uv.y() - cy_) / fy_;
  if (std::abs(y_norm) < 1e-6) {
    return Status::kInvalidArgument;
  }
  const double z = ground_z_rig;
  out_rig.z() = z;
  out_rig.x() = x_norm * z;
  out_rig.y() = y_norm * z;
  return Status::kOk;
}

Status Projection::rigToBevPixel(const Vec3& p_rig, int& out_col, int& out_row) {
  // col ↔ rig X (forward), row ↔ rig Y (left); pixel center at (col+0.5, row+0.5)
  const double col = (p_rig.x() - BevConfig::kXMin) / BevConfig::metersPerPixelX();
  const double row = (p_rig.y() - BevConfig::kYMin) / BevConfig::metersPerPixelY();
  out_col = static_cast<int>(std::floor(col));
  out_row = static_cast<int>(std::floor(row));
  if (out_col < 0 || out_col >= BevConfig::kImageWidth || out_row < 0 ||
      out_row >= BevConfig::kImageHeight) {
    return Status::kInvalidArgument;
  }
  return Status::kOk;
}

Vec3 Projection::worldToRig(const Mat44& T_world_rig, const Vec3& p_world) {
  const Eigen::Matrix3d R = T_world_rig.block<3, 3>(0, 0);
  const Vec3 t = T_world_rig.block<3, 1>(0, 3);
  return R.transpose() * (p_world - t);
}

Status Projection::worldToImage(const Mat44& T_world_rig, const Vec3& p_world, Vec2& out_uv,
                                double min_z_rig) const {
  const Vec3 p_rig = worldToRig(T_world_rig, p_world);
  if (p_rig.z() <= min_z_rig) {
    return Status::kInvalidArgument;
  }
  return projectRigToImage(p_rig, out_uv);
}

Status Projection::worldToBevPixel(const Mat44& T_world_rig, const Vec3& p_world, int& out_col,
                                   int& out_row) const {
  const Vec3 p_rig = worldToRig(T_world_rig, p_world);
  return rigToBevPixel(p_rig, out_col, out_row);
}

Status Projection::imageToWorldGround(const Mat44& T_world_rig, const Vec2& uv, Vec3& out_world,
                                      double ground_z_rig) const {
  Vec3 p_rig;
  const Status st = imageToGroundRig(uv, ground_z_rig, p_rig);
  if (st != Status::kOk) return st;
  const Eigen::Matrix3d R = T_world_rig.block<3, 3>(0, 0);
  const Vec3 t = T_world_rig.block<3, 1>(0, 3);
  out_world = R * p_rig + t;
  return Status::kOk;
}

Mat44 Projection::offsetToTransform(double x_m, double y_m, double yaw_rad) {
  // SE(2) in plane XY: translation (x, y), yaw about Z (right-handed)
  Mat44 T = Mat44::Identity();
  const double c = std::cos(yaw_rad);
  const double s = std::sin(yaw_rad);
  T(0, 0) = c;
  T(0, 1) = -s;
  T(1, 0) = s;
  T(1, 1) = c;
  T(0, 3) = x_m;
  T(1, 3) = y_m;
  return T;
}

Vec3 Projection::transformToOffset(const Mat44& T) {
  return Vec3(T(0, 3), T(1, 3), std::atan2(T(1, 0), T(0, 0)));
}

}  // namespace cam_loc::core
