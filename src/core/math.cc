// Shared SE(3) math helpers: rigid inversion, relative transforms, yaw
// extraction.

#include <cmath>

#include "cam_loc/core/frames.h"
#include "cam_loc/types/status.h"

namespace cam_loc {

Mat44 Mat34ToMat44(const Mat34& m) {
  Mat44 T = Mat44::Identity();
  T.block<3, 4>(0, 0) = m;
  return T;
}

Mat44 InvertRigid(const Mat44& T) {
  Mat44 inv = Mat44::Identity();
  const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  const Vec3 t = T.block<3, 1>(0, 3);
  inv.block<3, 3>(0, 0) = R.transpose();
  inv.block<3, 1>(0, 3) = -R.transpose() * t;
  return inv;
}

Mat44 RelativeTransform(const Mat44& T_world_prev, const Mat44& T_world_curr) {
  return InvertRigid(T_world_prev) * T_world_curr;
}

double YawFromRotation(const Eigen::Matrix3d& R) {
  return core::Frames::HeadingFromCam0Rotation(R);
}

Vec3 ToVehicleAxes(const Vec3& v_cam0) {
  return core::Frames::ToVehicle(v_cam0);
}

std::string FormatSequenceId(int sequence) {
  char buf[8];
  snprintf(buf, sizeof(buf), "%02d", sequence);
  return std::string(buf);
}

}  // namespace cam_loc
