// Shared SE(3) math helpers: rigid inversion, relative transforms, yaw extraction.

#include <cam_loc/types/status.hpp>

#include <cmath>

namespace cam_loc {

Mat44 mat34ToMat44(const Mat34& m) {
  Mat44 T = Mat44::Identity();
  T.block<3, 4>(0, 0) = m;
  return T;
}

Mat44 invertRigid(const Mat44& T) {
  Mat44 inv = Mat44::Identity();
  const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  const Vec3 t = T.block<3, 1>(0, 3);
  inv.block<3, 3>(0, 0) = R.transpose();
  inv.block<3, 1>(0, 3) = -R.transpose() * t;
  return inv;
}

Mat44 relativeTransform(const Mat44& T_world_prev, const Mat44& T_world_curr) {
  return invertRigid(T_world_prev) * T_world_curr;
}

double yawFromRotation(const Eigen::Matrix3d& R) {
  // Heading about the world +Z axis, taken from R's first column. Correct for a Z-up vehicle
  // frame; on the KITTI cam0 optical frame (Y down, Z forward) this reads the wrong component —
  // a known limitation of the yaw-error metric on real sequences (see docs/OPEN_ITEMS.md).
  return std::atan2(R(1, 0), R(0, 0));
}

std::string formatSequenceId(int sequence) {
  char buf[8];
  snprintf(buf, sizeof(buf), "%02d", sequence);
  return std::string(buf);
}

}  // namespace cam_loc
