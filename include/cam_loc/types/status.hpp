#pragma once

/// Shared status helpers, Eigen aliases, and rigid-transform utilities.

#include <cam_loc/types/status_codes.hpp>

#include <Eigen/Dense>
#include <string>

namespace cam_loc {

/// Human-readable label for a Status value (logging / CLI output).
inline const char* toString(Status s) {
  switch (s) {
    case Status::kOk:
      return "Ok";
    case Status::kInvalidArgument:
      return "InvalidArgument";
    case Status::kIoError:
      return "IoError";
    case Status::kNotFound:
      return "NotFound";
    case Status::kNotImplemented:
      return "NotImplemented";
    case Status::kCudaError:
      return "CudaError";
  }
  return "Unknown";
}

using Mat34 = Eigen::Matrix<double, 3, 4, Eigen::RowMajor>;
using Mat44 = Eigen::Matrix4d;
using Mat66 = Eigen::Matrix<double, 6, 6>;
using Vec3 = Eigen::Vector3d;
using Vec2 = Eigen::Vector2d;

/// Build 4x4 from KITTI 3x4 row-major line.
Mat44 mat34ToMat44(const Mat34& m);

/// Invert rigid transform (R|t).
Mat44 invertRigid(const Mat44& T);

/// Relative transform T_curr_prev such that T_world_curr = T_world_prev * T_curr_prev.
Mat44 relativeTransform(const Mat44& T_world_prev, const Mat44& T_world_curr);

/// Yaw (rad) from rotation matrix assuming flat ground (Z-up world).
double yawFromRotation(const Eigen::Matrix3d& R);

/// Format sequence id as two digits (e.g. 0 -> "00").
std::string formatSequenceId(int sequence);

}  // namespace cam_loc
