#ifndef CAM_LOC_TYPES_STATUS_H_
#define CAM_LOC_TYPES_STATUS_H_

/// Shared status helpers, Eigen aliases, and rigid-transform utilities.

#include <string>

#include <Eigen/Dense>

#include "cam_loc/types/status_codes.h"

namespace cam_loc {

using Mat34 = Eigen::Matrix<double, 3, 4, Eigen::RowMajor>;
using Mat44 = Eigen::Matrix4d;
using Mat66 = Eigen::Matrix<double, 6, 6>;
using Vec3 = Eigen::Vector3d;
using Vec2 = Eigen::Vector2d;

/// Build 4x4 from KITTI 3x4 row-major line.
Mat44 Mat34ToMat44(const Mat34& m);

/// Invert a rigid transform as `[Rᵀ | −Rᵀt]`.
///
/// @param T Assumed rigid; a matrix with scale or shear is inverted wrongly
///          rather than rejected.
Mat44 InvertRigid(const Mat44& T);

/// Relative transform T_curr_prev such that T_world_curr = T_world_prev *
/// T_curr_prev.
Mat44 RelativeTransform(const Mat44& T_world_prev, const Mat44& T_world_curr);

/// Vehicle heading of a cam0 rotation, measured about the world up axis.
///
/// @return Yaw in radians, in (−π, π]; zero for identity.
/// @note Thin wrapper over core::Frames::HeadingFromCam0Rotation, kept here
///       because the eval metrics include only this header.
double YawFromRotation(const Eigen::Matrix3d& R);

/// Resolve a cam0-frame vector onto vehicle axes.
///
/// @param v_cam0 Vector in cam0 (X right, Y down, Z forward).
/// @return The same vector as `(forward, left, up)`.
/// @note Thin wrapper over core::Frames::ToVehicle, kept here for the same
///       reason as YawFromRotation above.
Vec3 ToVehicleAxes(const Vec3& v_cam0);

/// Format sequence id as two digits (e.g. 0 -> "00").
std::string FormatSequenceId(int sequence);

}  // namespace cam_loc

#endif  // CAM_LOC_TYPES_STATUS_H_
