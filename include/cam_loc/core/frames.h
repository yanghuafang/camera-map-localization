#ifndef CAM_LOC_CORE_FRAMES_H_
#define CAM_LOC_CORE_FRAMES_H_

/// The two coordinate frames this project uses, and the fixed rotation between
/// them.

#include <cmath>

#include "cam_loc/types/status.h"

namespace cam_loc::core {

/// Axis conventions relating the sensor frame to the frame the search runs in.
///
/// **cam0** is KITTI's rectified left camera: X right, Y down, Z forward. Every
/// pose, every map point and every world coordinate in this project is in cam0,
/// because that is the frame KITTI odometry poses are published in.
///
/// **vehicle** is X forward, Y left, Z up. Nothing is *stored* in it; it exists
/// so that the quantities a localizer reasons about have their usual meaning —
/// a pose-grid offset of (1, 0, 0) is one metre forward, and yaw is heading
/// rather than roll about the optical axis.
///
/// The two differ by a fixed axis permutation, with no translation: cam0 sits
/// at the vehicle origin by definition here, and the one distance that does
/// matter — the camera's height above the road — is carried by Projection as
/// `ground_height_m` rather than folded in as an extrinsic.
///
/// Everything below is `constexpr`-friendly and allocation-free; these are
/// called once per grid cell.
struct Frames {
  /// Rotation taking a vehicle-frame vector into cam0.
  ///
  /// Columns are the vehicle axes expressed in cam0: forward → +Z, left → −X,
  /// up → −Y.
  static Eigen::Matrix3d RotCam0Vehicle() {
    Eigen::Matrix3d R;
    // clang-format off
    R <<  0.0, -1.0,  0.0,
          0.0,  0.0, -1.0,
          1.0,  0.0,  0.0;
    // clang-format on
    return R;
  }

  /// World "up" expressed in cam0. Y points down, so up is −Y.
  static Vec3 UpCam0() { return Vec3(0.0, -1.0, 0.0); }

  /// cam0 point → vehicle point.
  static Vec3 ToVehicle(const Vec3& p_cam0) {
    return Vec3(p_cam0.z(), -p_cam0.x(), -p_cam0.y());
  }

  /// vehicle point → cam0 point.
  static Vec3 ToCam0(const Vec3& p_vehicle) {
    return Vec3(-p_vehicle.y(), -p_vehicle.z(), p_vehicle.x());
  }

  /// Pose-grid offset → the cam0 transform that applies it.
  ///
  /// The offset is SE(2) in the vehicle ground plane: @p x_m forward, @p y_m
  /// left, @p yaw_rad about up. Composed on the right of a cam0 pose
  /// (`T_world_hyp = T_world_cam0 · this`), it moves the hypothesis the way a
  /// vehicle moves.
  ///
  /// This is `R_cam0_vehicle · SE2(x, y, yaw) · R_vehicle_cam0` written out:
  /// the translation becomes (−y, 0, x) and the rotation becomes a rotation
  /// about cam0 −Y. It is spelled as a closed form rather than three matrix
  /// products because it runs once per grid cell, and because the CUDA kernels
  /// have to reproduce exactly this.
  static Mat44 OffsetToCam0Transform(double x_m, double y_m, double yaw_rad) {
    const double c = std::cos(yaw_rad);
    const double s = std::sin(yaw_rad);
    Mat44 T = Mat44::Identity();
    T(0, 0) = c;
    T(0, 2) = -s;
    T(2, 0) = s;
    T(2, 2) = c;
    T(0, 3) = -y_m;
    T(2, 3) = x_m;
    return T;
  }

  /// Inverse of OffsetToCam0Transform.
  ///
  /// @return The packed offset `(x_m, y_m, yaw_rad)` — forward, left, heading.
  static Vec3 Cam0TransformToOffset(const Mat44& T) {
    return Vec3(T(2, 3), -T(0, 3), std::atan2(T(2, 0), T(0, 0)));
  }

  /// Heading of a cam0 rotation, measured about the vehicle up axis.
  ///
  /// The vehicle forward axis in world coordinates is `R · (0, 0, 1)`, i.e.
  /// R's third column; projecting it onto the ground plane leaves the heading.
  ///
  /// @return Radians in (−π, π]; zero when the rotation is identity.
  static double HeadingFromCam0Rotation(const Eigen::Matrix3d& R) {
    return std::atan2(-R(0, 2), R(2, 2));
  }
};

}  // namespace cam_loc::core

#endif  // CAM_LOC_CORE_FRAMES_H_
