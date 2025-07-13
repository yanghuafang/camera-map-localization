// The cam0 <-> vehicle axis convention, which everything geometric depends on.
#include "cam_loc/core/frames.h"

#include <gtest/gtest.h>

namespace {

using cam_loc::Mat44;
using cam_loc::Vec3;
using cam_loc::core::Frames;

constexpr double kEps = 1e-12;

}  // namespace

// cam0 is X right, Y down, Z forward; the vehicle frame is X forward, Y left,
// Z up. Everything else here is a consequence of these six mappings.
TEST(FramesTest, AxisConvention) {
  EXPECT_TRUE(Frames::ToCam0(Vec3(1, 0, 0)).isApprox(Vec3(0, 0, 1), kEps))
      << "vehicle forward should be cam0 +Z";
  EXPECT_TRUE(Frames::ToCam0(Vec3(0, 1, 0)).isApprox(Vec3(-1, 0, 0), kEps))
      << "vehicle left should be cam0 -X";
  EXPECT_TRUE(Frames::ToCam0(Vec3(0, 0, 1)).isApprox(Vec3(0, -1, 0), kEps))
      << "vehicle up should be cam0 -Y";
  EXPECT_TRUE(Frames::UpCam0().isApprox(Vec3(0, -1, 0), kEps));
}

TEST(FramesTest, ToVehicleIsTheInverse) {
  const Vec3 p(1.5, -2.25, 3.75);
  EXPECT_TRUE(Frames::ToVehicle(Frames::ToCam0(p)).isApprox(p, kEps));
  EXPECT_TRUE(Frames::ToCam0(Frames::ToVehicle(p)).isApprox(p, kEps));
}

// A pose-grid offset has to move the hypothesis the way a vehicle moves. The
// grid searched cam0 XY before this existed, which meant its second axis
// swept camera height and its "yaw" was roll about the optical axis.
TEST(FramesTest, OffsetMovesAlongVehicleAxes) {
  const Mat44 forward = Frames::OffsetToCam0Transform(2.0, 0.0, 0.0);
  const Vec3 forward_t = forward.block<3, 1>(0, 3);
  EXPECT_TRUE(forward_t.isApprox(Vec3(0, 0, 2), 1e-9))
      << "a forward offset must translate along cam0 +Z";

  const Mat44 left = Frames::OffsetToCam0Transform(0.0, 1.0, 0.0);
  const Vec3 left_t = left.block<3, 1>(0, 3);
  EXPECT_TRUE(left_t.isApprox(Vec3(-1, 0, 0), 1e-9))
      << "a left offset must translate along cam0 -X";

  // A quarter turn to the left takes forward (cam0 +Z) onto left (cam0 -X).
  const Mat44 turn = Frames::OffsetToCam0Transform(0.0, 0.0, M_PI / 2.0);
  const Eigen::Matrix3d turn_r = turn.block<3, 3>(0, 0);
  const Vec3 turned = turn_r * Vec3(0, 0, 1);
  EXPECT_TRUE(turned.isApprox(Vec3(-1, 0, 0), 1e-9))
      << "yaw must rotate about the vehicle up axis, not the optical axis";
}

TEST(FramesTest, OffsetRoundTrip) {
  const Vec3 offset(1.25, -0.75, 0.15);
  const Mat44 T =
      Frames::OffsetToCam0Transform(offset.x(), offset.y(), offset.z());
  EXPECT_TRUE(Frames::Cam0TransformToOffset(T).isApprox(offset, 1e-9));
}

TEST(FramesTest, HeadingMatchesTheYawThatProducedIt) {
  EXPECT_NEAR(Frames::HeadingFromCam0Rotation(Eigen::Matrix3d::Identity()), 0.0,
              1e-12);
  for (double yaw : {-1.0, -0.25, 0.1, 0.9}) {
    const Mat44 T = Frames::OffsetToCam0Transform(0.0, 0.0, yaw);
    const Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    EXPECT_NEAR(Frames::HeadingFromCam0Rotation(R), yaw, 1e-9);
  }
}
