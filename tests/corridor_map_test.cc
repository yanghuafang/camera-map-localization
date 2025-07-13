// Geometry of the synthetic corridor map: which way "left" is, where the road
// surface is, and that upright landmarks are emitted.
#include <gtest/gtest.h>

#include "cam_loc/core/frames.h"
#include "cam_loc/kitti/calib_parser.h"
#include "cam_loc/map/trajectory_corridor_map.h"

namespace {

using cam_loc::core::Frames;

/// A straight drive along cam0 +Z (forward) at 1 m per frame.
std::vector<cam_loc::kitti::Pose> StraightPath(int frames) {
  std::vector<cam_loc::kitti::Pose> poses;
  poses.reserve(frames);
  for (int i = 0; i < frames; ++i) {
    cam_loc::kitti::Pose p;
    p.frame = i;
    p.T_world_cam0 = cam_loc::Mat44::Identity();
    p.T_world_cam0(2, 3) = static_cast<double>(i);
    poses.push_back(p);
  }
  return poses;
}

int CountType(const cam_loc::kitti::MapChunk& map,
              cam_loc::kitti::PolylineType type) {
  int n = 0;
  for (const auto& pl : map.polylines) {
    if (pl.type == type) ++n;
  }
  return n;
}

}  // namespace

TEST(CorridorMapTest, BuildFromPoses) {
  const auto poses = StraightPath(60);
  cam_loc::map::TrajectoryCorridorMap map;
  cam_loc::map::CorridorMapOptions opts;
  opts.sample_step_m = 1.0;
  ASSERT_EQ(map.BuildFromPoses(poses, opts), cam_loc::Status::kOk);

  cam_loc::kitti::MapChunk local;
  ASSERT_EQ(map.QueryLocalMap(poses[30].T_world_cam0, 20.0, local),
            cam_loc::Status::kOk);
  EXPECT_GE(local.polylines.size(), 1u);
}

// The lane boundaries used to come out one lane-width *above and below* the
// camera: the lateral direction was taken as tangent x (0,0,1), which is the
// tangent itself on a straight cam0 drive, so the degenerate fallback picked
// the vertical axis. Everything downstream still agreed with itself, which is
// why only the geometry catches it.
TEST(CorridorMapTest, LaneBoundariesAreLateralNotVertical) {
  const auto poses = StraightPath(40);
  cam_loc::map::TrajectoryCorridorMap map;
  cam_loc::map::CorridorMapOptions opts;
  opts.sample_step_m = 1.0;
  ASSERT_EQ(map.BuildFromPoses(poses, opts), cam_loc::Status::kOk);

  bool saw_left = false;
  bool saw_right = false;
  for (const auto& pl : map.map().polylines) {
    if (pl.type != cam_loc::kitti::PolylineType::kLaneSolid) continue;
    // Vehicle frame: x forward, y left, z up. Offsets are relative to the
    // camera, which travels along the world origin line.
    const cam_loc::Vec3 v = Frames::ToVehicle(pl.points.front());
    EXPECT_NEAR(std::abs(v.y()), opts.half_width_m, 1e-6)
        << "boundary is not one half-width to the side";
    EXPECT_NEAR(v.z(), -opts.ground_height_m, 1e-6)
        << "boundary is not on the road surface";
    if (v.y() > 0.0) saw_left = true;
    if (v.y() < 0.0) saw_right = true;
  }
  EXPECT_TRUE(saw_left);
  EXPECT_TRUE(saw_right);
}

// Lane lines run parallel to travel and so cannot pin along-track position;
// upright landmarks are what break that aliasing, and the map has to carry
// them for the matcher to have anything to use.
TEST(CorridorMapTest, EmitsUprightLandmarks) {
  const auto poses = StraightPath(120);
  cam_loc::map::TrajectoryCorridorMap map;
  cam_loc::map::CorridorMapOptions opts;
  opts.sample_step_m = 1.0;
  ASSERT_EQ(map.BuildFromPoses(poses, opts), cam_loc::Status::kOk);

  EXPECT_GE(CountType(map.map(), cam_loc::kitti::PolylineType::kPole), 5);
  EXPECT_GE(CountType(map.map(), cam_loc::kitti::PolylineType::kSign), 1);

  for (const auto& pl : map.map().polylines) {
    if (pl.type != cam_loc::kitti::PolylineType::kPole) continue;
    const cam_loc::Vec3 base = Frames::ToVehicle(pl.points.front());
    const cam_loc::Vec3 top = Frames::ToVehicle(pl.points.back());
    EXPECT_NEAR(base.z(), -opts.ground_height_m, 1e-6) << "pole base off-road";
    EXPECT_NEAR(top.z(), opts.pole_height_m - opts.ground_height_m, 1e-6)
        << "pole does not stand up";
    EXPECT_NEAR(base.x(), top.x(), 1e-6) << "pole is not vertical";
  }
}
