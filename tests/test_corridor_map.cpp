// Unit tests for TrajectoryCorridorMap from GT poses.
#include <cam_loc/map/trajectory_corridor_map.hpp>
#include <cam_loc/kitti/calib_parser.hpp>

#include <gtest/gtest.h>

TEST(CorridorMapTest, BuildFromPoses) {
  std::vector<cam_loc::kitti::Pose> poses;
  for (int i = 0; i < 5; ++i) {
    cam_loc::kitti::Pose p;
    p.frame = i;
    p.T_world_cam0 = cam_loc::Mat44::Identity();
    p.T_world_cam0(2, 3) = static_cast<double>(i);
    poses.push_back(p);
  }

  cam_loc::map::TrajectoryCorridorMap map;
  ASSERT_EQ(map.buildFromPoses(poses, 1.75, 1.0), cam_loc::Status::kOk);

  cam_loc::kitti::MapChunk local;
  ASSERT_EQ(map.queryLocalMap(poses[2].T_world_cam0, 5.0, local), cam_loc::Status::kOk);
  EXPECT_GE(local.polylines.size(), 1u);
}
