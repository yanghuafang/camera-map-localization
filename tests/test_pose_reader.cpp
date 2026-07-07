// Unit tests for poses file load and egomotion construction.
#include <cam_loc/kitti/calib_parser.hpp>

#include <gtest/gtest.h>

TEST(PoseReaderTest, LoadsPoses) {
  const std::string path = TEST_DATA_DIR "/poses_minimal.txt";
  std::vector<cam_loc::kitti::Pose> poses;
  ASSERT_EQ(cam_loc::kitti::loadPosesFile(path, poses), cam_loc::Status::kOk);
  ASSERT_EQ(poses.size(), 3u);
  EXPECT_EQ(poses[0].frame, 0);
  EXPECT_EQ(poses[1].frame, 1);
  EXPECT_DOUBLE_EQ(poses[0].T_world_cam0(0, 3), 0.0);
  EXPECT_DOUBLE_EQ(poses[1].T_world_cam0(0, 3), 1.0);

  cam_loc::kitti::Egomotion ego;
  ASSERT_EQ(cam_loc::kitti::buildEgomotion(poses, 1, ego), cam_loc::Status::kOk);
  EXPECT_NEAR(ego.T_curr_prev(0, 3), 1.0, 1e-9);
}
