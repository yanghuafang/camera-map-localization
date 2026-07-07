// Unit tests for perception JSON I/O.
#include <cam_loc/kitti/calib_parser.hpp>

#include <gtest/gtest.h>

TEST(PerceptionJsonTest, LoadsPolylines) {
  const std::string path = TEST_DATA_DIR "/perception_minimal.json";
  cam_loc::kitti::FramePerception p;
  ASSERT_EQ(cam_loc::kitti::loadPerceptionJson(path, p), cam_loc::Status::kOk);
  EXPECT_EQ(p.frame, 0);
  ASSERT_EQ(p.lane_lines.size(), 1u);
  ASSERT_EQ(p.lane_lines[0].points.size(), 2u);
  EXPECT_DOUBLE_EQ(p.lane_lines[0].points[0].x(), 100.0);
}
