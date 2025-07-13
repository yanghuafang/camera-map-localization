// Perception JSON I/O: one "features" list, each entry carrying its class.
#include <gtest/gtest.h>

#include "cam_loc/kitti/calib_parser.h"

TEST(PerceptionJsonTest, LoadsPolylines) {
  const std::string path = TEST_DATA_DIR "/perception_minimal.json";
  cam_loc::kitti::FramePerception p;
  ASSERT_EQ(cam_loc::kitti::LoadPerceptionJson(path, p), cam_loc::Status::kOk);
  EXPECT_EQ(p.frame, 0);
  ASSERT_EQ(p.features.size(), 2u);
  EXPECT_EQ(p.features[0].type, cam_loc::kitti::PolylineType::kLaneSolid);
  ASSERT_EQ(p.features[0].points.size(), 2u);
  EXPECT_DOUBLE_EQ(p.features[0].points[0].x(), 100.0);
  // Classes beyond the two the format started with round-trip as themselves.
  EXPECT_EQ(p.features[1].type, cam_loc::kitti::PolylineType::kPole);
  EXPECT_EQ(p.CountOf(cam_loc::kitti::PolylineType::kPole), 1);
}
