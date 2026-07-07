// Unit tests for KITTI calib.txt parsing.
#include <cam_loc/kitti/calib_parser.hpp>

#include <gtest/gtest.h>

TEST(CalibParserTest, ParsesMinimalCalib) {
  const std::string path = TEST_DATA_DIR "/calib_minimal.txt";
  cam_loc::kitti::Calibration calib;
  ASSERT_EQ(cam_loc::kitti::parseCalibrationFile(path, calib), cam_loc::Status::kOk);
  EXPECT_DOUBLE_EQ(calib.P0(0, 0), 718.856);
  EXPECT_DOUBLE_EQ(calib.P0(1, 1), 718.856);
  EXPECT_DOUBLE_EQ(calib.Tr_velo_to_cam0(0, 3), -1.198459e-02);
}
