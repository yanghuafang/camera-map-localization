// Unit tests for LiDAR label projection to image polylines.
#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>

#include "cam_loc/kitti/calib_parser.h"
#include "cam_loc/semantic_kitti/lidar_project.h"

namespace {

void WriteVelodyneBin(const std::string& path,
                      const std::vector<cam_loc::Vec3>& pts) {
  std::ofstream out(path, std::ios::binary);
  for (const auto& p : pts) {
    const float row[4] = {static_cast<float>(p.x()), static_cast<float>(p.y()),
                          static_cast<float>(p.z()), 0.f};
    out.write(reinterpret_cast<const char*>(row), sizeof(row));
  }
}

void WriteLabels(const std::string& path, const std::vector<uint16_t>& labels) {
  std::ofstream out(path, std::ios::binary);
  for (uint16_t sem : labels) {
    const uint32_t raw = sem;
    out.write(reinterpret_cast<const char*>(&raw), sizeof(raw));
  }
}

}  // namespace

TEST(SemanticLidarTest, ProjectsLanePointsToPerception) {
  cam_loc::kitti::Calibration calib;
  ASSERT_EQ(cam_loc::kitti::ParseCalibrationFile(
                TEST_DATA_DIR "/calib_minimal.txt", calib),
            cam_loc::Status::kOk);

  std::vector<cam_loc::Vec3> pts;
  std::vector<uint16_t> labels;
  for (int i = 0; i < 80; ++i) {
    pts.emplace_back(20.0 + i * 0.2, 0.0, -1.6);
    labels.push_back(cam_loc::semantic_kitti::kLaneMarking);
  }

  cam_loc::semantic_kitti::PreprocessOptions opts;
  opts.frame = 0;
  opts.scan_stride = 2;
  opts.min_run_length = 4;

  cam_loc::kitti::FramePerception perception;
  ASSERT_EQ(cam_loc::semantic_kitti::ProjectLidarLabelsToPerception(
                calib, pts, labels, opts, perception),
            cam_loc::Status::kOk);
  EXPECT_GE(static_cast<size_t>(
                perception.CountOf(cam_loc::kitti::PolylineType::kLaneSolid)),
            1u);
}

TEST(SemanticLidarTest, LoadAndProjectFrameFiles) {
  const std::string root = TEST_DATA_DIR "/semantic_lidar_mini";
  const std::string seq_dir = root + "/dataset/sequences/00";
  std::filesystem::create_directories(seq_dir + "/velodyne");
  std::filesystem::create_directories(seq_dir + "/labels");

  cam_loc::kitti::Calibration calib;
  ASSERT_EQ(cam_loc::kitti::ParseCalibrationFile(
                TEST_DATA_DIR "/calib_minimal.txt", calib),
            cam_loc::Status::kOk);

  std::vector<cam_loc::Vec3> pts;
  std::vector<uint16_t> labels;
  for (int i = 0; i < 60; ++i) {
    pts.emplace_back(8.0 + i * 0.3, -1.2, -1.5);
    labels.push_back(cam_loc::semantic_kitti::kLaneMarking);
  }
  WriteVelodyneBin(seq_dir + "/velodyne/000000.bin", pts);
  WriteLabels(seq_dir + "/labels/000000.label", labels);

  cam_loc::semantic_kitti::PreprocessOptions opts;
  cam_loc::kitti::FramePerception perception;
  ASSERT_EQ(cam_loc::semantic_kitti::ProjectFrameFromFiles(root, 0, 0, calib,
                                                           opts, perception),
            cam_loc::Status::kOk);
  EXPECT_GE(static_cast<size_t>(
                perception.CountOf(cam_loc::kitti::PolylineType::kLaneSolid)),
            1u);
}
