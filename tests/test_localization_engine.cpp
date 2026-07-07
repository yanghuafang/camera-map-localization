// Integration test: map matching with synthesized perception.
#include <cam_loc/core/localization_engine.hpp>
#include <cam_loc/kitti/calib_parser.hpp>
#include <cam_loc/map/trajectory_corridor_map.hpp>
#include <cam_loc/perception/synthesize.hpp>

#include <gtest/gtest.h>

TEST(LocalizationEngineTest, MapMatchingWithSynthesizedPerception) {
  std::vector<cam_loc::kitti::Pose> poses;
  for (int i = 0; i < 30; ++i) {
    cam_loc::kitti::Pose p;
    p.frame = i;
    p.T_world_cam0 = cam_loc::Mat44::Identity();
    p.T_world_cam0(2, 3) = static_cast<double>(i) * 0.5;
    poses.push_back(p);
  }

  cam_loc::kitti::Calibration calib;
  ASSERT_EQ(cam_loc::kitti::parseCalibrationFile(TEST_DATA_DIR "/calib_minimal.txt", calib),
            cam_loc::Status::kOk);

  auto map = std::make_shared<cam_loc::map::TrajectoryCorridorMap>();
  ASSERT_EQ(map->buildFromPoses(poses, 1.75, 1.0), cam_loc::Status::kOk);

  cam_loc::LocalizationParams params;
  params.grid.num_x = 11;
  params.grid.num_y = 11;
  params.grid.num_yaw = 5;
  params.grid.step_x_m = 0.25;
  params.grid.step_y_m = 0.25;
  params.aggregation.window_size = 5;

  cam_loc::core::LocalizationEngine engine(params);
  engine.setMapLoader(map);
  engine.setCalibration(calib);

  cam_loc::kitti::Egomotion ego;
  ASSERT_EQ(cam_loc::kitti::buildEgomotion(poses, 10, ego), cam_loc::Status::kOk);

  cam_loc::kitti::FramePerception empty;
  ASSERT_EQ(engine.processFrame(ego, empty), cam_loc::Status::kOk);
  EXPECT_TRUE(engine.result().valid);
  EXPECT_TRUE(engine.result().perception_synthesized);
  EXPECT_TRUE(engine.result().sampling_measurement_applied);
  EXPECT_LT(engine.result().aggregate_min_cost, 5.f);
}
