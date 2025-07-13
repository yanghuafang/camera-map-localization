#include <gtest/gtest.h>

#include "cam_loc/core/localization_engine.h"
#include "cam_loc/kitti/calib_parser.h"
#include "cam_loc/map/trajectory_corridor_map.h"
#include "cam_loc/perception/synthesize.h"
#include "cam_loc/viz/frame_viz.h"

TEST(VizTest, RenderFrameFromEngineDebug) {
  std::vector<cam_loc::kitti::Pose> poses;
  for (int i = 0; i < 30; ++i) {
    cam_loc::kitti::Pose p;
    p.frame = i;
    p.T_world_cam0 = cam_loc::Mat44::Identity();
    p.T_world_cam0(2, 3) = static_cast<double>(i) * 0.5;
    poses.push_back(p);
  }

  cam_loc::kitti::Calibration calib;
  ASSERT_EQ(cam_loc::kitti::ParseCalibrationFile(
                TEST_DATA_DIR "/calib_minimal.txt", calib),
            cam_loc::Status::kOk);

  cam_loc::map::CorridorMapOptions corridor_opts;
  corridor_opts.sample_step_m = 1.0;
  auto map = std::make_shared<cam_loc::map::TrajectoryCorridorMap>();
  ASSERT_EQ(map->BuildFromPoses(poses, corridor_opts), cam_loc::Status::kOk);

  cam_loc::LocalizationParams params;
  params.grid.num_x = 7;
  params.grid.num_y = 7;
  params.grid.num_yaw = 5;
  params.aggregation.window_size = 3;

  cam_loc::core::LocalizationEngine engine(params);
  engine.set_map_loader(map);
  engine.SetCalibration(calib);

  const cam_loc::core::Projection proj(calib);
  for (int f = 0; f < 10; ++f) {
    cam_loc::kitti::Egomotion ego;
    ASSERT_EQ(cam_loc::kitti::BuildEgomotion(poses, f, ego),
              cam_loc::Status::kOk);
    engine.set_debug_capture(f == 9);
    // Oracle perception, projected at the ground-truth pose.
    cam_loc::kitti::MapChunk local;
    ASSERT_EQ(map->QueryLocalMap(poses[f].T_world_cam0, 50.0, local),
              cam_loc::Status::kOk);
    const auto perception = cam_loc::perception::SynthesizeFromMap(
        local, proj, poses[f].T_world_cam0, f);
    ASSERT_EQ(engine.ProcessFrame(ego, perception), cam_loc::Status::kOk);
  }

  ASSERT_TRUE(engine.debug_snapshot().valid);

  cam_loc::core::Projection projection(calib);
  cam_loc::viz::FrameVizInput input;
  input.frame = 9;
  input.debug = &engine.debug_snapshot();
  input.projection = &projection;
  input.result = &engine.result();
  input.gt_pose = &poses[9];

  const std::string out_dir = testing::TempDir() + "cam_loc_viz_test";
  cam_loc::viz::FrameVizOutput output;
  ASSERT_EQ(cam_loc::viz::RenderFrameViz(input, out_dir, output),
            cam_loc::Status::kOk);
  EXPECT_GE(output.written_files.size(), 5u);
}

TEST(VizTest, ResolveImagePathFormat) {
  const std::string path =
      cam_loc::kitti::ResolveImagePath("/data/kitti", 0, 42);
  EXPECT_EQ(path, "/data/kitti/dataset/sequences/00/image_0/000042.png");
}
