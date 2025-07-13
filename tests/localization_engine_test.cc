// End-to-end map matching: straight and turning, with pose accuracy asserted.
#include "cam_loc/core/localization_engine.h"

#include <gtest/gtest.h>

#include "cam_loc/core/frames.h"
#include "cam_loc/kitti/calib_parser.h"
#include "cam_loc/map/trajectory_corridor_map.h"
#include "cam_loc/perception/synthesize.h"

namespace {

using cam_loc::core::Frames;

/// Integrate a constant body-frame motion into a pose sequence.
///
/// @param yaw_rate_rad Heading change per frame. Zero gives a straight drive;
///        anything else exercises the rotating case the filter and the yaw
///        metric are easy to get wrong on.
std::vector<cam_loc::kitti::Pose> DrivePath(int frames, double step_m,
                                            double yaw_rate_rad) {
  std::vector<cam_loc::kitti::Pose> poses;
  poses.reserve(frames);
  cam_loc::Mat44 T = cam_loc::Mat44::Identity();
  for (int i = 0; i < frames; ++i) {
    cam_loc::kitti::Pose p;
    p.frame = i;
    p.T_world_cam0 = T;
    poses.push_back(p);
    T = T * Frames::OffsetToCam0Transform(step_m, 0.0, yaw_rate_rad);
  }
  return poses;
}

// A grid small enough to run a whole sequence in a unit test, and still wide
// enough to hold the drift these tests produce: +/-1.75 m and +/-1.5 deg about
// a plane that tracks the estimate. Cost is cubic in the cell counts, and this
// test drives every frame of a sequence through it.
cam_loc::LocalizationParams TestParams() {
  cam_loc::LocalizationParams params;
  params.grid.num_x = 15;
  params.grid.num_y = 15;
  params.grid.num_yaw = 7;
  params.grid.step_x_m = 0.25;
  params.grid.step_y_m = 0.25;
  params.grid.step_yaw_deg = 0.5;
  params.aggregation.window_size = 5;
  return params;
}

/// Oracle perception: the map as seen from the *ground-truth* pose.
///
/// It has to be the ground-truth pose and not the filter's estimate. Projecting
/// the map from the estimate makes the observation move with the estimate, so
/// the match reports zero error however far off it is -- a loop that cannot
/// fail and cannot detect anything.
cam_loc::kitti::FramePerception OraclePerception(
    const cam_loc::map::IMapLoader& map, const cam_loc::core::Projection& proj,
    const cam_loc::Mat44& T_world_gt, int frame) {
  cam_loc::kitti::MapChunk local;
  EXPECT_EQ(map.QueryLocalMap(T_world_gt, 50.0, local), cam_loc::Status::kOk);
  return cam_loc::perception::SynthesizeFromMap(local, proj, T_world_gt, frame);
}

/// Drive a path with oracle perception and return the worst position error.
///
/// The map is built over a longer route than the one driven. A map that stops
/// where the drive stops is not a property of the algorithm: the last stretch
/// has nothing ahead of the camera to match against, and what that measures is
/// the fixture. Real maps extend past the trip.
double WorstPositionError(int frames, double step_m, double yaw_rate_rad) {
  const auto poses = DrivePath(frames, step_m, yaw_rate_rad);
  const auto mapped_route = DrivePath(frames + 200, step_m, yaw_rate_rad);

  cam_loc::kitti::Calibration calib;
  EXPECT_EQ(cam_loc::kitti::ParseCalibrationFile(
                TEST_DATA_DIR "/calib_minimal.txt", calib),
            cam_loc::Status::kOk);

  cam_loc::map::CorridorMapOptions corridor_opts;
  corridor_opts.sample_step_m = 1.0;
  auto map = std::make_shared<cam_loc::map::TrajectoryCorridorMap>();
  EXPECT_EQ(map->BuildFromPoses(mapped_route, corridor_opts),
            cam_loc::Status::kOk);

  cam_loc::core::LocalizationEngine engine(TestParams());
  engine.set_map_loader(map);
  engine.SetCalibration(calib);

  const cam_loc::core::Projection proj(calib);
  double worst = 0.0;
  for (size_t f = 0; f < poses.size(); ++f) {
    cam_loc::kitti::Egomotion ego;
    EXPECT_EQ(cam_loc::kitti::BuildEgomotion(poses, static_cast<int>(f), ego),
              cam_loc::Status::kOk);
    const auto perception = OraclePerception(*map, proj, poses[f].T_world_cam0,
                                             static_cast<int>(f));
    EXPECT_EQ(engine.ProcessFrame(ego, perception), cam_loc::Status::kOk);

    const cam_loc::Vec3 err = engine.result().T_world_rig.block<3, 1>(0, 3) -
                              poses[f].T_world_cam0.block<3, 1>(0, 3);
    worst = std::max(worst, err.norm());
  }
  return worst;
}

}  // namespace

TEST(LocalizationEngineTest, MapMatchingWithSynthesizedPerception) {
  const auto poses = DrivePath(30, 0.5, 0.0);

  cam_loc::kitti::Calibration calib;
  ASSERT_EQ(cam_loc::kitti::ParseCalibrationFile(
                TEST_DATA_DIR "/calib_minimal.txt", calib),
            cam_loc::Status::kOk);

  cam_loc::map::CorridorMapOptions corridor_opts;
  corridor_opts.sample_step_m = 1.0;
  auto map = std::make_shared<cam_loc::map::TrajectoryCorridorMap>();
  ASSERT_EQ(map->BuildFromPoses(poses, corridor_opts), cam_loc::Status::kOk);

  cam_loc::core::LocalizationEngine engine(TestParams());
  engine.set_map_loader(map);
  engine.SetCalibration(calib);

  cam_loc::kitti::Egomotion ego;
  ASSERT_EQ(cam_loc::kitti::BuildEgomotion(poses, 10, ego),
            cam_loc::Status::kOk);

  const cam_loc::core::Projection proj(calib);
  const auto perception =
      OraclePerception(*map, proj, poses[10].T_world_cam0, 10);
  ASSERT_FALSE(perception.empty());
  ASSERT_EQ(engine.ProcessFrame(ego, perception), cam_loc::Status::kOk);
  EXPECT_TRUE(engine.result().valid);
  EXPECT_TRUE(engine.result().sampling_measurement_applied);
  EXPECT_LT(engine.result().aggregate_min_cost, 5.f);
}

// Perception is an input. With none, there is nothing to match and the filter
// coasts on the motion model -- it must not invent an observation from its own
// estimate and report a successful match.
TEST(LocalizationEngineTest, NoPerceptionMeansNoMapUpdate) {
  const auto poses = DrivePath(30, 0.5, 0.0);

  cam_loc::kitti::Calibration calib;
  ASSERT_EQ(cam_loc::kitti::ParseCalibrationFile(
                TEST_DATA_DIR "/calib_minimal.txt", calib),
            cam_loc::Status::kOk);
  auto map = std::make_shared<cam_loc::map::TrajectoryCorridorMap>();
  ASSERT_EQ(map->BuildFromPoses(poses, {}), cam_loc::Status::kOk);

  cam_loc::core::LocalizationEngine engine(TestParams());
  engine.set_map_loader(map);
  engine.SetCalibration(calib);

  cam_loc::kitti::Egomotion ego;
  ASSERT_EQ(cam_loc::kitti::BuildEgomotion(poses, 10, ego),
            cam_loc::Status::kOk);
  ASSERT_EQ(engine.ProcessFrame(ego, cam_loc::kitti::FramePerception{}),
            cam_loc::Status::kOk);
  EXPECT_TRUE(engine.result().valid);
  EXPECT_FALSE(engine.result().sampling_measurement_applied);
}

// The straight case is the one every earlier defect hid behind, so it is worth
// asserting the pose and not only that the pipeline ran.
TEST(LocalizationEngineTest, TracksAStraightDrive) {
  EXPECT_LT(WorstPositionError(40, 0.5, 0.0), 0.15);
}

// A turn is where the frame conventions and the filter's rotation handling stop
// agreeing by coincidence: the pose-grid axes have to follow the vehicle, and
// the filter's rotation residual and correction have to be on the same side.
TEST(LocalizationEngineTest, TracksATurn) {
  // ~0.6 deg per frame over 50 frames: a 29-degree curve.
  EXPECT_LT(WorstPositionError(50, 0.5, 0.01), 0.15);
}
