// Geometry of the synthetic corridor map: which way "left" is, where the road
// surface is, and that upright landmarks are emitted.
#include <cmath>

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
  // The shape of the corridor, not the survey of it: a displaced map is still
  // a correctly shaped one, and this test is about the shape.
  opts.survey_error = {};
  opts.survey_error.lateral_sigma_m = 0.0;
  opts.survey_error.longitudinal_sigma_m = 0.0;
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

// The map is a survey of the world, and a survey is wrong. Without that
// difference the oracle projects the same geometry the matcher scores, the
// error appears identically on both sides and cancels, and the search recovers
// the true pose however wrong the map is.
TEST(CorridorMapTest, TheSurveyDiffersFromTheWorldItSurveyed) {
  const auto poses = StraightPath(120);
  cam_loc::map::TrajectoryCorridorMap map;
  cam_loc::map::CorridorMapOptions opts;
  opts.sample_step_m = 1.0;
  opts.survey_error.lateral_sigma_m = 0.20;
  opts.survey_error.longitudinal_sigma_m = 0.20;
  ASSERT_EQ(map.BuildFromPoses(poses, opts), cam_loc::Status::kOk);

  const auto* world =
      dynamic_cast<const cam_loc::map::PolylineMap*>(&map.world());
  ASSERT_NE(world, nullptr)
      << "a surveyed map must expose the world it surveyed";
  ASSERT_NE(world, &map) << "world() must not be the map itself";
  ASSERT_EQ(world->map().polylines.size(), map.map().polylines.size())
      << "the survey moves landmarks, it does not add or drop them";

  double sum_sq = 0.0;
  size_t n = 0;
  for (size_t i = 0; i < map.map().polylines.size(); ++i) {
    const auto& surveyed = map.map().polylines[i].points;
    const auto& truth = world->map().polylines[i].points;
    ASSERT_EQ(surveyed.size(), truth.size());
    for (size_t j = 0; j < surveyed.size(); ++j) {
      sum_sq += (surveyed[j] - truth[j]).squaredNorm();
      ++n;
    }
  }
  ASSERT_GT(n, 0u);
  const double rms = std::sqrt(sum_sq / static_cast<double>(n));
  // Two axes at sigma each, so the 3-D displacement is about sqrt(2)*sigma.
  EXPECT_GT(rms, 0.05) << "the survey is indistinguishable from the world";
  EXPECT_LT(rms, 1.0) << "the survey error is implausibly large";
}

// A map with no survey error has no second geometry to hand out: world() is
// the map, and perception cut from it is cut from the same object.
TEST(CorridorMapTest, WithoutSurveyErrorTheWorldIsTheMap) {
  const auto poses = StraightPath(40);
  cam_loc::map::TrajectoryCorridorMap map;
  cam_loc::map::CorridorMapOptions opts;
  opts.survey_error.lateral_sigma_m = 0.0;
  opts.survey_error.longitudinal_sigma_m = 0.0;
  ASSERT_EQ(map.BuildFromPoses(poses, opts), cam_loc::Status::kOk);
  EXPECT_EQ(&map.world(), &map);
  EXPECT_FALSE(map.uncertainty().Enabled());
}

// The spatial grid keys a cell by packing (ix, iy) into one int64. Indices go
// negative the moment the map crosses the world origin, and the index is only
// built above 512 points -- so a short corridor sitting in positive coordinates
// never touches it. This drives a long corridor through the origin, which is
// what makes both the packing and the query path run.
TEST(CorridorMapTest, QueriesWorkWhereTheMapCrossesTheOrigin) {
  std::vector<cam_loc::kitti::Pose> poses;
  for (int i = 0; i < 400; ++i) {
    cam_loc::kitti::Pose p;
    p.T_world_cam0 = cam_loc::Mat44::Identity();
    // Straight drive from -200 m to +200 m along +Z, so half the corridor has
    // negative coordinates and the grid sees negative cell indices.
    p.T_world_cam0(2, 3) = -200.0 + static_cast<double>(i);
    poses.push_back(p);
  }
  cam_loc::map::TrajectoryCorridorMap map;
  cam_loc::map::CorridorMapOptions opts;
  ASSERT_EQ(map.BuildFromPoses(poses, opts), cam_loc::Status::kOk);

  size_t total = 0;
  for (const auto& pl : map.map().polylines) total += pl.points.size();
  ASSERT_GT(total, 512u) << "too few points for the spatial index to be built";

  for (double z : {-180.0, -1.0, 0.0, 1.0, 180.0}) {
    cam_loc::Mat44 T = cam_loc::Mat44::Identity();
    T(2, 3) = z;
    cam_loc::kitti::MapChunk local;
    ASSERT_EQ(map.QueryLocalMap(T, 30.0, local), cam_loc::Status::kOk);
    EXPECT_FALSE(local.polylines.empty())
        << "no map returned at z=" << z << "; the grid lost the cell";
  }
}
