// Unit tests for cost grid argmin and pose sampling.
#include "cam_loc/core/pose_sampler.h"

#include <gtest/gtest.h>

#include "cam_loc/core/cost_aggregator.h"
#include "cam_loc/core/cost_grid.h"
#include "cam_loc/core/frames.h"
#include "cam_loc/core/projection.h"
#include "cam_loc/kitti/types.h"
#include "cam_loc/map/trajectory_corridor_map.h"
#include "cam_loc/perception/synthesize.h"

namespace {

cam_loc::kitti::MapChunk MakeStraightMap() {
  cam_loc::kitti::MapChunk map;
  cam_loc::kitti::MapPolyline3D center;
  center.type = cam_loc::kitti::PolylineType::kLaneSolid;
  for (int i = 1; i < 40; ++i) {
    center.points.emplace_back(0.0, 0.0, static_cast<double>(i) * 0.5);
  }
  map.polylines.push_back(center);
  return map;
}

cam_loc::kitti::Calibration DefaultCalib() {
  cam_loc::kitti::Calibration c;
  c.P0(0, 0) = 718.0;
  c.P0(1, 1) = 718.0;
  c.P0(0, 2) = 607.0;
  c.P0(1, 2) = 185.0;
  c.P0(2, 2) = 1.0;
  return c;
}

}  // namespace

TEST(CostGridTest, ArgminCenter) {
  cam_loc::SamplingGridParams gp;
  gp.num_x = 5;
  gp.num_y = 5;
  gp.num_yaw = 3;
  cam_loc::core::CostGrid grid(gp);
  grid.Fill(10.f);
  const int cx = grid.nx();
  const int cy = grid.ny();
  const int cw = grid.nw();
  grid.At(cx, cy, cw) = 0.f;
  const auto best = grid.Argmin();
  EXPECT_EQ(best.ix, cx);
  EXPECT_EQ(best.iy, cy);
  EXPECT_EQ(best.iw, cw);
}

TEST(PoseSamplerTest, CenterHypothesisAchievesMinimumCost) {
  cam_loc::LocalizationParams params;
  params.grid.num_x = 11;
  params.grid.num_y = 11;
  params.grid.num_yaw = 5;
  params.grid.step_x_m = 0.25;
  params.grid.step_y_m = 0.25;
  params.grid.step_yaw_deg = 0.5;
  params.enable_bev = false;

  cam_loc::core::Projection proj(DefaultCalib());
  cam_loc::core::PoseSampler sampler(params);
  sampler.set_projection(proj);

  const cam_loc::Mat44 T_world = cam_loc::Mat44::Identity();
  const auto map = MakeStraightMap();
  const auto perception =
      cam_loc::perception::SynthesizeFromMap(map, proj, T_world, 0);

  cam_loc::core::LabelledDistanceTransform dt;
  ASSERT_EQ(sampler.BuildImageDt(perception, dt), cam_loc::Status::kOk);

  cam_loc::core::CostGrid costs(params.grid);
  ASSERT_EQ(sampler.ComputeImageCosts(map, T_world, dt, costs),
            cam_loc::Status::kOk);

  const auto best = costs.Argmin();
  const float center_cost = costs.At(costs.nx(), costs.ny(), costs.nw());
  EXPECT_NEAR(best.cost, center_cost, 1e-2f);
  EXPECT_LT(center_cost, 5.f);
}

TEST(CostAggregatorTest, HistoryLowersCostAtConsistentOffset) {
  cam_loc::AggregationParams ap;
  ap.window_size = 5;
  cam_loc::core::CostAggregator agg(ap);

  cam_loc::SamplingGridParams gp;
  gp.num_x = 5;
  gp.num_y = 5;
  gp.num_yaw = 3;

  cam_loc::core::CostGrid frame0(gp);
  frame0.Fill(5.f);
  frame0.At(frame0.nx(), frame0.ny(), frame0.nw()) = 1.f;
  agg.PushHistory(frame0, cam_loc::Mat44::Identity(), 0, 0.0);

  cam_loc::core::CostGrid frame1 = frame0;
  frame1.At(frame1.nx(), frame1.ny(), frame1.nw()) = 3.f;
  agg.Aggregate(frame1, cam_loc::Mat44::Identity(), 0.5);

  EXPECT_LT(frame1.At(frame1.nx(), frame1.ny(), frame1.nw()), 3.f);
}

// The decay used to be measured from the start of the sequence rather than
// from the current frame, so history stopped contributing after
// 1 / distance_decay metres however recent it was -- and the empty average was
// still blended in, scaling the whole surface. Both are visible here as a
// centre cell that stops improving after 100 m of driving.
TEST(CostAggregatorTest, HistoryStillHelpsAfterLongDrive) {
  cam_loc::AggregationParams ap;
  ap.window_size = 5;
  cam_loc::core::CostAggregator agg(ap);

  cam_loc::SamplingGridParams gp;
  gp.num_x = 5;
  gp.num_y = 5;
  gp.num_yaw = 3;

  // Drive 300 m, well past 1 / distance_decay = 100 m, pushing one grid per
  // frame from the same plane pose so the warp is the identity.
  const cam_loc::Mat44 plane = cam_loc::Mat44::Identity();
  double travel = 0.0;
  for (int f = 0; f < 600; ++f) {
    travel += 0.5;
    cam_loc::core::CostGrid hist(gp);
    hist.Fill(5.f);
    hist.At(hist.nx(), hist.ny(), hist.nw()) = 1.f;
    agg.PushHistory(hist, plane, f, travel);
  }

  cam_loc::core::CostGrid current(gp);
  current.Fill(5.f);
  current.At(current.nx(), current.ny(), current.nw()) = 3.f;
  agg.Aggregate(current, plane, travel);

  const float centre = current.At(current.nx(), current.ny(), current.nw());
  EXPECT_LT(centre, 3.f) << "recent history stopped contributing";
  // The off-centre cells agreed at 5 throughout, so they must be untouched --
  // a blended-in empty average would have halved them.
  EXPECT_NEAR(current.At(0, 0, 0), 5.f, 1e-5);
}

namespace {

/// A corridor map along cam0 +Z, plus the oracle perception seen from its
/// start. Poses are one metre apart so the map is finely sampled.
struct CorridorFixture {
  cam_loc::map::TrajectoryCorridorMap map;
  cam_loc::kitti::MapChunk local;
  cam_loc::kitti::FramePerception perception;

  explicit CorridorFixture(const cam_loc::core::Projection& proj) {
    std::vector<cam_loc::kitti::Pose> poses;
    for (int i = 0; i < 120; ++i) {
      cam_loc::kitti::Pose p;
      p.frame = i;
      p.T_world_cam0 = cam_loc::Mat44::Identity();
      p.T_world_cam0(2, 3) = static_cast<double>(i);
      poses.push_back(p);
    }
    cam_loc::map::CorridorMapOptions opts;
    opts.sample_step_m = 1.0;
    map.BuildFromPoses(poses, opts);
    map.QueryLocalMap(cam_loc::Mat44::Identity(), 50.0, local);
    perception = cam_loc::perception::SynthesizeFromMap(
        local, proj, cam_loc::Mat44::Identity(), 0);
  }
};

cam_loc::LocalizationParams GridParams() {
  cam_loc::LocalizationParams params;
  params.grid.num_x = 41;
  params.grid.num_y = 21;
  params.grid.num_yaw = 5;
  params.grid.step_x_m = 0.5;
  params.grid.step_y_m = 0.5;
  params.grid.step_yaw_deg = 0.5;
  params.enable_bev = false;
  return params;
}

}  // namespace

// Lane lines run parallel to travel, so on their own they leave the along-track
// position unobservable: sliding the hypothesis down the road costs nothing.
// The upright landmarks in the map are what pin it, and the cost has to be
// averaged per class for their handful of points to outweigh the lane's many.
TEST(PoseSamplerTest, RecoversAlongTrackOffset) {
  const auto params = GridParams();
  cam_loc::core::Projection proj(DefaultCalib());
  CorridorFixture fixture(proj);

  cam_loc::core::PoseSampler sampler(params);
  sampler.set_projection(proj);
  cam_loc::core::LabelledDistanceTransform dt;
  ASSERT_EQ(sampler.BuildImageDt(fixture.perception, dt), cam_loc::Status::kOk);

  // Anchor the grid 3 m behind the pose the perception was taken from. The
  // search should put the hypothesis back where it belongs: 3 m forward.
  const cam_loc::Mat44 plane =
      cam_loc::Mat44::Identity() *
      cam_loc::core::Frames::OffsetToCam0Transform(-3.0, 0.0, 0.0);

  cam_loc::core::CostGrid costs(params.grid);
  ASSERT_EQ(sampler.ComputeImageCosts(fixture.local, plane, dt, costs),
            cam_loc::Status::kOk);

  const auto best = costs.Argmin();
  const cam_loc::Vec3 offset = costs.RefinedOffset(best);
  EXPECT_NEAR(offset.x(), 3.0, 0.6) << "along-track offset not recovered";
  EXPECT_NEAR(offset.y(), 0.0, 0.3);
}

// The grid pitch bounds how precisely the argmin alone can place a pose. The
// parabolic fit is there to do better than half a cell, so it has to beat the
// cell centre on an offset that deliberately falls between cells.
TEST(PoseSamplerTest, SubCellRefinementBeatsTheCellCentre) {
  const auto params = GridParams();
  cam_loc::core::Projection proj(DefaultCalib());
  CorridorFixture fixture(proj);

  cam_loc::core::PoseSampler sampler(params);
  sampler.set_projection(proj);
  cam_loc::core::LabelledDistanceTransform dt;
  ASSERT_EQ(sampler.BuildImageDt(fixture.perception, dt), cam_loc::Status::kOk);

  // 1.25 m is two and a half cells: the worst case for a half-metre grid.
  constexpr double kTrueOffsetM = 1.25;
  const cam_loc::Mat44 plane =
      cam_loc::Mat44::Identity() *
      cam_loc::core::Frames::OffsetToCam0Transform(-kTrueOffsetM, 0.0, 0.0);

  cam_loc::core::CostGrid costs(params.grid);
  ASSERT_EQ(sampler.ComputeImageCosts(fixture.local, plane, dt, costs),
            cam_loc::Status::kOk);

  const auto best = costs.Argmin();
  const double cell = costs.IndexToOffset(best.ix, best.iy, best.iw).x();
  const double refined = costs.RefinedOffset(best).x();
  EXPECT_LE(std::abs(refined - kTrueOffsetM), std::abs(cell - kTrueOffsetM))
      << "refined " << refined << " is no better than cell centre " << cell;
}
