// Unit tests for cost grid argmin and pose sampling.
#include <cam_loc/core/cost_grid.hpp>
#include <cam_loc/core/cost_aggregator.hpp>
#include <cam_loc/core/pose_sampler.hpp>
#include <cam_loc/core/projection.hpp>
#include <cam_loc/kitti/types.hpp>
#include <cam_loc/perception/synthesize.hpp>

#include <gtest/gtest.h>

namespace {

cam_loc::kitti::MapChunk makeStraightMap() {
  cam_loc::kitti::MapChunk map;
  cam_loc::kitti::MapPolyline3D center;
  center.type = cam_loc::kitti::PolylineType::kLaneSolid;
  for (int i = 1; i < 40; ++i) {
    center.points.emplace_back(0.0, 0.0, static_cast<double>(i) * 0.5);
  }
  map.polylines.push_back(center);
  return map;
}

cam_loc::kitti::Calibration defaultCalib() {
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
  grid.fill(10.f);
  const int cx = grid.nx();
  const int cy = grid.ny();
  const int cw = grid.nw();
  grid.at(cx, cy, cw) = 0.f;
  const auto best = grid.argmin();
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

  cam_loc::core::Projection proj(defaultCalib());
  cam_loc::core::PoseSampler sampler(params);
  sampler.setProjection(proj);

  const cam_loc::Mat44 T_world = cam_loc::Mat44::Identity();
  const auto map = makeStraightMap();
  const auto perception =
      cam_loc::perception::synthesizeFromMap(map, proj, T_world, 0);

  cam_loc::core::LabelledDistanceTransform dt;
  ASSERT_EQ(sampler.buildImageDt(perception, dt), cam_loc::Status::kOk);

  cam_loc::core::CostGrid costs(params.grid);
  ASSERT_EQ(sampler.computeImageCosts(map, T_world, dt, costs), cam_loc::Status::kOk);

  const auto best = costs.argmin();
  const float center_cost = costs.at(costs.nx(), costs.ny(), costs.nw());
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
  frame0.fill(5.f);
  frame0.at(frame0.nx(), frame0.ny(), frame0.nw()) = 1.f;
  agg.pushHistory(frame0, cam_loc::Mat44::Identity(), 0);

  cam_loc::core::CostGrid frame1 = frame0;
  frame1.at(frame1.nx(), frame1.ny(), frame1.nw()) = 3.f;
  agg.aggregate(frame1, cam_loc::Mat44::Identity(), 0.1);

  EXPECT_LT(frame1.at(frame1.nx(), frame1.ny(), frame1.nw()), 3.f);
}
