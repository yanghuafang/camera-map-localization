// Separated local minima of a cost volume.

#include "cam_loc/core/cost_modes.h"

#include <gtest/gtest.h>

namespace {

using cam_loc::SamplingGridParams;
using cam_loc::core::CostGrid;
using cam_loc::core::CostMode;
using cam_loc::core::FindCostModes;

CostGrid Flat(float value) {
  SamplingGridParams p;
  p.num_x = 11;
  p.num_y = 11;
  p.num_yaw = 3;
  CostGrid grid(p);
  grid.Fill(value);
  return grid;
}

TEST(CostModesTest, AFlatVolumeHasNoModes) {
  // Every pose fits alike, so there is no hypothesis to report.
  const CostGrid grid = Flat(5.f);
  EXPECT_TRUE(FindCostModes(grid).empty());
}

TEST(CostModesTest, AMonotonicVolumeHasNoModes) {
  // The cost falls away to an edge, so the pose that would win is outside the
  // extent searched. The border argmin is a saturated estimate, not a
  // measurement, and there is no interior hypothesis to report.
  CostGrid grid = Flat(0.f);
  for (int iw = 0; iw < grid.DimW(); ++iw) {
    for (int iy = 0; iy < grid.DimY(); ++iy) {
      for (int ix = 0; ix < grid.DimX(); ++ix) {
        grid.At(ix, iy, iw) = static_cast<float>(ix);
      }
    }
  }
  EXPECT_TRUE(FindCostModes(grid).empty());
}

TEST(CostModesTest, SingleBasinGivesOneMode) {
  CostGrid grid = Flat(5.f);
  grid.At(grid.nx(), grid.ny(), grid.nw()) = 1.f;
  const auto modes = FindCostModes(grid);
  ASSERT_EQ(modes.size(), 1u);
  EXPECT_FLOAT_EQ(modes[0].cost, 1.f);
}

TEST(CostModesTest, TwoSeparatedBasinsAreBothReported) {
  CostGrid grid = Flat(5.f);
  grid.At(grid.nx() - 3, grid.ny(), grid.nw()) = 1.f;
  grid.At(grid.nx() + 3, grid.ny(), grid.nw()) = 2.f;
  const auto modes = FindCostModes(grid);
  ASSERT_EQ(modes.size(), 2u);
  EXPECT_FLOAT_EQ(modes[0].cost, 1.f) << "best first";
  EXPECT_FLOAT_EQ(modes[1].cost, 2.f);
}

TEST(CostModesTest, NeighbouringCellsOfOneBasinAreNotTwoHypotheses) {
  CostGrid grid = Flat(5.f);
  grid.At(grid.nx(), grid.ny(), grid.nw()) = 1.f;
  grid.At(grid.nx() + 1, grid.ny(), grid.nw()) = 1.1f;
  const auto modes = FindCostModes(grid, 4, 2);
  EXPECT_EQ(modes.size(), 1u);
}

TEST(CostModesTest, MaxModesIsRespected) {
  CostGrid grid = Flat(5.f);
  grid.At(grid.nx() - 3, grid.ny(), grid.nw()) = 1.f;
  grid.At(grid.nx(), grid.ny(), grid.nw()) = 2.f;
  grid.At(grid.nx() + 3, grid.ny(), grid.nw()) = 3.f;
  EXPECT_EQ(FindCostModes(grid, 2).size(), 2u);
  EXPECT_TRUE(FindCostModes(grid, 0).empty());
}

TEST(CostModesTest, BorderMinimaAreSkipped) {
  // Their neighbourhood is incomplete, so whether they are minima is not
  // decidable; the interior basin is still found.
  CostGrid grid = Flat(5.f);
  grid.At(0, 0, 0) = 0.f;
  grid.At(grid.nx(), grid.ny(), grid.nw()) = 1.f;
  const auto modes = FindCostModes(grid);
  ASSERT_EQ(modes.size(), 1u);
  EXPECT_FLOAT_EQ(modes[0].cost, 1.f);
}

TEST(CostModesTest, ATroughIsOneModeNotADozen) {
  // The lane-only failure: a line of equal-cost cells along the along-track
  // axis. Suppression has to collapse it, or every cell reports as a rival.
  CostGrid grid = Flat(5.f);
  for (int ix = 1; ix < grid.DimX() - 1; ++ix) {
    grid.At(ix, grid.ny(), grid.nw()) = 1.f;
  }
  const auto modes = FindCostModes(grid, 4, 2);
  EXPECT_LE(modes.size(), 4u);
  for (const CostMode& m : modes) EXPECT_FLOAT_EQ(m.cost, 1.f);
}

}  // namespace

TEST(CostModesTest, AmbiguityVanishesWithoutARunnerUp) {
  std::vector<CostMode> one(1);
  EXPECT_TRUE(cam_loc::core::ModeAmbiguityCovariance(one, 1.0).isZero());
  EXPECT_TRUE(cam_loc::core::ModeAmbiguityCovariance({}, 1.0).isZero());
}

TEST(CostModesTest, EquallyGoodModesGiveHalfTheSeparation) {
  std::vector<CostMode> modes(2);
  modes[0].cost = 1.f;
  modes[1].cost = 1.f;
  modes[1].offset = cam_loc::Vec3(4.0, 0.0, 0.0);

  const auto cov = cam_loc::core::ModeAmbiguityCovariance(modes, 1.0);
  EXPECT_NEAR(cov(0, 0), 4.0, 1e-9) << "(d/2)^2 for d = 4 m";
  EXPECT_NEAR(cov(1, 1), 0.0, 1e-12) << "nothing is claimed off the separation";
}

TEST(CostModesTest, ADistantRunnerUpAddsAlmostNothing) {
  std::vector<CostMode> modes(2);
  modes[0].cost = 1.f;
  modes[1].cost = 11.f;
  modes[1].offset = cam_loc::Vec3(4.0, 0.0, 0.0);
  EXPECT_LT(cam_loc::core::ModeAmbiguityCovariance(modes, 1.0)(0, 0), 1e-3);
}

TEST(CostModesTest, AmbiguityFollowsTheSeparationDirection) {
  std::vector<CostMode> modes(2);
  modes[0].cost = 1.f;
  modes[1].cost = 1.f;
  modes[1].offset = cam_loc::Vec3(3.0, 3.0, 0.0);

  const auto cov = cam_loc::core::ModeAmbiguityCovariance(modes, 1.0);
  EXPECT_NEAR(cov(0, 1), cov(0, 0), 1e-9)
      << "a diagonal disagreement is a cross term, not two separate ones";
}
