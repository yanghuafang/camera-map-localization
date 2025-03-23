// Local quadratic fit, and the sub-cell refinement built on it.

#include "cam_loc/core/cost_quadratic.h"

#include <cmath>

#include <gtest/gtest.h>

#include "cam_loc/core/cost_grid.h"

namespace {

using cam_loc::SamplingGridParams;
using cam_loc::Vec3;
using cam_loc::core::CostGrid;
using cam_loc::core::FitLocalQuadratic;

SamplingGridParams Grid() {
  SamplingGridParams p;
  p.num_x = 11;
  p.num_y = 11;
  p.num_yaw = 7;
  p.step_x_m = 0.5;
  p.step_y_m = 0.5;
  return p;
}

/// A quadratic bowl with its minimum at @p centre, optionally rotated in the
/// (x, y) plane so the basin does not line up with the grid axes.
CostGrid Bowl(const Vec3& centre, double kx, double ky, double kw,
              double tilt_rad = 0.0) {
  CostGrid grid(Grid());
  const double c = std::cos(tilt_rad);
  const double s = std::sin(tilt_rad);
  for (int iw = 0; iw < grid.DimW(); ++iw) {
    for (int iy = 0; iy < grid.DimY(); ++iy) {
      for (int ix = 0; ix < grid.DimX(); ++ix) {
        const Vec3 d = grid.IndexToOffset(ix, iy, iw) - centre;
        const double u = c * d.x() + s * d.y();
        const double v = -s * d.x() + c * d.y();
        grid.At(ix, iy, iw) = static_cast<float>(1.0 + kx * u * u + ky * v * v +
                                                 kw * d.z() * d.z());
      }
    }
  }
  return grid;
}

TEST(CostQuadraticTest, GradientVanishesAtTheMinimum) {
  const CostGrid grid = Bowl(Vec3::Zero(), 1.0, 1.0, 1.0);
  const auto fit = FitLocalQuadratic(grid, grid.Argmin());
  ASSERT_TRUE(fit.valid);
  EXPECT_NEAR(fit.gradient.norm(), 0.0, 1e-9);
  EXPECT_NEAR(fit.hessian(0, 0), 2.0, 1e-9) << "d2/dx2 of kx*x^2 is 2kx";
}

TEST(CostQuadraticTest, ATiltedBowlHasCrossTerms) {
  const CostGrid grid = Bowl(Vec3::Zero(), 4.0, 1.0, 1.0, M_PI / 4.0);
  const auto fit = FitLocalQuadratic(grid, grid.Argmin());
  ASSERT_TRUE(fit.valid);
  EXPECT_GT(std::abs(fit.hessian(0, 1)), 1.0)
      << "a basin at an angle to the axes is a cross term and nothing else";
}

TEST(CostQuadraticTest, BorderCellsHaveNoFit) {
  const CostGrid grid = Bowl(Vec3::Zero(), 1.0, 1.0, 1.0);
  CostGrid::ArgMinResult border;
  border.ix = 0;
  EXPECT_FALSE(FitLocalQuadratic(grid, border).valid);
}

// The reason for fitting a quadratic at all: recover where the minimum really
// is, when it falls between cells.
TEST(CostQuadraticTest, RefinementBeatsTheCellCentreOnATiltedBowl) {
  // Offset by a third of a cell on both axes, and tilted so that the error is
  // one three independent parabolas cannot correct.
  const Vec3 truth(0.5 / 3.0, -0.5 / 3.0, 0.0);
  const CostGrid grid = Bowl(truth, 4.0, 1.0, 1.0, M_PI / 4.0);

  const auto argmin = grid.Argmin();
  const Vec3 cell = grid.IndexToOffset(argmin.ix, argmin.iy, argmin.iw);
  const Vec3 refined = grid.RefinedOffset(argmin);

  EXPECT_LT((refined - truth).head<2>().norm(), (cell - truth).head<2>().norm())
      << "refined " << refined.transpose() << " is no better than cell centre "
      << cell.transpose();
}

// A direction the surface barely curves in must not produce a large step: an
// undamped -g/lambda there is enormous, and the cross terms carry it into the
// axes that are determined.
TEST(CostQuadraticTest, AFlatDirectionProducesABoundedStep) {
  const CostGrid grid = Bowl(Vec3(0.1, 0.0, 0.0), 4.0, 1e-6, 1.0);
  const auto argmin = grid.Argmin();
  const Vec3 refined = grid.RefinedOffset(argmin);
  const Vec3 cell = grid.IndexToOffset(argmin.ix, argmin.iy, argmin.iw);

  EXPECT_LE(std::abs(refined.y() - cell.y()), 0.5 * grid.step_y() + 1e-9);
  EXPECT_LE(std::abs(refined.x() - cell.x()), 0.5 * grid.step_x() + 1e-9);
}

TEST(CostQuadraticTest, RefinementNeverLeavesTheWinningCell) {
  for (double tilt : {0.0, 0.3, 1.0}) {
    const CostGrid grid = Bowl(Vec3(0.2, -0.15, 0.0), 3.0, 0.5, 1.0, tilt);
    const auto argmin = grid.Argmin();
    const Vec3 cell = grid.IndexToOffset(argmin.ix, argmin.iy, argmin.iw);
    const Vec3 refined = grid.RefinedOffset(argmin);
    EXPECT_LE(std::abs(refined.x() - cell.x()), 0.5 * grid.step_x() + 1e-9);
    EXPECT_LE(std::abs(refined.y() - cell.y()), 0.5 * grid.step_y() + 1e-9);
    EXPECT_LE(std::abs(refined.z() - cell.z()), 0.5 * grid.step_yaw() + 1e-9);
  }
}

}  // namespace
