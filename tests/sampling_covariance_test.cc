// Measurement covariance from the cost basin: curvature, residual, fallbacks.

#include "cam_loc/core/sampling_covariance.h"

#include <cmath>

#include <gtest/gtest.h>

namespace {

using cam_loc::SamplingGridParams;
using cam_loc::Vec3;
using cam_loc::core::CostGrid;
using cam_loc::core::SamplingConfidence;
using cam_loc::core::SamplingCovariance;

/// Deliberately not cubic. A ceiling taken from the widest axis rather than
/// per axis looks correct on a cubic grid and is wrong here.
SamplingGridParams SmallGrid() {
  SamplingGridParams p;
  p.num_x = 7;
  p.num_y = 11;
  p.num_yaw = 5;
  return p;
}

/// cost = c_min + ½(kx·x² + ky·y² + kw·yaw²), centred on the grid.
CostGrid Quadratic(double kx, double ky, double kw, float c_min = 1.f) {
  CostGrid grid(SmallGrid());
  for (int iw = 0; iw < grid.DimW(); ++iw) {
    for (int iy = 0; iy < grid.DimY(); ++iy) {
      for (int ix = 0; ix < grid.DimX(); ++ix) {
        const Vec3 d = grid.IndexToOffset(ix, iy, iw);
        grid.At(ix, iy, iw) =
            c_min +
            static_cast<float>(0.5 * (kx * d.x() * d.x() + ky * d.y() * d.y() +
                                      kw * d.z() * d.z()));
      }
    }
  }
  return grid;
}

TEST(SamplingCovarianceTest, SharperBasinGivesSmallerVariance) {
  const CostGrid shallow = Quadratic(1.0, 1.0, 1.0);
  const CostGrid sharp = Quadratic(100.0, 1.0, 1.0);
  const auto a = SamplingCovariance::Compute(shallow, shallow.Argmin(), 0.5f);
  const auto b = SamplingCovariance::Compute(sharp, sharp.Argmin(), 0.5f);
  ASSERT_TRUE(a.valid && b.valid);
  EXPECT_LT(b.covariance(0, 0), a.covariance(0, 0));
  EXPECT_NEAR(b.covariance(1, 1), a.covariance(1, 1), 1e-9);
}

TEST(SamplingCovarianceTest, VarianceIsLinearInTheResidual) {
  // 2σ_c H⁻¹: doubling the misfit at the argmin doubles the reported variance,
  // less the discretization floor, which does not scale.
  const CostGrid low = Quadratic(4.0, 4.0, 4.0, 1.f);
  const CostGrid high = Quadratic(4.0, 4.0, 4.0, 2.f);
  const auto a = SamplingCovariance::Compute(low, low.Argmin(), 0.5f);
  const auto b = SamplingCovariance::Compute(high, high.Argmin(), 0.5f);
  ASSERT_TRUE(a.valid && b.valid);

  const double floor = low.step_x() * low.step_x() / 12.0;
  EXPECT_NEAR(b.covariance(0, 0) - floor, 2.0 * (a.covariance(0, 0) - floor),
              1e-9);
}

TEST(SamplingCovarianceTest, NeverClaimsBetterThanTheCellItSearched) {
  const CostGrid sharp = Quadratic(1e6, 1e6, 1e6);
  const auto conf = SamplingCovariance::Compute(sharp, sharp.Argmin(), 0.5f);
  ASSERT_TRUE(conf.valid);
  EXPECT_GE(conf.covariance(0, 0), sharp.step_x() * sharp.step_x() / 12.0);
  EXPECT_GE(conf.covariance(2, 2), sharp.step_yaw() * sharp.step_yaw() / 12.0);
}

TEST(SamplingCovarianceTest, RidgeFallsBackInsteadOfReportingNegativeVariance) {
  // Flat along y: no basin, so H is singular and cannot be inverted.
  const CostGrid ridge = Quadratic(4.0, 0.0, 4.0);
  const auto conf = SamplingCovariance::Compute(ridge, ridge.Argmin(), 0.5f);
  ASSERT_TRUE(conf.valid);
  EXPECT_GT(conf.covariance(1, 1), conf.covariance(0, 0))
      << "the unconstrained axis must be the uncertain one";
}

TEST(SamplingCovarianceTest, BorderArgminUsesTheFallback) {
  CostGrid grid(SmallGrid());
  grid.Fill(5.f);
  grid.At(0, 0, 0) = 0.f;
  const auto conf = SamplingCovariance::Compute(grid, grid.Argmin(), 0.5f);
  EXPECT_TRUE(conf.valid) << "no curvature is available, but the spread is";
}

TEST(SamplingCovarianceTest, NeverClaimsMoreThanTheExtentSearched) {
  // Nothing outside the grid was scored, so no direction may report a sigma
  // wider than the grid itself.
  const CostGrid ridge = Quadratic(4.0, 0.0, 4.0);
  const auto conf = SamplingCovariance::Compute(ridge, ridge.Argmin(), 0.5f);
  ASSERT_TRUE(conf.valid);

  const double reach_y = ridge.ny() * ridge.step_y();
  const double reach_yaw = ridge.nw() * ridge.step_yaw();
  EXPECT_LE(conf.covariance(1, 1), reach_y * reach_y * 1.000001);
  EXPECT_LE(conf.covariance(2, 2), reach_yaw * reach_yaw * 1.000001)
      << "yaw must be bounded by yaw's own extent, not the widest axis's";
}

TEST(SamplingCovarianceTest, KeepsTheDirectionOfADiagonalTrough) {
  // The failure the eigenbasis exists for: a trough at 45 degrees to the grid
  // axes. Its diagonal alone would report equal uncertainty on both, losing
  // which combination is actually unconstrained.
  CostGrid grid(SmallGrid());
  for (int iw = 0; iw < grid.DimW(); ++iw) {
    for (int iy = 0; iy < grid.DimY(); ++iy) {
      for (int ix = 0; ix < grid.DimX(); ++ix) {
        const Vec3 d = grid.IndexToOffset(ix, iy, iw);
        const double across = (d.x() - d.y()) / std::sqrt(2.0);
        grid.At(ix, iy, iw) =
            1.f + static_cast<float>(2.0 * across * across + d.z() * d.z());
      }
    }
  }
  const auto conf = SamplingCovariance::Compute(grid, grid.Argmin(), 0.5f);
  ASSERT_TRUE(conf.valid);
  EXPECT_GT(conf.covariance(0, 1), 0.0)
      << "the free direction is forward+left, which only a cross term records";
}

TEST(SamplingCovarianceTest, RejectsANonPositiveTemperature) {
  const CostGrid grid = Quadratic(4.0, 4.0, 4.0);
  EXPECT_FALSE(SamplingCovariance::Compute(grid, grid.Argmin(), 0.f).valid);
}

}  // namespace
