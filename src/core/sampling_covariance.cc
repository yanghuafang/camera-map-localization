// Measurement covariance from the shape of the cost basin at the argmin.

#include "cam_loc/core/sampling_covariance.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "cam_loc/core/cost_quadratic.h"

namespace cam_loc::core {

namespace {

/// Softmax-weighted second moment of the offset cloud about the argmin.
///
/// For a quadratic basin this returns cost_scale * H⁻¹, so it tracks the basin
/// shape but scales with an arbitrary temperature. Kept as the fallback for
/// surfaces the Hessian cannot be taken on.
Eigen::Matrix3d SoftmaxSpread(const CostGrid& grid,
                              const CostGrid::ArgMinResult& argmin,
                              float cost_scale) {
  const Vec3 mu = grid.IndexToOffset(argmin.ix, argmin.iy, argmin.iw);
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  double weight_sum = 0.0;

  for (int iw = 0; iw < grid.DimW(); ++iw) {
    for (int iy = 0; iy < grid.DimY(); ++iy) {
      for (int ix = 0; ix < grid.DimX(); ++ix) {
        const double w =
            std::exp(-static_cast<double>(grid.At(ix, iy, iw) - argmin.cost) /
                     cost_scale);
        if (w < 1e-6) continue;
        const Vec3 d = grid.IndexToOffset(ix, iy, iw) - mu;
        cov += w * d * d.transpose();
        weight_sum += w;
      }
    }
  }
  return weight_sum > 1e-9 ? Eigen::Matrix3d(cov / weight_sum)
                           : Eigen::Matrix3d::Zero();
}

/// Clamp a covariance's eigenvalues after whitening by @p unit.
///
/// The eigenvectors mix metres with radians, so a bound stated as one number
/// means nothing until the axes are made comparable. Whitening by a per-axis
/// scale does that, and which scale depends on which bound is being applied.
Eigen::Matrix3d ClampEigenvalues(const Eigen::Matrix3d& cov, const Vec3& unit,
                                 double lo, double hi) {
  const Eigen::Matrix3d to_units(unit.cwiseInverse().asDiagonal());
  const Eigen::Matrix3d from_units(unit.asDiagonal());

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(to_units * cov * to_units);
  if (es.info() != Eigen::Success) return cov;

  Vec3 values = es.eigenvalues();
  for (int i = 0; i < 3; ++i) values[i] = std::clamp(values[i], lo, hi);
  const Eigen::Matrix3d clamped =
      es.eigenvectors() * values.asDiagonal() * es.eigenvectors().transpose();
  return from_units * clamped * from_units;
}

/// Bound the covariance to what the search could have resolved.
///
/// Two bounds, each in its own units. The floor is one cell, so whitening is by
/// the grid step and no eigenvalue may fall below 1/12, the variance of a value
/// known only to lie within one cell. The ceiling is the search itself, so
/// whitening is by each axis's half-extent and no eigenvalue may exceed 1:
/// nothing outside the grid was scored, so no direction may claim a sigma wider
/// than the grid spans along it. Whitening the ceiling per axis is what keeps
/// the yaw axis, spanning a few degrees, from inheriting the forward axis's
/// metres.
Eigen::Matrix3d ClampToSearchedExtent(const CostGrid& grid,
                                      const Eigen::Matrix3d& cov) {
  const Vec3 step(grid.step_x(), grid.step_y(), grid.step_yaw());
  const Vec3 half_extent(grid.nx() * grid.step_x(), grid.ny() * grid.step_y(),
                         grid.nw() * grid.step_yaw());
  const Eigen::Matrix3d floored = ClampEigenvalues(
      cov, step, 1.0 / 12.0, std::numeric_limits<double>::infinity());
  return ClampEigenvalues(floored, half_extent, 0.0, 1.0);
}

/// Variance of a value known only to lie within one cell.
Eigen::Matrix3d DiscretizationFloor(const CostGrid& grid) {
  const Vec3 step(grid.step_x(), grid.step_y(), grid.step_yaw());
  return Eigen::Matrix3d((step.cwiseProduct(step) / 12.0).asDiagonal());
}

}  // namespace

SamplingConfidence SamplingCovariance::Compute(
    const CostGrid& grid, const CostGrid::ArgMinResult& argmin,
    float cost_scale) {
  SamplingConfidence conf;
  if (cost_scale <= 0.f) return conf;

  const Eigen::Matrix3d floor = DiscretizationFloor(grid);

  const LocalQuadratic fit = FitLocalQuadratic(grid, argmin);
  if (fit.valid) {
    const Eigen::LDLT<Eigen::Matrix3d> ldlt(fit.hessian);
    const auto d = ldlt.vectorD();
    // A non-definite Hessian is a saddle or a ridge, not a basin: there is a
    // direction the argmin can slide along for free, and inverting it would
    // report a negative variance.
    if (ldlt.info() == Eigen::Success && d.minCoeff() > 1e-12 * d.maxCoeff()) {
      // Sigma of the cost itself, floored so a perfect fit does not claim
      // infinite precision.
      const double sigma_cost =
          std::max(static_cast<double>(argmin.cost), 1e-3);
      const Eigen::Matrix3d cov =
          2.0 * sigma_cost * ldlt.solve(Eigen::Matrix3d::Identity());
      conf.covariance = ClampToSearchedExtent(grid, cov + floor);
      conf.valid = true;
      return conf;
    }
  }

  const Eigen::Matrix3d spread = SoftmaxSpread(grid, argmin, cost_scale);
  if (spread.trace() <= 0.0) return conf;
  conf.covariance = ClampToSearchedExtent(grid, spread + floor);
  conf.valid = true;
  return conf;
}

}  // namespace cam_loc::core
