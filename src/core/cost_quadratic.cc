// Local quadratic model of a cost volume, by central differences.

#include "cam_loc/core/cost_quadratic.h"

namespace cam_loc::core {

LocalQuadratic FitLocalQuadratic(const CostGrid& grid,
                                 const CostGrid::ArgMinResult& cell) {
  LocalQuadratic out;
  if (cell.ix <= 0 || cell.ix >= grid.DimX() - 1 || cell.iy <= 0 ||
      cell.iy >= grid.DimY() - 1 || cell.iw <= 0 ||
      cell.iw >= grid.DimW() - 1) {
    return out;
  }

  const double h[3] = {grid.step_x(), grid.step_y(), grid.step_yaw()};
  const auto at = [&](int dx, int dy, int dw) {
    return static_cast<double>(
        grid.At(cell.ix + dx, cell.iy + dy, cell.iw + dw));
  };
  const double c0 = at(0, 0, 0);

  out.gradient.x() = (at(1, 0, 0) - at(-1, 0, 0)) / (2.0 * h[0]);
  out.gradient.y() = (at(0, 1, 0) - at(0, -1, 0)) / (2.0 * h[1]);
  out.gradient.z() = (at(0, 0, 1) - at(0, 0, -1)) / (2.0 * h[2]);

  out.hessian(0, 0) = (at(1, 0, 0) - 2.0 * c0 + at(-1, 0, 0)) / (h[0] * h[0]);
  out.hessian(1, 1) = (at(0, 1, 0) - 2.0 * c0 + at(0, -1, 0)) / (h[1] * h[1]);
  out.hessian(2, 2) = (at(0, 0, 1) - 2.0 * c0 + at(0, 0, -1)) / (h[2] * h[2]);
  out.hessian(0, 1) = out.hessian(1, 0) =
      (at(1, 1, 0) - at(1, -1, 0) - at(-1, 1, 0) + at(-1, -1, 0)) /
      (4.0 * h[0] * h[1]);
  out.hessian(0, 2) = out.hessian(2, 0) =
      (at(1, 0, 1) - at(1, 0, -1) - at(-1, 0, 1) + at(-1, 0, -1)) /
      (4.0 * h[0] * h[2]);
  out.hessian(1, 2) = out.hessian(2, 1) =
      (at(0, 1, 1) - at(0, 1, -1) - at(0, -1, 1) + at(0, -1, -1)) /
      (4.0 * h[1] * h[2]);

  out.valid = true;
  return out;
}

}  // namespace cam_loc::core
