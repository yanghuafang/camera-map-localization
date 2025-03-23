// Local minima of the pose cost volume, separated by non-maximum suppression.

#include "cam_loc/core/cost_modes.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace cam_loc::core {

namespace {

/// True when no neighbour is lower and at least one is strictly higher.
///
/// Both halves are needed. Accepting ties alone would mark every cell of a
/// uniformly flat volume as a minimum; requiring a strict minimum everywhere
/// would reject a flat-bottomed trough entirely, which is the shape lane-only
/// geometry actually produces. Requiring some rise out of the neighbourhood
/// keeps the trough and drops the plateau interior, and suppression collapses
/// what remains of the trough into one entry per basin.
bool IsLocalMin(const CostGrid& grid, int ix, int iy, int iw) {
  const float c = grid.At(ix, iy, iw);
  bool rises = false;
  for (int dw = -1; dw <= 1; ++dw) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0 && dw == 0) continue;
        const float n = grid.At(ix + dx, iy + dy, iw + dw);
        if (n < c) return false;
        if (n > c) rises = true;
      }
    }
  }
  return rises;
}

bool WithinSeparation(const CostMode& a, const CostMode& b, int cells) {
  return std::abs(a.ix - b.ix) <= cells && std::abs(a.iy - b.iy) <= cells &&
         std::abs(a.iw - b.iw) <= cells;
}

}  // namespace

std::vector<CostMode> FindCostModes(const CostGrid& grid, int max_modes,
                                    int separation_cells) {
  std::vector<CostMode> modes;
  if (max_modes <= 0) return modes;

  std::vector<CostMode> candidates;
  for (int iw = 1; iw < grid.DimW() - 1; ++iw) {
    for (int iy = 1; iy < grid.DimY() - 1; ++iy) {
      for (int ix = 1; ix < grid.DimX() - 1; ++ix) {
        if (!IsLocalMin(grid, ix, iy, iw)) continue;
        CostMode m;
        m.offset = grid.IndexToOffset(ix, iy, iw);
        m.cost = grid.At(ix, iy, iw);
        m.ix = ix;
        m.iy = iy;
        m.iw = iw;
        candidates.push_back(m);
      }
    }
  }

  std::sort(
      candidates.begin(), candidates.end(),
      [](const CostMode& a, const CostMode& b) { return a.cost < b.cost; });

  for (const auto& c : candidates) {
    if (static_cast<int>(modes.size()) >= max_modes) break;
    const bool suppressed =
        std::any_of(modes.begin(), modes.end(), [&](const CostMode& kept) {
          return WithinSeparation(kept, c, separation_cells);
        });
    if (!suppressed) modes.push_back(c);
  }
  return modes;
}

Eigen::Matrix3d ModeAmbiguityCovariance(const std::vector<CostMode>& modes,
                                        double sigma_cost) {
  if (modes.size() < 2 || sigma_cost <= 0.0) return Eigen::Matrix3d::Zero();

  const double odds = std::exp(-(modes[1].cost - modes[0].cost) / sigma_cost);
  const double p_second = odds / (1.0 + odds);
  const Vec3 separation = modes[1].offset - modes[0].offset;
  return (1.0 - p_second) * p_second * separation * separation.transpose();
}

}  // namespace cam_loc::core
