#ifndef CAM_LOC_CORE_COST_MODES_H_
#define CAM_LOC_CORE_COST_MODES_H_

/// Separated local minima of a pose cost volume.

#include <vector>

#include "cam_loc/core/cost_grid.h"
#include "cam_loc/types/status.h"

namespace cam_loc::core {

/// One local minimum of the cost volume.
struct CostMode {
  /// Packed offset `(x_m, y_m, yaw_rad)` in the sampling plane.
  Vec3 offset = Vec3::Zero();
  float cost = 0.f;
  int ix = 0;
  int iy = 0;
  int iw = 0;
};

/// Lowest separated local minima, best first.
///
/// The argmin alone says where the best hypothesis is, not whether anything
/// else fits nearly as well. With lane geometry only, the along-track direction
/// is a long shallow trough that can carry several minima a few centimetres
/// apart in cost and metres apart in pose; reporting the winner as a confident
/// answer is how a localizer ends up a car length down the road.
///
/// A cell is a minimum when no neighbour is lower and at least one is higher,
/// so a flat-bottomed trough is kept but a uniformly flat volume yields
/// nothing -- there is no hypothesis to report when every pose fits alike.
///
/// Cells on the volume border are skipped: their neighbourhood is incomplete,
/// so whether they are minima cannot be decided.
///
/// @param max_modes           Upper bound on the returned count.
/// @param separation_cells    Chebyshev radius, in cells, within which a lower
///                            minimum suppresses a higher one. Neighbouring
///                            cells of one basin are not competing hypotheses.
std::vector<CostMode> FindCostModes(const CostGrid& grid, int max_modes = 4,
                                    int separation_cells = 2);

/// Covariance contributed by not being able to rule out the runner-up.
///
/// Two minima the frame cannot separate make the measurement a two-component
/// mixture rather than one Gaussian. A mixture's covariance carries
/// `p₁·p₂·ddᵀ` over the separation `d` between the components: it inflates
/// along the direction the two disagree on and nowhere else, vanishes when the
/// runner-up is far behind on cost, and reaches `(d/2)²` when the two are
/// equally good — the answer is somewhere between them, and nothing narrower
/// is claimed.
///
/// @param modes      Separated minima, best first. Fewer than two gives zero.
/// @param sigma_cost Cost noise scale, in DT pixels; the mixture weights come
///                   from `exp(-Δcost / sigma_cost)`.
Eigen::Matrix3d ModeAmbiguityCovariance(const std::vector<CostMode>& modes,
                                        double sigma_cost);

}  // namespace cam_loc::core

#endif  // CAM_LOC_CORE_COST_MODES_H_
