#ifndef CAM_LOC_KITTI_EGOMOTION_NOISE_H_
#define CAM_LOC_KITTI_EGOMOTION_NOISE_H_

/// Odometry error injected into the relative motion the filter predicts with.

#include <cmath>

#include "cam_loc/kitti/types.h"

namespace cam_loc::kitti {

/// Corruption applied to one relative-motion step, in the terms odometry error
/// is actually specified in: a fraction of the distance travelled, and a
/// heading drift per metre.
///
/// Both are *per metre* rather than per frame because that is how the error
/// behaves. A vehicle standing still accumulates no odometry error however long
/// it stands, and a per-frame sigma would say otherwise.
///
/// Each pair is a systematic term and a random one, and the distinction is the
/// point. The random terms average out over a window and mostly widen the
/// covariance; the systematic ones integrate into the steady drift that dead
/// reckoning is actually limited by, and that a map match has to keep pulling
/// back. A model with only white noise flatters the filter.
struct EgomotionNoiseParams {
  /// Systematic scale error on travelled distance, as a fraction: 0.01 is an
  /// odometer reading 1% long.
  double scale_error = 0.0;
  /// Standard deviation of the random part of that scale error, per step.
  double scale_sigma = 0.0;
  /// Systematic heading drift, degrees per metre travelled.
  double heading_bias_deg_per_m = 0.0;
  /// Standard deviation of the random part of the heading drift, degrees per
  /// metre travelled.
  double heading_sigma_deg_per_m = 0.0;

  bool Enabled() const {
    return std::abs(scale_error) > 1e-12 || scale_sigma > 0.0 ||
           std::abs(heading_bias_deg_per_m) > 1e-12 ||
           heading_sigma_deg_per_m > 0.0;
  }
};

/// Corrupt the relative motion of one frame, leaving everything else alone.
///
/// Only `T_curr_prev` is touched. `global` stays the ground-truth pose it was:
/// it is what the filter initializes from on frame 0, and a localizer that
/// cannot be told where it starts is a different problem from one whose
/// odometry drifts. The global prior is not fed back as a measurement unless
/// `use_global_ego_measurement` asks for it, so leaving it truthful does not
/// quietly repair the drift this function injects.
///
/// Deterministic: the same seed and frame index give the same perturbation.
Egomotion AddEgomotionNoise(const Egomotion& in,
                            const EgomotionNoiseParams& params, uint32_t seed);

}  // namespace cam_loc::kitti

#endif  // CAM_LOC_KITTI_EGOMOTION_NOISE_H_
