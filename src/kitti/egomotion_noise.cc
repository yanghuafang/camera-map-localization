// Deterministic odometry error: seeded scale and heading drift on the relative
// motion fed to the prediction step.

#include "cam_loc/kitti/egomotion_noise.h"

#include "cam_loc/core/frames.h"
#include "cam_loc/types/random.h"

namespace cam_loc::kitti {

namespace {

/// Per-frame seed: reproducible run-to-run yet decorrelated between frames.
/// Same mix as the perception noise, so the two sources stay independent when
/// a run enables both.
uint32_t MixSeed(uint32_t seed, int frame) {
  return seed ^ static_cast<uint32_t>(frame * 2654435761u);
}

}  // namespace

Egomotion AddEgomotionNoise(const Egomotion& in,
                            const EgomotionNoiseParams& params, uint32_t seed) {
  if (!params.Enabled()) return in;

  Egomotion out = in;
  const Vec3 t_delta = in.T_curr_prev.block<3, 1>(0, 3);
  const double distance_m = t_delta.norm();
  if (distance_m < 1e-9) return out;

  Sampler rng(MixSeed(seed, in.global.frame));

  // Scale error: systematic plus a per-step random part.
  const double scale =
      1.0 + params.scale_error + params.scale_sigma * rng.Gaussian();

  // Heading drift, in degrees per metre, so a longer step drifts further.
  const double drift_deg =
      distance_m * (params.heading_bias_deg_per_m +
                    params.heading_sigma_deg_per_m * rng.Gaussian());
  const double drift_rad = drift_deg * M_PI / 180.0;

  // The step is in the previous body frame, so the heading error goes on the
  // right: the vehicle turned slightly more (or less) than it believed it did
  // over this step. Applying it on the left would instead re-interpret where
  // the step started, which is a different mistake.
  const Eigen::Matrix3d R_drift =
      Eigen::AngleAxisd(drift_rad, core::Frames::UpCam0()).toRotationMatrix();

  out.T_curr_prev.block<3, 1>(0, 3) = scale * t_delta;
  out.T_curr_prev.block<3, 3>(0, 0) =
      in.T_curr_prev.block<3, 3>(0, 0) * R_drift;
  return out;
}

}  // namespace cam_loc::kitti
