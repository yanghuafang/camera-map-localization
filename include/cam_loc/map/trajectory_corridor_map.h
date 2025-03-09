#ifndef CAM_LOC_MAP_TRAJECTORY_CORRIDOR_MAP_H_
#define CAM_LOC_MAP_TRAJECTORY_CORRIDOR_MAP_H_

/// Stand-in HD map built from ground-truth odometry, for when no real one
/// exists.

#include <cstdint>
#include <memory>
#include <vector>

#include "cam_loc/kitti/types.h"
#include "cam_loc/map/polyline_map.h"

namespace cam_loc::map {

/// How far the stored map is from the world it surveyed.
///
/// Without this the corridor is the world, and so is the perception projected
/// from it -- the same geometry on both sides of the match. Every pose error
/// then comes from discretization and filter lag, and no amount of perception
/// noise changes that, because noise perturbs the *observation* of a geometry
/// that is still exactly right.
///
/// The error is smooth along the route rather than independent per point. A
/// survey is wrong in a way that varies over tens of metres: two landmarks a
/// metre apart are wrong by nearly the same amount, and the local map inside
/// one query is therefore displaced almost bodily. That is what makes it
/// unrecoverable -- a bodily displacement is indistinguishable from being
/// somewhere else, which is exactly why map error puts a floor under
/// localization accuracy that no better search can lift.
struct MapSurveyErrorParams {
  /// Sigma of the lateral displacement, metres.
  double lateral_sigma_m = 0.20;
  /// Sigma of the along-track displacement, metres.
  double longitudinal_sigma_m = 0.20;
  /// Distance over which the displacement decorrelates. Shorter than the map
  /// query radius and the error partly averages out within one frame; much
  /// longer and it is a constant offset the filter absorbs once.
  double correlation_length_m = 80.0;
  /// Seed. The same seed and path always give the same survey.
  uint32_t seed = 7u;

  bool Enabled() const {
    return lateral_sigma_m > 0.0 || longitudinal_sigma_m > 0.0;
  }
};

/// Shape of the synthetic corridor.
///
/// Distances are metres, in the vehicle sense: lateral offsets are left of the
/// trajectory for positive values, heights are above the road.
struct CorridorMapOptions {
  /// Half the lane width; the two solid boundaries sit this far either side.
  double half_width_m = 1.75;
  /// Spacing of sampled points along each polyline.
  double sample_step_m = 2.0;
  /// Camera height above the road. The trajectory is the camera path, so the
  /// road surface is this far below it.
  double ground_height_m = 1.65;

  /// Poles every this far along the route, alternating sides. Zero disables.
  double pole_spacing_m = 15.0;
  /// Lateral offset of a pole from the trajectory.
  double pole_offset_m = 4.5;
  /// Height of a pole above the road.
  double pole_height_m = 4.0;

  /// Signs every this far along the route, on the right. Zero disables.
  double sign_spacing_m = 45.0;
  double sign_offset_m = 4.0;
  /// Height of the sign panel's centre above the road.
  double sign_height_m = 2.2;
  /// Width of the sign panel.
  double sign_width_m = 0.8;

  /// How wrong the stored map is. **On by default**: a map that is exactly
  /// right is the one assumption this dataset lets you make and reality never
  /// does, and every accuracy number taken under it measures the search rather
  /// than localization.
  MapSurveyErrorParams survey_error;
};

/// Lane, boundary, pole and sign geometry offset from a ground-truth path.
///
/// KITTI Odometry ships no HD map, so map matching needs one from somewhere.
/// This builds the simplest thing that is geometrically consistent with the
/// trajectory: two solid lane boundaries and a dashed centreline on the road
/// surface, plus poles and signs beside it.
///
/// The poles and signs are the point of it. Lane lines run parallel to travel,
/// so they pin the vehicle laterally and in heading but say almost nothing
/// about where along the road it is; a localizer given only lane geometry has
/// an unobservable degree of freedom. Upright landmarks break that. See
/// docs/ARCHITECTURE.md for the full table of what each class constrains.
///
/// Being derived from ground truth, this map still cannot *validate* absolute
/// accuracy: the corridor follows the trajectory, so a systematic error in the
/// trajectory is invisible to it. What `MapSurveyErrorParams` does buy is the
/// error that matters for map matching — the map and the world disagreeing —
/// which is what makes the pose error something the search has to overcome
/// rather than something it recovers by construction.
class TrajectoryCorridorMap : public PolylineMap {
 public:
  /// Build the world, then survey it into the stored map.
  ///
  /// Both are built from the same path samples, so they carry the same
  /// landmarks in the same order; only their positions differ, by the survey
  /// error. `world()` returns the first, which is what perception is cut from;
  /// `QueryLocalMap` returns the second, which is what the matcher scores.
  ///
  /// @param poses Camera poses, world ← cam0. At least two are needed to
  ///        establish a direction.
  /// @return `kInvalidArgument` for fewer than two poses.
  Status BuildFromPoses(const std::vector<kitti::Pose>& poses,
                        const CorridorMapOptions& options = {});

  /// The un-surveyed geometry; `*this` when no survey error was applied.
  const IMapLoader& world() const override {
    return world_ ? static_cast<const IMapLoader&>(*world_) : *this;
  }

  /// The survey error this map was built with, which is what it is worth.
  MapUncertainty uncertainty() const override { return uncertainty_; }

 private:
  std::unique_ptr<PolylineMap> world_;
  MapUncertainty uncertainty_;
};

}  // namespace cam_loc::map

#endif  // CAM_LOC_MAP_TRAJECTORY_CORRIDOR_MAP_H_
