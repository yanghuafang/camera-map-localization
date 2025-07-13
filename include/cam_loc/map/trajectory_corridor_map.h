#ifndef CAM_LOC_MAP_TRAJECTORY_CORRIDOR_MAP_H_
#define CAM_LOC_MAP_TRAJECTORY_CORRIDOR_MAP_H_

/// Stand-in HD map built from ground-truth odometry, for when no real one
/// exists.

#include <vector>

#include "cam_loc/kitti/types.h"
#include "cam_loc/map/polyline_map.h"

namespace cam_loc::map {

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
/// Being derived from ground truth, this map cannot be used to *validate*
/// accuracy — matching against it recovers the trajectory it was built from.
/// It exists so the pipeline can be exercised end to end without an HD map.
class TrajectoryCorridorMap : public PolylineMap {
 public:
  /// Build the corridor from a ground-truth camera path.
  ///
  /// @param poses Camera poses, world ← cam0. At least two are needed to
  ///        establish a direction.
  /// @return `kInvalidArgument` for fewer than two poses.
  Status BuildFromPoses(const std::vector<kitti::Pose>& poses,
                        const CorridorMapOptions& options = {});
};

}  // namespace cam_loc::map

#endif  // CAM_LOC_MAP_TRAJECTORY_CORRIDOR_MAP_H_
