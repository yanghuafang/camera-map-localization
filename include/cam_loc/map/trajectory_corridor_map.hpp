#pragma once

/// Default v1 map when no HD map file is provided: corridor lanes from GT odometry.

#include <cam_loc/kitti/types.hpp>
#include <cam_loc/map/polyline_map.hpp>

#include <vector>

namespace cam_loc::map {

/// Synthetic lane map by offsetting GT trajectory left/right in the horizontal plane.
class TrajectoryCorridorMap : public PolylineMap {
 public:
  /// Build solid left/right boundaries and a dashed centerline from GT camera poses.
  Status buildFromPoses(const std::vector<kitti::Pose>& poses, double half_width_m = 1.75,
                        double sample_step_m = 2.0);
};

}  // namespace cam_loc::map
