#pragma once

#include <cam_loc/types/status.hpp>

#include <string>

namespace cam_loc::map {

/// WGS84 anchor: KITTI world (0,0,0) ↔ geographic origin.
///
/// Horizontal mapping uses a local tangent plane (east/north meters from origin).
/// `world_yaw_rad` rotates ENU into KITTI XY: +Z stays up.
struct MapGeoref {
  double origin_lat_deg = 0.0;
  double origin_lon_deg = 0.0;
  double origin_alt_m = 0.0;
  /// CCW angle from East to KITTI +X in the horizontal plane.
  double world_yaw_rad = 0.0;

  bool isValid() const;

  static MapGeoref fromFirstPoseHeading(double origin_lat_deg, double origin_lon_deg,
                                        double heading_rad, double origin_alt_m = 0.0);

  Status loadFromJsonFile(const std::string& path);
  Status parseFromJsonString(const std::string& json_text);

  /// Convert WGS84 degrees to KITTI world XYZ (meters).
  Vec3 wgs84ToWorld(double lat_deg, double lon_deg, double alt_m = 0.0) const;
};

}  // namespace cam_loc::map
