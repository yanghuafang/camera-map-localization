#ifndef CAM_LOC_MAP_MAP_GEOREF_H_
#define CAM_LOC_MAP_MAP_GEOREF_H_

#include <string>

#include "cam_loc/types/status.h"

namespace cam_loc::map {

/// WGS84 anchor: KITTI world (0,0,0) ↔ geographic origin.
///
/// Horizontal mapping uses a local tangent plane (east/north metres from the
/// origin); `world_yaw_rad` rotates it so the map's forward axis lines up with
/// the sequence heading. The result is in cam0 — X right, Y down, Z forward —
/// the same frame as the poses.
struct MapGeoref {
  double origin_lat_deg = 0.0;
  double origin_lon_deg = 0.0;
  double origin_alt_m = 0.0;
  /// CCW angle from East to KITTI +X in the horizontal plane.
  double world_yaw_rad = 0.0;

  /// @return Whether the origin is a well-formed lat/lon pair. Note this only
  ///         range-checks; it cannot tell an unset (0, 0) origin from a real
  ///         one in the Gulf of Guinea.
  bool IsValid() const;

  static MapGeoref FromFirstPoseHeading(double origin_lat_deg,
                                        double origin_lon_deg,
                                        double heading_rad,
                                        double origin_alt_m = 0.0);

  /// @return `kIoError` if the file cannot be opened, `kInvalidArgument` if it
  ///         is malformed or the resulting origin fails IsValid.
  Status LoadFromJsonFile(const std::string& path);

  /// Accepts `world_yaw_deg` or `world_yaw_rad`; degrees win if both appear.
  Status ParseFromJsonString(const std::string& json_text);

  /// Convert WGS84 degrees to world metres via a flat-earth local tangent
  /// plane, accurate over the extent of one sequence.
  ///
  /// @param alt_m Altitude; the returned Z is `alt_m − origin_alt_m`.
  /// @return Cam0 world coordinates in metres: X right, Y down, Z forward.
  Vec3 Wgs84ToWorld(double lat_deg, double lon_deg, double alt_m = 0.0) const;
};

}  // namespace cam_loc::map

#endif  // CAM_LOC_MAP_MAP_GEOREF_H_
