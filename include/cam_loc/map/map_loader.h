#ifndef CAM_LOC_MAP_MAP_LOADER_H_
#define CAM_LOC_MAP_MAP_LOADER_H_

/// Map data access for localization: load polylines and query a local
/// neighborhood per frame.

#include <string>
#include <vector>

#include "cam_loc/kitti/types.h"
#include "cam_loc/types/status.h"

namespace cam_loc::map {

/// Abstract map source: load polylines once, query a local neighborhood per
/// frame.
class IMapLoader {
 public:
  virtual ~IMapLoader() = default;

  /// Populate the full map from a JSON file.
  ///
  /// Points are world-frame `[x, y, z]` by default, or WGS84
  /// `[lat, lon, alt]` when the polyline sets `"coord_frame": "wgs84"` and the
  /// file carries a top-level `"georef"` block.
  ///
  /// @return `kIoError` if the file cannot be opened, `kInvalidArgument` if it
  ///         is malformed or yields no polyline with at least two points.
  virtual Status LoadFromJsonFile(const std::string& path) = 0;

  /// Return the map near a pose, trimmed point by point.
  ///
  /// @param T_world_rig Query pose; only its translation is used.
  /// @param radius_m    3-D radius about that translation.
  /// @param out         Cleared, then filled with the polylines that keep at
  ///                    least two points inside the radius. A polyline that
  ///                    enters and leaves the radius is returned as a single
  ///                    polyline with the gap closed, not split in two.
  virtual Status QueryLocalMap(const Mat44& T_world_rig, double radius_m,
                               kitti::MapChunk& out) const = 0;
};

}  // namespace cam_loc::map

#endif  // CAM_LOC_MAP_MAP_LOADER_H_
