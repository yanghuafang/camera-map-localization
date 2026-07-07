#pragma once

/// Map data access for localization: load polylines and query a local neighborhood per frame.

#include <cam_loc/kitti/types.hpp>
#include <cam_loc/types/status.hpp>

#include <string>
#include <vector>

namespace cam_loc::map {

/// Abstract map source: load polylines once, query a local neighborhood per frame.
class IMapLoader {
 public:
  virtual ~IMapLoader() = default;

  /// Populate the full map from a JSON file (world-frame or WGS84 + embedded georef).
  virtual Status loadFromJsonFile(const std::string& path) = 0;

  /// Return polylines whose points lie within @p radius_m of the rig translation in @p T_world_rig.
  virtual Status queryLocalMap(const Mat44& T_world_rig, double radius_m,
                               kitti::MapChunk& out) const = 0;
};

}  // namespace cam_loc::map
