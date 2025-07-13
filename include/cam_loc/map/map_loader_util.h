#ifndef CAM_LOC_MAP_MAP_LOADER_UTIL_H_
#define CAM_LOC_MAP_MAP_LOADER_UTIL_H_

#include <memory>
#include <string>
#include <vector>

#include "cam_loc/map/map_georef.h"
#include "cam_loc/map/map_loader.h"

namespace cam_loc::kitti {
struct Pose;
}

namespace cam_loc::map {

/// How `CreateMapLoader` interprets `MapLoadOptions::map_path`.
enum class MapFileKind {
  kCorridor,  ///< Empty path → synthetic corridor from GT poses.
  kJson,      ///< World-frame or WGS84 JSON polyline dump.
  kOsm,       ///< Native OSM XML (requires valid georef).
};

/// Infer loader kind from path extension; empty path → corridor.
MapFileKind DetectMapFileKind(const std::string& path);

/// Inputs shared by JSON, OSM, and corridor map construction.
struct MapLoadOptions {
  std::string map_path;
  std::string georef_path;
  MapGeoref georef;
  /// When true and poses are provided, set world yaw from frame-0 motion
  /// heading.
  bool align_yaw_to_first_pose = false;
  const std::vector<kitti::Pose>* poses = nullptr;
};

/// Create a map loader from path: corridor (empty path), JSON, or native OSM.
Status CreateMapLoader(const MapLoadOptions& options,
                       std::shared_ptr<IMapLoader>& out);

}  // namespace cam_loc::map

#endif  // CAM_LOC_MAP_MAP_LOADER_UTIL_H_
