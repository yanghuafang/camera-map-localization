#pragma once

#include <cam_loc/map/map_loader.hpp>
#include <cam_loc/map/map_georef.hpp>

#include <memory>
#include <string>
#include <vector>

namespace cam_loc::kitti {
struct Pose;
}

namespace cam_loc::map {

/// How `createMapLoader` interprets `MapLoadOptions::map_path`.
enum class MapFileKind {
  kCorridor,  ///< Empty path → synthetic corridor from GT poses.
  kJson,      ///< World-frame or WGS84 JSON polyline dump.
  kOsm,       ///< Native OSM XML (requires valid georef).
};

/// Infer loader kind from path extension; empty path → corridor.
MapFileKind detectMapFileKind(const std::string& path);

/// Inputs shared by JSON, OSM, and corridor map construction.
struct MapLoadOptions {
  std::string map_path;
  std::string georef_path;
  MapGeoref georef;
  /// When true and poses are provided, set world yaw from frame-0 motion heading.
  bool align_yaw_to_first_pose = false;
  const std::vector<kitti::Pose>* poses = nullptr;
};

/// Create a map loader from path: corridor (empty path), JSON, or native OSM.
Status createMapLoader(const MapLoadOptions& options, std::shared_ptr<IMapLoader>& out);

}  // namespace cam_loc::map
