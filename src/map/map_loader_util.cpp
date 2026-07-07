// Map loader factory: detect file kind, apply georef/yaw alignment, return the right IMapLoader.

#include <cam_loc/map/map_loader_util.hpp>

#include <cam_loc/kitti/types.hpp>

#include <cam_loc/map/osm_map_loader.hpp>
#include <cam_loc/map/trajectory_corridor_map.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>

namespace cam_loc::map {

namespace {

std::string toLower(std::string s) {
  for (char& c : s) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return s;
}

std::string extension(const std::string& path) {
  const size_t dot = path.find_last_of('.');
  if (dot == std::string::npos || dot + 1 >= path.size()) return {};
  return toLower(path.substr(dot + 1));
}

double headingFromPoses(const std::vector<kitti::Pose>& poses) {
  if (poses.size() < 2) return 0.0;
  Vec3 fwd = poses[1].T_world_cam0.block<3, 1>(0, 3) - poses[0].T_world_cam0.block<3, 1>(0, 3);
  fwd.z() = 0.0;
  if (fwd.norm() < 1e-3) return 0.0;
  fwd.normalize();
  return std::atan2(fwd.y(), fwd.x());
}

}  // namespace

MapFileKind detectMapFileKind(const std::string& path) {
  if (path.empty()) return MapFileKind::kCorridor;
  const std::string ext = extension(path);
  if (ext == "osm" || ext == "xml") return MapFileKind::kOsm;
  return MapFileKind::kJson;
}

Status createMapLoader(const MapLoadOptions& options, std::shared_ptr<IMapLoader>& out) {
  const MapFileKind kind = detectMapFileKind(options.map_path);

  // No map file: build left/right lane boundaries from GT trajectory.
  if (kind == MapFileKind::kCorridor) {
    if (options.poses == nullptr || options.poses->size() < 2) {
      return Status::kInvalidArgument;
    }
    auto corridor = std::make_shared<TrajectoryCorridorMap>();
    const Status st = corridor->buildFromPoses(*options.poses);
    if (st != Status::kOk) return st;
    out = std::move(corridor);
    return Status::kOk;
  }

  // Shared georef for OSM and optional WGS84 JSON; may override yaw from motion heading.
  MapGeoref georef = options.georef;
  if (!options.georef_path.empty()) {
    const Status st = georef.loadFromJsonFile(options.georef_path);
    if (st != Status::kOk) return st;
  }

  if (options.align_yaw_to_first_pose && options.poses != nullptr && !options.poses->empty()) {
    georef.world_yaw_rad = headingFromPoses(*options.poses);
  }

  auto loader = std::make_shared<OsmMapLoader>();
  loader->setGeoref(georef);

  if (kind == MapFileKind::kOsm) {
    if (!georef.isValid()) return Status::kInvalidArgument;
    const Status st = loader->loadFromOsmFile(options.map_path);
    if (st != Status::kOk) return st;
    out = std::move(loader);
    return Status::kOk;
  }

  // Default: pre-exported world-frame JSON polylines.
  const Status st = loader->loadFromJsonFile(options.map_path);
  if (st != Status::kOk) return st;
  out = std::move(loader);
  return Status::kOk;
}

}  // namespace cam_loc::map
