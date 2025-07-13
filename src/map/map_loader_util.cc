// Map loader factory: detect file kind, apply georef/yaw alignment, return the
// right IMapLoader.

#include "cam_loc/map/map_loader_util.h"

#include <algorithm>
#include <cctype>
#include <cmath>

#include "cam_loc/core/frames.h"
#include "cam_loc/kitti/types.h"
#include "cam_loc/map/osm_map_loader.h"
#include "cam_loc/map/trajectory_corridor_map.h"

namespace cam_loc::map {

namespace {

std::string ToLower(std::string s) {
  for (char& c : s) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return s;
}

std::string Extension(const std::string& path) {
  const size_t dot = path.find_last_of('.');
  if (dot == std::string::npos || dot + 1 >= path.size()) return {};
  return ToLower(path.substr(dot + 1));
}

double HeadingFromPoses(const std::vector<kitti::Pose>& poses) {
  if (poses.size() < 2) return 0.0;
  const Vec3 step = poses[1].T_world_cam0.block<3, 1>(0, 3) -
                    poses[0].T_world_cam0.block<3, 1>(0, 3);
  // Drop the vertical component and take the heading in the ground plane. In
  // cam0 that plane is XZ, not XY -- zeroing z instead left a straight drive
  // with a zero-length vector and a heading of 0 for every sequence.
  const Vec3 ground = core::Frames::ToVehicle(step);
  if (ground.head<2>().norm() < 1e-3) return 0.0;
  return std::atan2(ground.y(), ground.x());
}

}  // namespace

MapFileKind DetectMapFileKind(const std::string& path) {
  if (path.empty()) return MapFileKind::kCorridor;
  const std::string ext = Extension(path);
  if (ext == "osm" || ext == "xml") return MapFileKind::kOsm;
  return MapFileKind::kJson;
}

Status CreateMapLoader(const MapLoadOptions& options,
                       std::shared_ptr<IMapLoader>& out) {
  const MapFileKind kind = DetectMapFileKind(options.map_path);

  // No map file: build left/right lane boundaries from GT trajectory.
  if (kind == MapFileKind::kCorridor) {
    if (options.poses == nullptr || options.poses->size() < 2) {
      return Status::kInvalidArgument;
    }
    auto corridor = std::make_shared<TrajectoryCorridorMap>();
    const Status st = corridor->BuildFromPoses(*options.poses);
    if (st != Status::kOk) return st;
    out = std::move(corridor);
    return Status::kOk;
  }

  // Shared georef for OSM and optional WGS84 JSON; may override yaw from motion
  // heading.
  MapGeoref georef = options.georef;
  if (!options.georef_path.empty()) {
    const Status st = georef.LoadFromJsonFile(options.georef_path);
    if (st != Status::kOk) return st;
  }

  if (options.align_yaw_to_first_pose && options.poses != nullptr &&
      !options.poses->empty()) {
    georef.world_yaw_rad = HeadingFromPoses(*options.poses);
  }

  auto loader = std::make_shared<OsmMapLoader>();
  loader->set_georef(georef);

  if (kind == MapFileKind::kOsm) {
    if (!georef.IsValid()) return Status::kInvalidArgument;
    const Status st = loader->LoadFromOsmFile(options.map_path);
    if (st != Status::kOk) return st;
    out = std::move(loader);
    return Status::kOk;
  }

  // Default: pre-exported world-frame JSON polylines.
  const Status st = loader->LoadFromJsonFile(options.map_path);
  if (st != Status::kOk) return st;
  out = std::move(loader);
  return Status::kOk;
}

}  // namespace cam_loc::map
