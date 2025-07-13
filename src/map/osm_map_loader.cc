// OsmMapLoader: dispatch JSON vs OSM file load, then index polylines for local
// queries.

#include "cam_loc/map/osm_map_loader.h"

#include "cam_loc/map/map_loader_util.h"
#include "cam_loc/map/osm_xml_parser.h"

namespace cam_loc::map {

Status OsmMapLoader::LoadFromFile(const std::string& path) {
  const MapFileKind kind = DetectMapFileKind(path);
  if (kind == MapFileKind::kOsm) {
    return LoadFromOsmFile(path);
  }
  return LoadFromJsonFile(path);
}

Status OsmMapLoader::LoadFromOsmFile(const std::string& path) {
  if (!georef_.IsValid()) {
    return Status::kInvalidArgument;
  }

  kitti::MapChunk parsed;
  // The file's <bounds> element is not used here; the local-map query works off
  // the pose, not the extract's extent.
  const Status st = ParseOsmXmlFile(path, georef_, parsed, nullptr);
  if (st != Status::kOk) {
    return st;
  }

  map_ = std::move(parsed);
  RebuildSpatialIndex();
  return Status::kOk;
}

}  // namespace cam_loc::map
