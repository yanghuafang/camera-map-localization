// OsmMapLoader: dispatch JSON vs OSM file load, then index polylines for local queries.

#include <cam_loc/map/osm_map_loader.hpp>

#include <cam_loc/map/map_loader_util.hpp>
#include <cam_loc/map/osm_xml_parser.hpp>

namespace cam_loc::map {

Status OsmMapLoader::loadFromFile(const std::string& path) {
  const MapFileKind kind = detectMapFileKind(path);
  if (kind == MapFileKind::kOsm) {
    return loadFromOsmFile(path);
  }
  return loadFromJsonFile(path);
}

Status OsmMapLoader::loadFromOsmFile(const std::string& path) {
  if (!georef_.isValid()) {
    return Status::kInvalidArgument;
  }

  kitti::MapChunk parsed;
  OsmBounds bounds;
  const Status st = parseOsmXmlFile(path, georef_, parsed, &bounds);
  if (st != Status::kOk) {
    return st;
  }

  map_ = std::move(parsed);
  rebuildSpatialIndex();
  return Status::kOk;
}

}  // namespace cam_loc::map
