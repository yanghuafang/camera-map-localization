#pragma once

#include <cam_loc/kitti/types.hpp>
#include <cam_loc/map/map_georef.hpp>
#include <cam_loc/types/status.hpp>

#include <string>
#include <vector>

namespace cam_loc::map {

/// Geographic bounding box from an OSM `<bounds>` element (optional output).
struct OsmBounds {
  double min_lat = 0;
  double min_lon = 0;
  double max_lat = 0;
  double max_lon = 0;
  bool valid = false;
};

/// Parse OSM XML (0.6) into world-frame map polylines using `georef`.
Status parseOsmXml(const std::string& xml_text, const MapGeoref& georef,
                   kitti::MapChunk& out, OsmBounds* out_bounds = nullptr);

Status parseOsmXmlFile(const std::string& path, const MapGeoref& georef, kitti::MapChunk& out,
                       OsmBounds* out_bounds = nullptr);

}  // namespace cam_loc::map
