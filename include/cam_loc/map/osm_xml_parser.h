#ifndef CAM_LOC_MAP_OSM_XML_PARSER_H_
#define CAM_LOC_MAP_OSM_XML_PARSER_H_

#include <string>
#include <vector>

#include "cam_loc/kitti/types.h"
#include "cam_loc/map/map_georef.h"
#include "cam_loc/types/status.h"

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
Status ParseOsmXml(const std::string& xml_text, const MapGeoref& georef,
                   kitti::MapChunk& out, OsmBounds* out_bounds = nullptr);

Status ParseOsmXmlFile(const std::string& path, const MapGeoref& georef,
                       kitti::MapChunk& out, OsmBounds* out_bounds = nullptr);

}  // namespace cam_loc::map

#endif  // CAM_LOC_MAP_OSM_XML_PARSER_H_
