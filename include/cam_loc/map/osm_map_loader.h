#ifndef CAM_LOC_MAP_OSM_MAP_LOADER_H_
#define CAM_LOC_MAP_OSM_MAP_LOADER_H_

#include "cam_loc/map/map_georef.h"
#include "cam_loc/map/polyline_map.h"

namespace cam_loc::map {

/// Loads HD maps from JSON (world-frame) or native OSM XML (WGS84 + georef).
class OsmMapLoader : public PolylineMap {
 public:
  void set_georef(const MapGeoref& georef) { georef_ = georef; }
  const MapGeoref& georef() const { return georef_; }

  /// Auto-detect `.json` vs `.osm` / `.xml`.
  Status LoadFromFile(const std::string& path);

  /// Load native OSM XML via `ParseOsmXmlFile` (requires valid `georef_`).
  Status LoadFromOsmFile(const std::string& path);

 private:
  MapGeoref georef_;
};

}  // namespace cam_loc::map

#endif  // CAM_LOC_MAP_OSM_MAP_LOADER_H_
