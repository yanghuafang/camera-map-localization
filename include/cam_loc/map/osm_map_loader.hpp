#pragma once

#include <cam_loc/map/map_georef.hpp>
#include <cam_loc/map/polyline_map.hpp>

namespace cam_loc::map {

/// Loads HD maps from JSON (world-frame) or native OSM XML (WGS84 + georef).
class OsmMapLoader : public PolylineMap {
 public:
  void setGeoref(const MapGeoref& georef) { georef_ = georef; }
  const MapGeoref& georef() const { return georef_; }

  /// Auto-detect `.json` vs `.osm` / `.xml`.
  Status loadFromFile(const std::string& path);

  /// Load native OSM XML via `parseOsmXmlFile` (requires valid `georef_`).
  Status loadFromOsmFile(const std::string& path);

 private:
  MapGeoref georef_;
};

}  // namespace cam_loc::map
