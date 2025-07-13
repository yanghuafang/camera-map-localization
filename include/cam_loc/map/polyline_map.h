#ifndef CAM_LOC_MAP_POLYLINE_MAP_H_
#define CAM_LOC_MAP_POLYLINE_MAP_H_

/// In-memory polyline map with optional spatial grid index for fast local
/// queries.

#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "cam_loc/map/map_loader.h"

namespace cam_loc::map {

/// Map stored as world-frame 3-D polylines (JSON, OSM, or programmatic fill).
class PolylineMap : public IMapLoader {
 public:
  Status LoadFromJsonFile(const std::string& path) override;

  Status QueryLocalMap(const Mat44& T_world_rig, double radius_m,
                       kitti::MapChunk& out) const override;

  const kitti::MapChunk& map() const { return map_; }
  kitti::MapChunk& map() { return map_; }

 protected:
  /// Build a uniform grid index when the map is large enough for local queries.
  void RebuildSpatialIndex(double cell_size_m = 50.0);

  kitti::MapChunk map_;

 private:
  /// Axis-aligned XY footprint of one stored polyline (for grid insertion).
  struct PolylineBounds {
    size_t index = 0;
    double min_x = 0;
    double min_y = 0;
    double max_x = 0;
    double max_y = 0;
  };

  static int64_t CellKey(int ix, int iy);
  void QueryPolylineIndices(const Vec3& query, double radius_m,
                            std::unordered_set<size_t>& out) const;

  bool use_spatial_index_ = false;
  double cell_size_m_ = 50.0;
  std::vector<PolylineBounds> polyline_bounds_;
  std::unordered_map<int64_t, std::vector<size_t>> grid_;
};

}  // namespace cam_loc::map

#endif  // CAM_LOC_MAP_POLYLINE_MAP_H_
