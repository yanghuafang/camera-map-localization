// Polyline map storage, JSON ingest, and optional uniform-grid spatial index for queries.

#include <cam_loc/map/polyline_map.hpp>

#include <cam_loc/map/map_georef.hpp>

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>

namespace cam_loc::map {

namespace {

int64_t packCell(int ix, int iy) {
  return (static_cast<int64_t>(ix) << 32) ^ static_cast<uint32_t>(iy);
}

}  // namespace

int64_t PolylineMap::cellKey(int ix, int iy) { return packCell(ix, iy); }

void PolylineMap::rebuildSpatialIndex(double cell_size_m) {
  cell_size_m_ = cell_size_m;
  grid_.clear();
  polyline_bounds_.clear();
  use_spatial_index_ = false;

  if (map_.polylines.empty()) return;

  // Small maps: brute-force query is cheaper than maintaining a grid.
  size_t total_points = 0;
  for (const auto& pl : map_.polylines) {
    total_points += pl.points.size();
  }
  if (total_points < 512) return;

  use_spatial_index_ = true;
  polyline_bounds_.reserve(map_.polylines.size());

  for (size_t i = 0; i < map_.polylines.size(); ++i) {
    const auto& pl = map_.polylines[i];
    if (pl.points.empty()) continue;

    PolylineBounds bb;
    bb.index = i;
    bb.min_x = bb.max_x = pl.points[0].x();
    bb.min_y = bb.max_y = pl.points[0].y();
    for (const auto& p : pl.points) {
      bb.min_x = std::min(bb.min_x, p.x());
      bb.max_x = std::max(bb.max_x, p.x());
      bb.min_y = std::min(bb.min_y, p.y());
      bb.max_y = std::max(bb.max_y, p.y());
    }
    polyline_bounds_.push_back(bb);

    const int ix0 = static_cast<int>(std::floor(bb.min_x / cell_size_m_));
    const int ix1 = static_cast<int>(std::floor(bb.max_x / cell_size_m_));
    const int iy0 = static_cast<int>(std::floor(bb.min_y / cell_size_m_));
    const int iy1 = static_cast<int>(std::floor(bb.max_y / cell_size_m_));
    for (int ix = ix0; ix <= ix1; ++ix) {
      for (int iy = iy0; iy <= iy1; ++iy) {
        grid_[cellKey(ix, iy)].push_back(i);
      }
    }
  }
}

void PolylineMap::queryPolylineIndices(const Vec3& query, double radius_m,
                                       std::unordered_set<size_t>& out) const {
  out.clear();
  if (!use_spatial_index_) {
    for (size_t i = 0; i < map_.polylines.size(); ++i) {
      out.insert(i);
    }
    return;
  }

  const int ix0 = static_cast<int>(std::floor((query.x() - radius_m) / cell_size_m_));
  const int ix1 = static_cast<int>(std::floor((query.x() + radius_m) / cell_size_m_));
  const int iy0 = static_cast<int>(std::floor((query.y() - radius_m) / cell_size_m_));
  const int iy1 = static_cast<int>(std::floor((query.y() + radius_m) / cell_size_m_));
  for (int ix = ix0; ix <= ix1; ++ix) {
    for (int iy = iy0; iy <= iy1; ++iy) {
      const auto it = grid_.find(cellKey(ix, iy));
      if (it == grid_.end()) continue;
      for (size_t idx : it->second) {
        out.insert(idx);
      }
    }
  }
}

Status PolylineMap::loadFromJsonFile(const std::string& path) {
  std::ifstream in(path);
  if (!in.is_open()) {
    return Status::kIoError;
  }

  nlohmann::json j;
  try {
    in >> j;
  } catch (const nlohmann::json::exception&) {
    return Status::kInvalidArgument;
  }

  map_.polylines.clear();
  if (!j.contains("polylines") || !j["polylines"].is_array()) {
    return Status::kInvalidArgument;
  }

  // Optional embedded georef lets polylines use coord_frame "wgs84".
  MapGeoref georef;
  bool have_georef = false;
  if (j.contains("georef") && j["georef"].is_object()) {
    have_georef = georef.parseFromJsonString(j["georef"].dump()) == Status::kOk;
  }

  for (const auto& item : j["polylines"]) {
    kitti::MapPolyline3D pl;
    if (item.contains("id")) pl.id = item["id"].get<uint64_t>();
    if (item.contains("type")) {
      pl.type = kitti::polylineTypeFromString(item["type"].get<std::string>());
    }

    if (item.contains("points") && item["points"].is_array()) {
      const bool wgs84_points =
          have_georef && item.value("coord_frame", std::string("world")) == "wgs84";
      for (const auto& pt : item["points"]) {
        if (!pt.is_array() || pt.size() < 2) continue;
        if (wgs84_points) {
          const double lat = pt[0].get<double>();
          const double lon = pt[1].get<double>();
          const double alt = pt.size() >= 3 ? pt[2].get<double>() : 0.0;
          pl.points.push_back(georef.wgs84ToWorld(lat, lon, alt));
        } else if (pt.size() >= 3) {
          pl.points.emplace_back(pt[0].get<double>(), pt[1].get<double>(), pt[2].get<double>());
        }
      }
    }
    if (pl.points.size() >= 2) {
      map_.polylines.push_back(std::move(pl));
    }
  }

  if (map_.polylines.empty()) {
    return Status::kInvalidArgument;
  }
  rebuildSpatialIndex();
  return Status::kOk;
}

Status PolylineMap::queryLocalMap(const Mat44& T_world_rig, double radius_m,
                                  kitti::MapChunk& out) const {
  out.polylines.clear();
  const Vec3 query = T_world_rig.block<3, 1>(0, 3);
  const double r2 = radius_m * radius_m;

  // Grid narrows candidate polylines; per-point radius filter trims segments.
  std::unordered_set<size_t> candidates;
  queryPolylineIndices(query, radius_m, candidates);

  for (size_t pi : candidates) {
    const auto& pl = map_.polylines[pi];
    kitti::MapPolyline3D local = pl;
    local.points.clear();
    for (const auto& p : pl.points) {
      if ((p - query).squaredNorm() <= r2) {
        local.points.push_back(p);
      }
    }
    if (local.points.size() >= 2) {
      out.polylines.push_back(std::move(local));
    }
  }

  return Status::kOk;
}

}  // namespace cam_loc::map
