/// Per-frame perception JSON loader (lane_lines + road_boundaries).
#include <cam_loc/kitti/calib_parser.hpp>

#include <nlohmann/json.hpp>

#include <fstream>

namespace cam_loc::kitti {

namespace {

Polyline2D parsePolyline2D(const nlohmann::json& j) {
  Polyline2D pl;
  if (j.contains("type")) {
    pl.type = polylineTypeFromString(j["type"].get<std::string>());
  }
  if (j.contains("points") && j["points"].is_array()) {
    for (const auto& pt : j["points"]) {
      if (pt.is_array() && pt.size() >= 2) {
        pl.points.emplace_back(pt[0].get<double>(), pt[1].get<double>());
      }
    }
  }
  return pl;
}

}  // namespace

Status loadPerceptionJson(const std::string& path, FramePerception& out) {
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

  out.lane_lines.clear();
  out.road_boundaries.clear();

  if (j.contains("frame")) {
    out.frame = j["frame"].get<int>();
  }

  if (j.contains("lane_lines") && j["lane_lines"].is_array()) {
    for (const auto& item : j["lane_lines"]) {
      out.lane_lines.push_back(parsePolyline2D(item));
    }
  }

  if (j.contains("road_boundaries") && j["road_boundaries"].is_array()) {
    for (const auto& item : j["road_boundaries"]) {
      out.road_boundaries.push_back(parsePolyline2D(item));
    }
  }

  return Status::kOk;
}

}  // namespace cam_loc::kitti
