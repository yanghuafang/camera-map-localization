/// Per-frame perception JSON loader (lane_lines + road_boundaries).
#include <fstream>

#include <nlohmann/json.hpp>

#include "cam_loc/kitti/calib_parser.h"

namespace cam_loc::kitti {

namespace {

Polyline2D ParsePolyline2D(const nlohmann::json& j) {
  Polyline2D pl;
  if (j.contains("type")) {
    pl.type = PolylineTypeFromString(j["type"].get<std::string>());
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

Status LoadPerceptionJson(const std::string& path, FramePerception& out) {
  std::ifstream in(path);
  if (!in.is_open()) {
    return Status::kIoError;
  }

  // allow_exceptions=false: a malformed file comes back as a discarded value
  // rather than a throw, which is what lets this stay exception-free.
  const nlohmann::json j =
      nlohmann::json::parse(in, nullptr, /*allow_exceptions=*/false);
  if (j.is_discarded()) return Status::kInvalidArgument;

  out.features.clear();

  if (j.contains("frame")) {
    out.frame = j["frame"].get<int>();
  }

  if (j.contains("features") && j["features"].is_array()) {
    for (const auto& item : j["features"]) {
      out.features.push_back(ParsePolyline2D(item));
    }
  }

  return Status::kOk;
}

}  // namespace cam_loc::kitti
