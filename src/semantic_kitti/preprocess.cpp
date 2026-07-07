/// Label raster → lane/boundary polyline extraction (horizontal row scans).
#include <cam_loc/semantic_kitti/preprocess.hpp>

#include <nlohmann/json.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <fstream>
#include <unordered_map>

namespace cam_loc::semantic_kitti {

namespace {

bool isLane(uint16_t id) { return id == kLaneMarking; }
bool isBoundary(uint16_t id) { return id == kSidewalk || id == kTerrain || id == kRoad; }

kitti::PolylineType boundaryType(uint16_t id) {
  if (id == kSidewalk || id == kTerrain) return kitti::PolylineType::kRoadEdge;
  return kitti::PolylineType::kLaneSolid;
}

void appendPolyline(std::vector<kitti::Polyline2D>& dst, kitti::PolylineType type,
                    const std::vector<Vec2>& pts) {
  if (pts.size() < 2) return;
  kitti::Polyline2D pl;
  pl.type = type;
  pl.points = pts;
  dst.push_back(std::move(pl));
}

}  // namespace

Status loadLabelImage16(const std::string& path, int expected_width, int expected_height,
                        std::vector<uint16_t>& out_labels) {
  int w = 0, h = 0, comp = 0;
  uint16_t* data = reinterpret_cast<uint16_t*>(
      stbi_load_16(path.c_str(), &w, &h, &comp, STBI_grey));
  if (!data) {
    return Status::kIoError;
  }
  if (w != expected_width || h != expected_height) {
    stbi_image_free(data);
    return Status::kInvalidArgument;
  }
  out_labels.assign(static_cast<size_t>(w * h), 0);
  for (int i = 0; i < w * h; ++i) {
    out_labels[static_cast<size_t>(i)] = data[i];
  }
  stbi_image_free(data);
  return Status::kOk;
}

Status labelsToPerception(const std::vector<uint16_t>& labels, int width, int height,
                          const PreprocessOptions& opts, kitti::FramePerception& out) {
  if (static_cast<int>(labels.size()) != width * height) {
    return Status::kInvalidArgument;
  }
  out.frame = opts.frame;
  out.lane_lines.clear();
  out.road_boundaries.clear();

  auto scan_rows = [&](bool lane_mode) {
    for (int y = 0; y < height; y += opts.row_stride) {
      std::vector<Vec2> run;
      auto flush = [&](kitti::PolylineType type) {
        appendPolyline(lane_mode ? out.lane_lines : out.road_boundaries, type, run);
        run.clear();
      };

      for (int x = 0; x < width; ++x) {
        const uint16_t id = labels[static_cast<size_t>(y * width + x)];
        const bool hit = lane_mode ? isLane(id) : isBoundary(id);
        if (hit) {
          run.emplace_back(static_cast<double>(x), static_cast<double>(y));
        } else if (run.size() >= static_cast<size_t>(opts.min_run_length)) {
          flush(lane_mode ? kitti::PolylineType::kLaneSolid : boundaryType(id));
        } else {
          run.clear();
        }
      }
      if (run.size() >= static_cast<size_t>(opts.min_run_length)) {
        flush(lane_mode ? kitti::PolylineType::kLaneSolid : kitti::PolylineType::kRoadEdge);
      }
    }
  };

  scan_rows(true);
  scan_rows(false);
  return Status::kOk;
}

Status writePerceptionJson(const std::string& path, const kitti::FramePerception& perception) {
  nlohmann::json j;
  j["frame"] = perception.frame;
  auto polyToJson = [](const kitti::Polyline2D& pl) {
    nlohmann::json pj;
    pj["type"] = kitti::polylineTypeToString(pl.type);
    nlohmann::json pts = nlohmann::json::array();
    for (const auto& p : pl.points) {
      pts.push_back(nlohmann::json::array({p.x(), p.y()}));
    }
    pj["points"] = pts;
    return pj;
  };
  j["lane_lines"] = nlohmann::json::array();
  for (const auto& pl : perception.lane_lines) {
    j["lane_lines"].push_back(polyToJson(pl));
  }
  j["road_boundaries"] = nlohmann::json::array();
  for (const auto& pl : perception.road_boundaries) {
    j["road_boundaries"].push_back(polyToJson(pl));
  }

  std::ofstream out(path);
  if (!out.is_open()) return Status::kIoError;
  out << j.dump(2);
  return Status::kOk;
}

}  // namespace cam_loc::semantic_kitti
