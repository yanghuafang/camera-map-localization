/// SemanticKITTI label raster → localization landmarks.

#include "cam_loc/semantic_kitti/preprocess.h"

#include <fstream>

#include <nlohmann/json.hpp>

// stb is header-only, and this translation unit is where the image reader's
// implementation is compiled; see the note in viz/frame_viz.cc for why the
// macro and its include sit last and apart from the sorted blocks above.
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

namespace cam_loc::semantic_kitti {

namespace {

/// Reads a label raster without every caller repeating the index arithmetic.
class LabelImage {
 public:
  LabelImage(const std::vector<uint16_t>& labels, int width, int height)
      : labels_(labels), width_(width), height_(height) {}

  int width() const { return width_; }
  int height() const { return height_; }

  uint16_t At(int x, int y) const {
    return labels_[static_cast<size_t>(y) * static_cast<size_t>(width_) +
                   static_cast<size_t>(x)];
  }

 private:
  const std::vector<uint16_t>& labels_;
  int width_;
  int height_;
};

void AppendPolyline(kitti::FramePerception& out, kitti::PolylineType type,
                    std::vector<Vec2> points) {
  if (points.size() < 2) return;
  kitti::Polyline2D pl;
  pl.type = type;
  pl.points = std::move(points);
  out.features.push_back(std::move(pl));
}

/// Trace horizontal runs of one class, one polyline per run.
void ScanRows(const LabelImage& image, uint16_t want, kitti::PolylineType type,
              const PreprocessOptions& opts, kitti::FramePerception& out) {
  for (int y = 0; y < image.height(); y += opts.scan_stride) {
    std::vector<Vec2> run;
    for (int x = 0; x < image.width(); ++x) {
      if (image.At(x, y) == want) {
        run.emplace_back(x, y);
        continue;
      }
      // The run ended. Its class is `want` by construction -- taking the type
      // from the pixel that *terminated* it, as this once did, can only ever
      // read a pixel of some other class.
      if (static_cast<int>(run.size()) >= opts.min_run_length) {
        AppendPolyline(out, type, std::move(run));
      }
      run.clear();
    }
    if (static_cast<int>(run.size()) >= opts.min_run_length) {
      AppendPolyline(out, type, std::move(run));
    }
  }
}

/// Trace vertical runs of one class. Upright landmarks are a few pixels wide
/// and many tall, so this is the orientation that finds them.
void ScanColumns(const LabelImage& image, uint16_t want,
                 kitti::PolylineType type, const PreprocessOptions& opts,
                 kitti::FramePerception& out) {
  for (int x = 0; x < image.width(); x += opts.scan_stride) {
    std::vector<Vec2> run;
    for (int y = 0; y < image.height(); ++y) {
      if (image.At(x, y) == want) {
        run.emplace_back(x, y);
        continue;
      }
      if (static_cast<int>(run.size()) >= opts.min_run_length) {
        AppendPolyline(out, type, std::move(run));
      }
      run.clear();
    }
    if (static_cast<int>(run.size()) >= opts.min_run_length) {
      AppendPolyline(out, type, std::move(run));
    }
  }
}

/// Trace the left and right edges of the drivable surface.
///
/// One point per scanned row on each side, linked into two polylines. Rows with
/// no road pixel simply contribute nothing, so a boundary that leaves the image
/// and comes back is one polyline with a gap rather than two.
void ScanRoadBoundaries(const LabelImage& image, const PreprocessOptions& opts,
                        kitti::FramePerception& out) {
  std::vector<Vec2> left;
  std::vector<Vec2> right;
  for (int y = 0; y < image.height(); y += opts.scan_stride) {
    int first = -1;
    int last = -1;
    for (int x = 0; x < image.width(); ++x) {
      if (image.At(x, y) != kRoad) continue;
      if (first < 0) first = x;
      last = x;
    }
    if (first < 0 || last - first < opts.min_run_length) continue;
    left.emplace_back(first, y);
    right.emplace_back(last, y);
  }
  AppendPolyline(out, kitti::PolylineType::kRoadEdge, std::move(left));
  AppendPolyline(out, kitti::PolylineType::kRoadEdge, std::move(right));
}

}  // namespace

Status LoadLabelImage16(const std::string& path, int expected_width,
                        int expected_height,
                        std::vector<uint16_t>& out_labels) {
  int w = 0;
  int h = 0;
  int comp = 0;
  auto* data = reinterpret_cast<uint16_t*>(
      stbi_load_16(path.c_str(), &w, &h, &comp, STBI_grey));
  if (data == nullptr) {
    return Status::kIoError;
  }
  if (w != expected_width || h != expected_height) {
    stbi_image_free(data);
    return Status::kInvalidArgument;
  }
  out_labels.assign(data, data + static_cast<size_t>(w) * h);
  stbi_image_free(data);
  return Status::kOk;
}

Status LabelsToPerception(const std::vector<uint16_t>& labels, int width,
                          int height, const PreprocessOptions& opts,
                          kitti::FramePerception& out) {
  if (width <= 0 || height <= 0 ||
      labels.size() != static_cast<size_t>(width) * height) {
    return Status::kInvalidArgument;
  }
  out.frame = opts.frame;
  out.features.clear();

  const LabelImage image(labels, width, height);
  ScanRows(image, kLaneMarking, kitti::PolylineType::kLaneSolid, opts, out);
  ScanRoadBoundaries(image, opts, out);
  // The two classes that constrain along-track position. Without them the cost
  // surface is flat along the road, however good the lane detection is.
  ScanColumns(image, kPole, kitti::PolylineType::kPole, opts, out);
  ScanColumns(image, kTrafficSign, kitti::PolylineType::kSign, opts, out);
  return Status::kOk;
}

Status WritePerceptionJson(const std::string& path,
                           const kitti::FramePerception& perception) {
  nlohmann::json j;
  j["frame"] = perception.frame;
  j["features"] = nlohmann::json::array();
  for (const auto& pl : perception.features) {
    nlohmann::json pj;
    pj["type"] = kitti::PolylineTypeToString(pl.type);
    nlohmann::json pts = nlohmann::json::array();
    for (const auto& p : pl.points) {
      pts.push_back(nlohmann::json::array({p.x(), p.y()}));
    }
    pj["points"] = pts;
    j["features"].push_back(std::move(pj));
  }

  std::ofstream out(path);
  if (!out.is_open()) return Status::kIoError;
  out << j.dump(2);
  return out.good() ? Status::kOk : Status::kIoError;
}

}  // namespace cam_loc::semantic_kitti
