/// Frame viz implementation: layer PNGs, composite panel, and trajectory
/// overlay.
#include "cam_loc/viz/frame_viz.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>

#include <nlohmann/json.hpp>

// stb is header-only, and this translation unit is where the PNG writer's
// implementation is compiled: the macro has to precede the include that pulls
// it in. The pair sits last and apart because a #define ends an include block
// for clang-format, so anything sorted after it would be sorted separately --
// which is how the macro previously came to sit above <algorithm>, reading as
// if it had anything to do with it.
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image.h>
#include <stb_image_write.h>

namespace cam_loc::viz {

namespace {

constexpr int kImageDtWidth = 1241;
constexpr int kImageDtHeight = 376;

constexpr Rgb kGreen{60, 220, 80};
constexpr Rgb kCyan{80, 220, 255};
constexpr Rgb kYellow{255, 220, 60};
constexpr Rgb kOrange{255, 140, 40};
constexpr Rgb kRed{255, 60, 60};
constexpr Rgb kWhite{240, 240, 240};
constexpr Rgb kMagenta{255, 60, 255};
constexpr Rgb kBlue{120, 160, 255};
constexpr Rgb kPink{255, 150, 190};

/// One colour per perception class, so a rendered frame says which landmark is
/// which. Perception is drawn in cool colours and the map in warm ones, so the
/// two are told apart at a glance where they overlap.
Rgb PerceptionTypeColor(kitti::PolylineType type) {
  switch (type) {
    case kitti::PolylineType::kRoadEdge:
      return kCyan;
    case kitti::PolylineType::kPole:
      return kBlue;
    case kitti::PolylineType::kSign:
      return kMagenta;
    default:
      return kGreen;
  }
}

/// One colour per map class.
Rgb MapTypeColor(kitti::PolylineType type) {
  switch (type) {
    case kitti::PolylineType::kRoadEdge:
      return kOrange;
    case kitti::PolylineType::kPole:
      return kBlue;
    case kitti::PolylineType::kSign:
      return kPink;
    default:
      return kYellow;
  }
}

void PushFile(FrameVizOutput& out, const std::string& path) {
  out.written_files.push_back(path);
}

Rgb JetColormap(float t) {
  t = std::clamp(t, 0.f, 1.f);
  const float r = std::clamp(1.5f * std::abs(t - 0.75f) - 0.5f, 0.f, 1.f);
  const float g = std::clamp(1.5f * std::abs(t - 0.50f) - 0.5f, 0.f, 1.f);
  const float b = std::clamp(1.5f * std::abs(t - 0.25f) - 0.5f, 0.f, 1.f);
  return {static_cast<uint8_t>(r * 255.f), static_cast<uint8_t>(g * 255.f),
          static_cast<uint8_t>(b * 255.f)};
}

RgbImage ResizeNearest(const RgbImage& src, int new_w, int new_h) {
  RgbImage out;
  out.Resize(new_w, new_h);
  if (src.width <= 0 || src.height <= 0) return out;
  for (int y = 0; y < new_h; ++y) {
    const int sy = y * src.height / new_h;
    for (int x = 0; x < new_w; ++x) {
      const int sx = x * src.width / new_w;
      out.SetPixel(x, y, src.GetPixel(sx, sy));
    }
  }
  return out;
}

void Blit(const RgbImage& src, RgbImage& dst, int ox, int oy) {
  for (int y = 0; y < src.height; ++y) {
    const int dy = oy + y;
    if (dy < 0 || dy >= dst.height) continue;
    for (int x = 0; x < src.width; ++x) {
      const int dx = ox + x;
      if (dx < 0 || dx >= dst.width) continue;
      dst.SetPixel(dx, dy, src.GetPixel(x, y));
    }
  }
}

TrajectoryPoint PoseToTopDown(const Mat44& T) {
  TrajectoryPoint p;
  p.x = T(0, 3);
  p.z = T(2, 3);
  return p;
}

/// Affine world (X, Z) -> pixel with Y-flip, shared by top-down and trajectory
/// views.
Vec2 ProjectTopDown(double x, double z, double ref_x, double ref_z,
                    double scale, double px_base, double py_base) {
  return Vec2(px_base + (x - ref_x) * scale, py_base - (z - ref_z) * scale);
}

std::string FormatFrameId(int frame) {
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%06d", frame);
  return buf;
}

}  // namespace

void RgbImage::Resize(int w, int h, uint8_t r, uint8_t g, uint8_t b) {
  width = w;
  height = h;
  rgb.assign(static_cast<size_t>(w * h * 3), 0);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      SetPixel(x, y, {r, g, b});
    }
  }
}

void RgbImage::SetPixel(int x, int y, Rgb c) {
  if (x < 0 || y < 0 || x >= width || y >= height) return;
  const auto i = static_cast<size_t>((y * width + x) * 3);
  rgb[i] = c.r;
  rgb[i + 1] = c.g;
  rgb[i + 2] = c.b;
}

Rgb RgbImage::GetPixel(int x, int y) const {
  if (x < 0 || y < 0 || x >= width || y >= height) return {0, 0, 0};
  const auto i = static_cast<size_t>((y * width + x) * 3);
  return {rgb[i], rgb[i + 1], rgb[i + 2]};
}

Status LoadRgbImage(const std::string& path, RgbImage& out) {
  int w = 0, h = 0, comp = 0;
  unsigned char* data = stbi_load(path.c_str(), &w, &h, &comp, 3);
  if (data == nullptr) {
    out = RgbImage{};
    return Status::kIoError;
  }
  out.width = w;
  out.height = h;
  out.rgb.assign(data, data + static_cast<size_t>(w * h * 3));
  stbi_image_free(data);
  return Status::kOk;
}

Status WritePng(const std::string& path, const RgbImage& img) {
  if (img.width <= 0 || img.height <= 0 || img.rgb.empty()) {
    return Status::kInvalidArgument;
  }
  std::filesystem::create_directories(
      std::filesystem::path(path).parent_path());
  if (stbi_write_png(path.c_str(), img.width, img.height, 3, img.rgb.data(),
                     img.width * 3) == 0) {
    return Status::kIoError;
  }
  return Status::kOk;
}

void DrawLine(RgbImage& img, int x0, int y0, int x1, int y1, Rgb color,
              int thickness) {
  const int dx = std::abs(x1 - x0);
  const int dy = std::abs(y1 - y0);
  const int sx = x0 < x1 ? 1 : -1;
  const int sy = y0 < y1 ? 1 : -1;
  int err = dx - dy;
  int x = x0;
  int y = y0;
  while (true) {
    for (int ty = -thickness; ty <= thickness; ++ty) {
      for (int tx = -thickness; tx <= thickness; ++tx) {
        img.SetPixel(x + tx, y + ty, color);
      }
    }
    if (x == x1 && y == y1) break;
    const int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x += sx;
    }
    if (e2 < dx) {
      err += dx;
      y += sy;
    }
  }
}

void DrawPolyline2D(RgbImage& img, const std::vector<Vec2>& points, Rgb color,
                    int thickness) {
  if (points.size() < 2) return;
  for (size_t i = 1; i < points.size(); ++i) {
    DrawLine(img, static_cast<int>(std::lround(points[i - 1].x())),
             static_cast<int>(std::lround(points[i - 1].y())),
             static_cast<int>(std::lround(points[i].x())),
             static_cast<int>(std::lround(points[i].y())), color, thickness);
  }
}

void DrawCross(RgbImage& img, int cx, int cy, int half_len, Rgb color,
               int thickness) {
  DrawLine(img, cx - half_len, cy, cx + half_len, cy, color, thickness);
  DrawLine(img, cx, cy - half_len, cx, cy + half_len, color, thickness);
}

RgbImage RenderDistanceTransform(const core::LabelledDistanceTransform& dt,
                                 float max_dist) {
  RgbImage out;
  if (dt.width <= 0 || dt.height <= 0) return out;
  out.Resize(dt.width, dt.height, 20, 20, 24);
  float max_d = max_dist;
  if (max_d <= 0.f) {
    max_d = 1.f;
    for (float v : dt.distance) {
      if (std::isfinite(v)) max_d = std::max(max_d, v);
    }
  }
  if (max_d < 1e-3f) max_d = 1.f;
  for (int y = 0; y < dt.height; ++y) {
    for (int x = 0; x < dt.width; ++x) {
      const float d = dt.distance[static_cast<size_t>(y * dt.width + x)];
      const float t = 1.f - std::clamp(d / max_d, 0.f, 1.f);
      out.SetPixel(x, y, JetColormap(t));
    }
  }
  return out;
}

RgbImage RenderCostSliceXy(const core::CostGrid& grid, int yaw_index,
                           float* out_min, float* out_max) {
  const int dim_x = grid.DimX();
  const int dim_y = grid.DimY();
  RgbImage out;
  out.Resize(dim_x, dim_y, 16, 16, 20);

  float min_c = std::numeric_limits<float>::max();
  float max_c = -std::numeric_limits<float>::max();
  for (int iy = 0; iy < dim_y; ++iy) {
    for (int ix = 0; ix < dim_x; ++ix) {
      const float c = grid.At(ix, iy, yaw_index);
      min_c = std::min(min_c, c);
      max_c = std::max(max_c, c);
    }
  }
  if (out_min != nullptr) *out_min = min_c;
  if (out_max != nullptr) *out_max = max_c;
  const float spread = std::max(max_c - min_c, 1e-3f);

  for (int iy = 0; iy < dim_y; ++iy) {
    for (int ix = 0; ix < dim_x; ++ix) {
      const float c = grid.At(ix, iy, yaw_index);
      const float t = 1.f - std::clamp((c - min_c) / spread, 0.f, 1.f);
      out.SetPixel(ix, dim_y - 1 - iy, JetColormap(t));
    }
  }
  return out;
}

void ProjectMapPolylinesToImage(const kitti::MapChunk& map,
                                const core::Projection& projection,
                                const Mat44& T_world_cam, RgbImage& img,
                                Rgb color, int thickness) {
  for (const auto& mpl : map.polylines) {
    std::vector<Vec2> projected;
    for (const auto& p_world : mpl.points) {
      Vec2 uv;
      if (projection.WorldToImage(T_world_cam, p_world, uv) != Status::kOk)
        continue;
      projected.push_back(uv);
    }
    DrawPolyline2D(img, projected, color, thickness);
  }
}

void DrawPerceptionOnImage(const kitti::FramePerception& perception,
                           RgbImage& img) {
  for (const auto& pl : perception.features) {
    DrawPolyline2D(img, pl.points, PerceptionTypeColor(pl.type), 2);
  }
}

RgbImage RenderBevOverlay(const kitti::MapChunk& map,
                          const kitti::FramePerception& perception,
                          const core::Projection& projection,
                          const Mat44& T_world_cam) {
  RgbImage img;
  img.Resize(core::BevConfig::kImageWidth, core::BevConfig::kImageHeight, 24,
             24, 28);

  for (const auto& mpl : map.polylines) {
    std::vector<Vec2> pts;
    for (const auto& p_world : mpl.points) {
      int col = 0;
      int row = 0;
      if (core::Projection::WorldToBevPixel(T_world_cam, p_world, col, row) !=
          Status::kOk)
        continue;
      pts.emplace_back(static_cast<double>(col), static_cast<double>(row));
    }
    DrawPolyline2D(img, pts, MapTypeColor(mpl.type), 2);
  }

  auto draw_bev_polylines =
      [&](const std::vector<kitti::Polyline2D>& polylines) {
        for (const auto& pl : polylines) {
          std::vector<Vec2> pts;
          // Only ground-plane classes have a place in a top-down view; an
          // elevated pole put through inverse perspective would be drawn at
          // whatever range that assumption implies.
          if (!kitti::IsGroundPlaneType(pl.type)) continue;
          for (const auto& uv_img : pl.points) {
            Vec3 p_rig;
            if (projection.ImageToGroundRig(uv_img, p_rig) != Status::kOk)
              continue;
            int col = 0;
            int row = 0;
            if (cam_loc::core::Projection::RigToBevPixel(p_rig, col, row) !=
                Status::kOk)
              continue;
            pts.emplace_back(static_cast<double>(col),
                             static_cast<double>(row));
          }
          DrawPolyline2D(img, pts, PerceptionTypeColor(pl.type), 2);
        }
      };
  draw_bev_polylines(perception.features);

  return img;
}

RgbImage RenderTopDown(const kitti::MapChunk& map, const TrajectoryPoint& gt,
                       const TrajectoryPoint& estimate,
                       const TrajectoryPoint& sampling_plane,
                       double half_range_m) {
  constexpr int kSize = 500;
  RgbImage img;
  img.Resize(kSize, kSize, 18, 20, 24);

  const double cx = sampling_plane.x;
  const double cz = sampling_plane.z;
  const double scale = (kSize - 40) / (2.0 * half_range_m);

  auto to_pix = [&](double x, double z) -> Vec2 {
    return ProjectTopDown(x, z, cx, cz, scale, kSize * 0.5, kSize * 0.5);
  };

  for (const auto& mpl : map.polylines) {
    std::vector<Vec2> pts;
    pts.reserve(mpl.points.size());
    for (const auto& p : mpl.points) {
      pts.push_back(to_pix(p.x(), p.z()));
    }
    DrawPolyline2D(img, pts, MapTypeColor(mpl.type), 2);
  }

  const Vec2 gt_p = to_pix(gt.x, gt.z);
  const Vec2 est_p = to_pix(estimate.x, estimate.z);
  const Vec2 plane_p = to_pix(sampling_plane.x, sampling_plane.z);
  DrawCross(img, static_cast<int>(gt_p.x()), static_cast<int>(gt_p.y()), 8,
            kWhite, 2);
  DrawCross(img, static_cast<int>(est_p.x()), static_cast<int>(est_p.y()), 8,
            kRed, 2);
  DrawCross(img, static_cast<int>(plane_p.x()), static_cast<int>(plane_p.y()),
            5, kMagenta, 1);
  return img;
}

Status RenderFrameViz(const FrameVizInput& input, const std::string& output_dir,
                      FrameVizOutput& out) {
  out = FrameVizOutput{};
  out.output_dir = output_dir;
  if ((input.debug == nullptr) || !input.debug->valid ||
      (input.projection == nullptr) || (input.result == nullptr)) {
    return Status::kInvalidArgument;
  }

  std::filesystem::create_directories(output_dir);
  const std::string prefix =
      output_dir + "/frame_" + FormatFrameId(input.frame);

  const auto& dbg = *input.debug;

  // Layer 1: camera image with perception + projected map polylines.
  RgbImage camera;
  if (input.camera.width > 0) {
    camera = input.camera;
  } else {
    camera.Resize(kImageDtWidth, kImageDtHeight, 40, 40, 44);
  }
  DrawPerceptionOnImage(dbg.perception, camera);
  ProjectMapPolylinesToImage(dbg.local_map, *input.projection,
                             dbg.T_world_plane, camera, kYellow, 2);
  const std::string camera_path = prefix + "_camera.png";
  if (WritePng(camera_path, camera) != Status::kOk) return Status::kIoError;
  PushFile(out, camera_path);

  // Layer 2–5: image DT, BEV overlay, optional BEV DT, cost-grid XY slice at
  // best yaw.
  const RgbImage image_dt = RenderDistanceTransform(dbg.image_dt);
  const std::string image_dt_path = prefix + "_image_dt.png";
  if (WritePng(image_dt_path, image_dt) != Status::kOk) return Status::kIoError;
  PushFile(out, image_dt_path);

  const RgbImage bev = RenderBevOverlay(dbg.local_map, dbg.perception,
                                        *input.projection, dbg.T_world_plane);
  const std::string bev_path = prefix + "_bev.png";
  if (WritePng(bev_path, bev) != Status::kOk) return Status::kIoError;
  PushFile(out, bev_path);

  if (dbg.has_bev_dt) {
    const RgbImage bev_dt = RenderDistanceTransform(dbg.bev_dt);
    const std::string bev_dt_path = prefix + "_bev_dt.png";
    if (WritePng(bev_dt_path, bev_dt) != Status::kOk) return Status::kIoError;
    PushFile(out, bev_dt_path);
  }

  RgbImage cost_xy = RenderCostSliceXy(dbg.aggregated_costs, dbg.argmin.iw);
  DrawCross(cost_xy, dbg.argmin.ix, cost_xy.height - 1 - dbg.argmin.iy, 6,
            kMagenta, 2);
  const std::string cost_path = prefix + "_cost_xy.png";
  if (WritePng(cost_path, cost_xy) != Status::kOk) return Status::kIoError;
  PushFile(out, cost_path);

  // Layer 6: top-down map with GT (white), estimate (red), sampling plane
  // (magenta).
  TrajectoryPoint gt_pt{};
  TrajectoryPoint est_pt = PoseToTopDown(input.result->T_world_rig);
  TrajectoryPoint plane_pt = PoseToTopDown(dbg.T_world_plane);
  if (input.gt_pose != nullptr) {
    gt_pt = PoseToTopDown(input.gt_pose->T_world_cam0);
  } else {
    gt_pt = plane_pt;
  }
  const RgbImage topdown =
      RenderTopDown(dbg.local_map, gt_pt, est_pt, plane_pt);
  const std::string topdown_path = prefix + "_topdown.png";
  if (WritePng(topdown_path, topdown) != Status::kOk) return Status::kIoError;
  PushFile(out, topdown_path);

  // Composite panel.
  const RgbImage cam_small = ResizeNearest(camera, 620, 188);
  const RgbImage dt_small = ResizeNearest(image_dt, 620, 188);
  const RgbImage bev_small = ResizeNearest(bev, 350, 188);
  const RgbImage cost_small = ResizeNearest(cost_xy, 350, 350);
  const RgbImage top_small = ResizeNearest(topdown, 620, 350);

  const int panel_w = 620 + 20 + 350;
  const int panel_h = 188 + 20 + 188 + 20 + 350;
  RgbImage panel;
  panel.Resize(panel_w, panel_h, 12, 12, 16);
  Blit(cam_small, panel, 0, 0);
  Blit(bev_small, panel, 640, 0);
  Blit(dt_small, panel, 0, 208);
  Blit(cost_small, panel, 640, 208);
  Blit(top_small, panel, 0, 416);
  const std::string panel_path = prefix + "_panel.png";
  if (WritePng(panel_path, panel) != Status::kOk) return Status::kIoError;
  PushFile(out, panel_path);

  nlohmann::json meta;
  meta["frame"] = input.frame;
  meta["aggregate_min_cost"] = input.result->aggregate_min_cost;
  meta["cost_map_spread"] = input.result->cost_map_spread;
  meta["cost_map_flat"] = input.result->cost_map_flat;
  meta["match_cost_too_high"] = input.result->match_cost_too_high;
  meta["sampling_applied"] = input.result->sampling_measurement_applied;
  meta["best_offset_m"] = input.result->best_offset_norm_m;
  meta["best_sample_xy_m"] = {input.result->best_sample_xyyaw.x(),
                              input.result->best_sample_xyyaw.y()};
  meta["best_sample_yaw_deg"] =
      input.result->best_sample_xyyaw.z() * 180.0 / M_PI;
  meta["files"] = out.written_files;
  const std::string meta_path = prefix + "_meta.json";
  std::ofstream meta_out(meta_path);
  meta_out << meta.dump(2) << "\n";
  PushFile(out, meta_path);

  return Status::kOk;
}

Status RenderTrajectoryViz(const TrajectoryVizInput& input,
                           const std::string& output_path) {
  if (input.gt.empty()) return Status::kInvalidArgument;

  constexpr int kW = 800;
  constexpr int kH = 800;
  RgbImage img;
  img.Resize(kW, kH, 18, 20, 24);

  double min_x = input.gt.front().x;
  double max_x = input.gt.front().x;
  double min_z = input.gt.front().z;
  double max_z = input.gt.front().z;
  auto expand = [&](const TrajectoryPoint& p) {
    min_x = std::min(min_x, p.x);
    max_x = std::max(max_x, p.x);
    min_z = std::min(min_z, p.z);
    max_z = std::max(max_z, p.z);
  };
  for (const auto& p : input.gt) expand(p);
  for (const auto& p : input.estimate) expand(p);
  const double pad = 5.0;
  min_x -= pad;
  max_x += pad;
  min_z -= pad;
  max_z += pad;
  const double sx = (kW - 40) / std::max(max_x - min_x, 1.0);
  const double sz = (kH - 40) / std::max(max_z - min_z, 1.0);
  const double scale = std::min(sx, sz);

  auto to_pix = [&](const TrajectoryPoint& p) -> Vec2 {
    return ProjectTopDown(p.x, p.z, min_x, min_z, scale, 20.0,
                          static_cast<double>(kH) - 20.0);
  };

  auto draw_path = [&](const std::vector<TrajectoryPoint>& path, Rgb color) {
    if (path.size() < 2) return;
    for (size_t i = 1; i < path.size(); ++i) {
      const Vec2 a = to_pix(path[i - 1]);
      const Vec2 b = to_pix(path[i]);
      DrawLine(img, static_cast<int>(a.x()), static_cast<int>(a.y()),
               static_cast<int>(b.x()), static_cast<int>(b.y()), color, 2);
    }
  };

  draw_path(input.gt, kWhite);
  draw_path(input.estimate, kRed);
  if (!input.gt.empty()) {
    const Vec2 p = to_pix(input.gt.back());
    DrawCross(img, static_cast<int>(p.x()), static_cast<int>(p.y()), 6, kWhite,
              2);
  }
  if (!input.estimate.empty()) {
    const Vec2 p = to_pix(input.estimate.back());
    DrawCross(img, static_cast<int>(p.x()), static_cast<int>(p.y()), 6, kRed,
              2);
  }

  return WritePng(output_path, img);
}

}  // namespace cam_loc::viz
