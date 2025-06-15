#ifndef CAM_LOC_VIZ_FRAME_VIZ_H_
#define CAM_LOC_VIZ_FRAME_VIZ_H_

/// Offline PNG visualization for localization debug snapshots.
///
/// Renders camera overlay, distance transforms, BEV, cost-grid slice, top-down
/// map view, and a stitched panel per frame. Used by viz_frame and eval
/// tooling.

#include <string>
#include <vector>

#include "cam_loc/core/bev_config.h"
#include "cam_loc/core/cost_grid.h"
#include "cam_loc/core/localization_debug.h"
#include "cam_loc/core/projection.h"
#include "cam_loc/kitti/types.h"
#include "cam_loc/types/params.h"
#include "cam_loc/types/status.h"

namespace cam_loc::viz {

struct Rgb {
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
};

struct RgbImage {
  int width = 0;
  int height = 0;
  std::vector<uint8_t> rgb;  // row-major RGB8

  void Resize(int w, int h, uint8_t r = 32, uint8_t g = 32, uint8_t b = 32);
  void SetPixel(int x, int y, Rgb c);
  Rgb GetPixel(int x, int y) const;
};

Status LoadRgbImage(const std::string& path, RgbImage& out);
Status WritePng(const std::string& path, const RgbImage& img);

// --- Primitive drawing ---

void DrawLine(RgbImage& img, int x0, int y0, int x1, int y1, Rgb color,
              int thickness = 1);
void DrawPolyline2D(RgbImage& img, const std::vector<Vec2>& points, Rgb color,
                    int thickness = 2);
void DrawCross(RgbImage& img, int cx, int cy, int half_len, Rgb color,
               int thickness = 2);

// --- Layer renderers (map, perception, costs) ---

RgbImage RenderDistanceTransform(const core::LabelledDistanceTransform& dt,
                                 float max_dist = -1.f);
RgbImage RenderCostSliceXy(const core::CostGrid& grid, int yaw_index,
                           float* out_min = nullptr, float* out_max = nullptr);

void ProjectMapPolylinesToImage(const kitti::MapChunk& map,
                                const core::Projection& projection,
                                const Mat44& T_world_cam, RgbImage& img,
                                Rgb color, int thickness = 2);

void DrawPerceptionOnImage(const kitti::FramePerception& perception,
                           RgbImage& img);

RgbImage RenderBevOverlay(const kitti::MapChunk& map,
                          const kitti::FramePerception& perception,
                          const core::Projection& projection,
                          const Mat44& T_world_cam);

struct TrajectoryPoint {
  double x = 0.0;
  double z = 0.0;
};

RgbImage RenderTopDown(const kitti::MapChunk& map, const TrajectoryPoint& gt,
                       const TrajectoryPoint& estimate,
                       const TrajectoryPoint& sampling_plane,
                       double half_range_m = 30.0);

/// Inputs for RenderFrameViz: debug snapshot, projection, result, optional GT
/// pose/camera.
struct FrameVizInput {
  int frame = 0;
  const core::LocalizationDebugSnapshot* debug = nullptr;
  const core::Projection* projection = nullptr;
  const LocalizationResult* result = nullptr;
  const kitti::Pose* gt_pose = nullptr;
  RgbImage camera;  // optional background; empty if unavailable
};

/// List of PNG/JSON paths written by RenderFrameViz.
struct FrameVizOutput {
  std::string output_dir;
  std::vector<std::string> written_files;
};

/// Write per-layer PNGs plus a stitched panel for one frame.
Status RenderFrameViz(const FrameVizInput& input, const std::string& output_dir,
                      FrameVizOutput& out);

/// GT vs estimate top-down paths for sequence-level PNG export.
struct TrajectoryVizInput {
  std::vector<TrajectoryPoint> gt;
  std::vector<TrajectoryPoint> estimate;
};

Status RenderTrajectoryViz(const TrajectoryVizInput& input,
                           const std::string& output_path);

}  // namespace cam_loc::viz

#endif  // CAM_LOC_VIZ_FRAME_VIZ_H_
