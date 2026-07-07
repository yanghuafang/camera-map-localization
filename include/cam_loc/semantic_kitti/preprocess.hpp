#pragma once

/// SemanticKITTI / KITTI label raster → cam_loc FramePerception conversion.
///
/// Supports 16-bit label PNG row-scan and LiDAR point-label projection paths.

#include <cam_loc/kitti/types.hpp>
#include <cam_loc/types/status.hpp>

#include <string>
#include <vector>

namespace cam_loc::semantic_kitti {

/// Semantic KITTI label IDs (commonly used classes).
enum LabelId : uint16_t {
  kRoad = 40,
  kParking = 44,
  kSidewalk = 48,
  kLaneMarking = 60,
  kTerrain = 72,
};

/// Tunables for label raster → polyline extraction (row stride, min run length).
struct PreprocessOptions {
  int frame = 0;
  int image_width = 1242;
  int image_height = 375;
  int row_stride = 4;
  int min_run_length = 8;
};

/// Load uint16 label image (Semantic KITTI .label.png stored as 16-bit grayscale).
Status loadLabelImage16(const std::string& path, int expected_width, int expected_height,
                        std::vector<uint16_t>& out_labels);

/// Extract lane / road-boundary polylines from a label raster.
Status labelsToPerception(const std::vector<uint16_t>& labels, int width, int height,
                          const PreprocessOptions& opts, kitti::FramePerception& out);

Status writePerceptionJson(const std::string& path, const kitti::FramePerception& perception);

}  // namespace cam_loc::semantic_kitti
