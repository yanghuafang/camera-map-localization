#pragma once

/// Map-matching pose search: build perception DTs and score a 3-DOF (x, y, yaw) cost grid.

#include <cam_loc/core/cost_grid.hpp>
#include <cam_loc/core/projection.hpp>
#include <cam_loc/kitti/types.hpp>
#include <cam_loc/types/params.hpp>
#include <cam_loc/types/status.hpp>

namespace cam_loc::core {

/// Per-pixel Euclidean distance to nearest perception feature, plus type labels.
/// Binary raster convention: 0 = feature stroke, 255 = background.
struct LabelledDistanceTransform {
  std::vector<float> distance;
  std::vector<uint8_t> labels;
  int width = 0;
  int height = 0;
  float max_cost = 5.f;
};

/// Pose-sampling pipeline: rasterize perception → DT → score map points per hypothesis.
///
/// Pipeline per frame:
///   1. buildImageDt / buildBevDtFromImagePerception — rasterize lanes/edges, run EDT
///   2. computeImageCosts / computeBevCosts — for each CostGrid cell, transform map
///      points to rig, project (image) or rasterize (BEV), bilinearly sample DT
///   3. Type-mismatch between map point and DT label returns max_cost (hard gate)
class PoseSampler {
 public:
  explicit PoseSampler(const LocalizationParams& params);

  void setProjection(const Projection& projection);

  Status buildImageDt(const kitti::FramePerception& perception, LabelledDistanceTransform& out);

  Status buildBevDtFromImagePerception(const kitti::FramePerception& perception,
                                       LabelledDistanceTransform& out);

  Status computeImageCosts(const kitti::MapChunk& map, const Mat44& T_world_plane,
                           const LabelledDistanceTransform& dt, CostGrid& costs) const;

  Status computeBevCosts(const kitti::MapChunk& map, const Mat44& T_world_plane,
                         const LabelledDistanceTransform& dt, CostGrid& costs) const;

 private:
  float sampleImageCost(const LabelledDistanceTransform& dt, const Vec2& uv,
                        kitti::PolylineType type) const;

  float sampleBevCost(const LabelledDistanceTransform& dt, const Vec3& p_rig,
                      kitti::PolylineType type) const;

  LocalizationParams params_;
  const Projection* projection_ = nullptr;
};

}  // namespace cam_loc::core
