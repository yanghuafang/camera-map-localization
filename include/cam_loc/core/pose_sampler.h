#ifndef CAM_LOC_CORE_POSE_SAMPLER_H_
#define CAM_LOC_CORE_POSE_SAMPLER_H_

/// Map-matching pose search: build perception DTs and score a 3-DOF (x, y, yaw)
/// cost grid.

#include "cam_loc/core/cost_grid.h"
#include "cam_loc/core/projection.h"
#include "cam_loc/kitti/types.h"
#include "cam_loc/types/params.h"
#include "cam_loc/types/status.h"

namespace cam_loc::core {

/// Per-pixel Euclidean distance to nearest perception feature, plus type
/// labels. Binary raster convention: 0 = feature stroke, 255 = background.
struct LabelledDistanceTransform {
  std::vector<float> distance;
  std::vector<uint8_t> labels;
  int width = 0;
  int height = 0;
  float max_cost = 5.f;
};

/// Pose-sampling pipeline: rasterize perception → DT → score map points per
/// hypothesis.
///
/// Per frame:
///   1. BuildImageDt / BuildBevDtFromImagePerception: rasterize lanes and
///      edges, run the EDT.
///   2. ComputeImageCosts / ComputeBevCosts: per CostGrid cell, transform map
///      points to rig, project (image) or rasterize (BEV), sample the DT
///      bilinearly.
///   3. A type mismatch between map point and DT label scores max_cost, which
///      is a hard gate rather than a penalty.
class PoseSampler {
 public:
  explicit PoseSampler(const LocalizationParams& params);

  /// @param projection Borrowed, not copied; it must outlive this sampler.
  void set_projection(const Projection& projection);

  /// Rasterize image-space perception and run the EDT over it.
  ///
  /// @return `kInvalidArgument` when no projection has been set.
  Status BuildImageDt(const kitti::FramePerception& perception,
                      LabelledDistanceTransform& out);

  /// Inverse-project image perception onto the ground plane, then rasterize in
  /// BEV.
  ///
  /// @warning Every pixel currently maps to the same BEV cell, so the result is
  ///          a single feature point rather than the perception outline. This
  ///          is the ImageToGroundRig defect in docs/OPEN_ITEMS.md.
  Status BuildBevDtFromImagePerception(const kitti::FramePerception& perception,
                                       LabelledDistanceTransform& out);

  /// Score every grid hypothesis by projecting map points into the image DT.
  ///
  /// @param map           Local map chunk in world coordinates.
  /// @param T_world_plane Anchor pose; hypothesis `i` is
  ///                      `T_world_plane · OffsetToTransform(offset_i)`.
  /// @param costs         Filled with the mean DT cost per hypothesis. A cell
  ///                      whose hypothesis projects no map point at all gets
  ///                      `dt.max_cost * 10` as a sentinel, which is far above
  ///                      any real cost and skews any spread taken over the
  ///                      whole grid.
  Status ComputeImageCosts(const kitti::MapChunk& map,
                           const Mat44& T_world_plane,
                           const LabelledDistanceTransform& dt,
                           CostGrid& costs) const;

  /// As ComputeImageCosts, but scoring in the BEV raster with no pinhole
  /// projection. Carries the BuildBevDtFromImagePerception caveat.
  Status ComputeBevCosts(const kitti::MapChunk& map, const Mat44& T_world_plane,
                         const LabelledDistanceTransform& dt,
                         CostGrid& costs) const;

 private:
  static float SampleImageCost(const LabelledDistanceTransform& dt,
                               const Vec2& uv, kitti::PolylineType type);

  static float SampleBevCost(const LabelledDistanceTransform& dt,
                             const Vec3& p_rig, kitti::PolylineType type);

  LocalizationParams params_;
  const Projection* projection_ = nullptr;
};

}  // namespace cam_loc::core

#endif  // CAM_LOC_CORE_POSE_SAMPLER_H_
