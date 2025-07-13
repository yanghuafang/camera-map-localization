#ifndef CAM_LOC_KITTI_TYPES_H_
#define CAM_LOC_KITTI_TYPES_H_

/// KITTI dataset types: calibration, poses, 2-D perception polylines, and 3-D
/// map chunks.
///
/// Used by localization, evaluation, visualization, and SemanticKITTI
/// preprocessing.

#include <string>
#include <vector>

#include "cam_loc/types/status.h"

namespace cam_loc::kitti {

/// Semantic class of a map or perception polyline.
///
/// Only the first three take part in data association: the distance-transform
/// label channel maps them to 1, 2 and 3, and everything else — including
/// kPole and kSign — to the same unlabelled 0, so those two are carried through
/// the I/O layers but do not yet gate a match. See docs/OPEN_ITEMS.md.
enum class PolylineType : uint8_t {
  kLaneSolid,
  kLaneDashed,
  kRoadEdge,
  kPole,
  kSign,
  kUnknown,
};

/// @return kUnknown for an unrecognized name; parsing never fails.
PolylineType PolylineTypeFromString(const std::string& s);

/// @return The canonical spelling, e.g. "lane_solid". PolylineTypeFromString
///         also accepts the short forms ("solid", "dashed", "edge").
std::string PolylineTypeToString(PolylineType t);

/// Whether a class lies on the road surface.
///
/// Inverse-perspective mapping assumes the point is on the road, so only these
/// classes can be scored in the bird's-eye branch; a pole or a sign put through
/// it would land at whatever range that assumption implies rather than where it
/// is. Elevated classes are scored in the image branch alone.
bool IsGroundPlaneType(PolylineType t);

/// Camera intrinsics/extrinsics from KITTI calib.txt (cam0 + velodyne).
struct Calibration {
  /// Rectified cam0 projection, 3x4. The only calibration the pose grid uses.
  Mat34 P0;
  /// Rectified cam1 projection. Parsed but never read: stereo is unimplemented.
  Mat34 P1;
  /// Rectification rotation; identity when calib.txt omits it. Read only by
  /// T_cam0_velo(), so only on the LiDAR preprocessing path.
  Eigen::Matrix3d R0_rect = Eigen::Matrix3d::Identity();
  /// cam0 ← velodyne, 3x4, metres. Also only the LiDAR path.
  Mat34 Tr_velo_to_cam0;

  Eigen::Matrix3d IntrinsicCam0() const;
  Mat44 T_cam0_velo() const;
};

/// Ground-truth or odometry pose for one sequence frame (world ← cam0).
struct Pose {
  int frame = 0;
  /// Synthesized at 10 Hz (`frame * 1e8`); KITTI odometry poses carry none.
  int64_t timestamp_ns = 0;
  /// World ← cam0, metres. World is the cam0 frame of the sequence origin.
  Mat44 T_world_cam0 = Mat44::Identity();
};

/// EKF inputs for one frame: global prior, relative motion, and covariances.
struct Egomotion {
  /// Global pose prior for this frame (e.g. KITTI GT or VO world pose).
  Pose global;
  /// Relative motion from previous frame: T_curr_prev = T_world_prev⁻¹ ·
  /// T_world_curr. Used as the EKF prediction input.
  Mat44 T_curr_prev = Mat44::Identity();
  /// Prior covariance on global pose (used at init and for optional global
  /// updates).
  Mat66 cov_global = Mat66::Identity();
  /// Intended process noise for the predict step. Not read: the engine passes
  /// LocalizationKF::DefaultProcessCov() instead.
  Mat66 cov_relative = Mat66::Identity() * 0.01;
};

/// Image-space polyline (lane line or road boundary) from perception JSON.
struct Polyline2D {
  PolylineType type = PolylineType::kUnknown;
  /// Rectified cam0 image pixels (u, v), via Calibration::P0. 2-D: a pixel
  /// carries no range, so these can only be scored in the image branch.
  std::vector<Vec2> points;
};

/// Everything perception found in one frame, fed into map matching.
///
/// One list rather than one per class: each polyline already carries its own
/// PolylineType, so a new landmark class costs a new enumerator and nothing
/// else. The previous split into `lane_lines` and `road_boundaries` meant every
/// consumer concatenated the two by hand, and a third class would have meant
/// touching all of them.
struct FramePerception {
  int frame = 0;
  std::vector<Polyline2D> features;

  bool empty() const { return features.empty(); }

  /// Count of a single class, for diagnostics and tests.
  int CountOf(PolylineType type) const;
};

/// Single 3-D map feature (lane, edge, pole, etc.).
struct MapPolyline3D {
  /// Source id where one exists (the OSM way id); otherwise an index.
  uint64_t id = 0;
  PolylineType type = PolylineType::kUnknown;
  /// World frame, metres. 3-D, unlike the image-space Polyline2D above.
  std::vector<Vec3> points;
};

/// Local map excerpt returned by IMapLoader::query around the current pose.
struct MapChunk {
  std::vector<MapPolyline3D> polylines;
};

}  // namespace cam_loc::kitti

#endif  // CAM_LOC_KITTI_TYPES_H_
