#pragma once

/// KITTI dataset types: calibration, poses, 2-D perception polylines, and 3-D map chunks.
///
/// Used by localization, evaluation, visualization, and SemanticKITTI preprocessing.

#include <cam_loc/types/status.hpp>

#include <string>
#include <vector>

namespace cam_loc::kitti {

/// Semantic classes used when rasterizing LiDAR labels or scanning label PNGs.
enum class PolylineType : uint8_t {
  kLaneSolid,
  kLaneDashed,
  kRoadEdge,
  kPole,
  kSign,
  kUnknown,
};

PolylineType polylineTypeFromString(const std::string& s);
std::string polylineTypeToString(PolylineType t);

/// Camera intrinsics/extrinsics from KITTI calib.txt (cam0 + velodyne).
struct Calibration {
  Mat34 P0{};
  Mat34 P1{};
  Eigen::Matrix3d R0_rect = Eigen::Matrix3d::Identity();
  Mat34 Tr_velo_to_cam0{};

  Eigen::Matrix3d intrinsicCam0() const;
  Mat44 T_cam0_velo() const;
};

/// Ground-truth or odometry pose for one sequence frame (world ← cam0).
struct Pose {
  int frame = 0;
  int64_t timestamp_ns = 0;
  Mat44 T_world_cam0 = Mat44::Identity();
};

/// EKF inputs for one frame: global prior, relative motion, and covariances.
struct Egomotion {
  /// Global pose prior for this frame (e.g. KITTI GT or VO world pose).
  Pose global;
  /// Relative motion from previous frame: T_curr_prev = T_world_prev⁻¹ · T_world_curr.
  /// Used as the EKF prediction input.
  Mat44 T_curr_prev = Mat44::Identity();
  /// Prior covariance on global pose (used at init and for optional global updates).
  Mat66 cov_global = Mat66::Identity();
  Mat66 cov_relative = Mat66::Identity() * 0.01;
};

/// Image-space polyline (lane line or road boundary) from perception JSON.
struct Polyline2D {
  PolylineType type = PolylineType::kUnknown;
  std::vector<Vec2> points;
};

/// Per-frame lane/boundary detections fed into map matching.
struct FramePerception {
  int frame = 0;
  std::vector<Polyline2D> lane_lines;
  std::vector<Polyline2D> road_boundaries;
};

/// Single 3-D map feature (lane, edge, pole, etc.).
struct MapPolyline3D {
  uint64_t id = 0;
  PolylineType type = PolylineType::kUnknown;
  std::vector<Vec3> points;
};

/// Local map excerpt returned by IMapLoader::query around the current pose.
struct MapChunk {
  std::vector<MapPolyline3D> polylines;
};

}  // namespace cam_loc::kitti
