/// PolylineType string conversion (perception JSON ↔ internal enum).
#include <cam_loc/kitti/types.hpp>

namespace cam_loc::kitti {

PolylineType polylineTypeFromString(const std::string& s) {
  if (s == "solid" || s == "lane_solid") return PolylineType::kLaneSolid;
  if (s == "dashed" || s == "lane_dashed") return PolylineType::kLaneDashed;
  if (s == "edge" || s == "road_edge") return PolylineType::kRoadEdge;
  if (s == "pole") return PolylineType::kPole;
  if (s == "sign") return PolylineType::kSign;
  return PolylineType::kUnknown;
}

std::string polylineTypeToString(PolylineType t) {
  switch (t) {
    case PolylineType::kLaneSolid:
      return "lane_solid";
    case PolylineType::kLaneDashed:
      return "lane_dashed";
    case PolylineType::kRoadEdge:
      return "road_edge";
    case PolylineType::kPole:
      return "pole";
    case PolylineType::kSign:
      return "sign";
    default:
      return "unknown";
  }
}

Eigen::Matrix3d Calibration::intrinsicCam0() const {
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
  K(0, 0) = P0(0, 0);
  K(1, 1) = P0(1, 1);
  K(0, 2) = P0(0, 2);
  K(1, 2) = P0(1, 2);
  return K;
}

Mat44 Calibration::T_cam0_velo() const {
  // Rectified cam0 from velodyne applies R0_rect after Tr_velo_to_cam (KITTI convention).
  // R0_rect defaults to identity when the calib omits it, so this is a no-op for such files.
  Mat44 rect = Mat44::Identity();
  rect.block<3, 3>(0, 0) = R0_rect;
  return rect * mat34ToMat44(Tr_velo_to_cam0);
}

}  // namespace cam_loc::kitti
