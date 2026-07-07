// Synthetic lane map: offset GT camera trajectory left/right to form a driving corridor.

#include <cam_loc/map/trajectory_corridor_map.hpp>

#include <cmath>

namespace cam_loc::map {

namespace {

Vec3 horizontalOffsetPoint(const Vec3& p, const Vec3& tangent, double offset_m) {
  Vec3 up(0, 0, 1);
  Vec3 lateral = tangent.cross(up);
  // tangent parallel to up (near-vertical motion) makes the cross product degenerate; fall back to Y.
  if (lateral.norm() < 1e-6) {
    lateral = Vec3(0, 1, 0);
  }
  lateral.normalize();
  return p + lateral * offset_m;
}

}  // namespace

Status TrajectoryCorridorMap::buildFromPoses(const std::vector<kitti::Pose>& poses,
                                             double half_width_m, double sample_step_m) {
  if (poses.size() < 2) {
    return Status::kInvalidArgument;
  }

  map_.polylines.clear();
  uint64_t id = 0;

  // Sample along motion direction; lateral offset is perpendicular to segment tangent in XY.
  auto buildSide = [&](double sign) {
    kitti::MapPolyline3D pl;
    pl.id = id++;
    pl.type = kitti::PolylineType::kLaneSolid;

    double accum = 0.0;
    pl.points.push_back(poses[0].T_world_cam0.block<3, 1>(0, 3));

    for (size_t i = 1; i < poses.size(); ++i) {
      const Vec3 prev = poses[i - 1].T_world_cam0.block<3, 1>(0, 3);
      const Vec3 curr = poses[i].T_world_cam0.block<3, 1>(0, 3);
      const Vec3 seg = curr - prev;
      const double len = seg.norm();
      if (len < 1e-6) continue;

      accum += len;
      if (accum < sample_step_m) continue;
      accum = 0.0;

      const Vec3 tangent = seg / len;
      pl.points.push_back(horizontalOffsetPoint(curr, tangent, sign * half_width_m));
    }

    if (pl.points.size() >= 2) {
      map_.polylines.push_back(std::move(pl));
    }
  };

  // sign selects left/right; the lambda scales by half_width_m (do not pass the width here).
  buildSide(+1.0);
  buildSide(-1.0);

  // Center dashed line follows the raw trajectory for reference.
  kitti::MapPolyline3D center;
  center.id = id++;
  center.type = kitti::PolylineType::kLaneDashed;
  double accum_center = 0.0;
  center.points.push_back(poses[0].T_world_cam0.block<3, 1>(0, 3));
  for (size_t i = 1; i < poses.size(); ++i) {
    const Vec3 prev = poses[i - 1].T_world_cam0.block<3, 1>(0, 3);
    const Vec3 curr = poses[i].T_world_cam0.block<3, 1>(0, 3);
    accum_center += (curr - prev).norm();
    if (accum_center < sample_step_m) continue;
    accum_center = 0.0;
    center.points.push_back(curr);
  }
  if (center.points.size() < 2) {
    center.points.push_back(poses.back().T_world_cam0.block<3, 1>(0, 3));
  }
  map_.polylines.push_back(std::move(center));

  return Status::kOk;
}

}  // namespace cam_loc::map
