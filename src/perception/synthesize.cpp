// Project local 3-D map polylines into the image at the given rig pose (oracle perception).

#include <cam_loc/perception/synthesize.hpp>

namespace cam_loc::perception {

kitti::FramePerception synthesizeFromMap(const kitti::MapChunk& map,
                                         const core::Projection& projection,
                                         const Mat44& T_world_rig, int frame) {
  kitti::FramePerception out;
  out.frame = frame;

  // World → rig → camera; keep forward points and split lanes vs road edges.
  for (const auto& mpl : map.polylines) {
    kitti::Polyline2D pl;
    pl.type = mpl.type;
    for (const auto& p_world : mpl.points) {
      Vec2 uv;
      if (projection.worldToImage(T_world_rig, p_world, uv) != Status::kOk) continue;
      pl.points.push_back(uv);
    }
    if (pl.points.size() < 2) continue;
    if (pl.type == kitti::PolylineType::kRoadEdge) {
      out.road_boundaries.push_back(std::move(pl));
    } else {
      out.lane_lines.push_back(std::move(pl));
    }
  }
  return out;
}

}  // namespace cam_loc::perception
