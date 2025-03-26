// Project local 3-D map polylines into the image at the given rig pose (oracle
// perception).

#include "cam_loc/perception/synthesize.h"

namespace cam_loc::perception {

kitti::FramePerception SynthesizeFromMap(const kitti::MapChunk& map,
                                         const core::Projection& projection,
                                         const Mat44& T_world_rig, int frame) {
  kitti::FramePerception out;
  out.frame = frame;

  // World → rig → image, keeping the class. Points behind the camera are
  // dropped by WorldToImage, so a polyline crossing the near plane comes back
  // as its visible part.
  for (const auto& mpl : map.polylines) {
    kitti::Polyline2D pl;
    pl.type = mpl.type;
    pl.points.reserve(mpl.points.size());
    for (const auto& p_world : mpl.points) {
      Vec2 uv;
      if (projection.WorldToImage(T_world_rig, p_world, uv) != Status::kOk) {
        continue;
      }
      pl.points.push_back(uv);
    }
    if (pl.points.size() >= 2) out.features.push_back(std::move(pl));
  }
  return out;
}

}  // namespace cam_loc::perception
