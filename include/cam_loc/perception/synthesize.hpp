#pragma once

/// Project local map polylines into the camera image when no perception JSON exists.

#include <cam_loc/core/projection.hpp>
#include <cam_loc/kitti/types.hpp>
#include <cam_loc/types/status.hpp>

namespace cam_loc::perception {

/// When no JSON perception exists, project map polylines into the image at @p T_world_rig.
kitti::FramePerception synthesizeFromMap(const kitti::MapChunk& map,
                                         const core::Projection& projection,
                                         const Mat44& T_world_rig, int frame);

}  // namespace cam_loc::perception
