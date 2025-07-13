#ifndef CAM_LOC_PERCEPTION_SYNTHESIZE_H_
#define CAM_LOC_PERCEPTION_SYNTHESIZE_H_

/// Project local map polylines into the camera image when no perception JSON
/// exists.

#include "cam_loc/core/projection.h"
#include "cam_loc/kitti/types.h"
#include "cam_loc/types/status.h"

namespace cam_loc::perception {

/// When no JSON perception exists, project map polylines into the image at @p
/// T_world_rig.
kitti::FramePerception SynthesizeFromMap(const kitti::MapChunk& map,
                                         const core::Projection& projection,
                                         const Mat44& T_world_rig, int frame);

}  // namespace cam_loc::perception

#endif  // CAM_LOC_PERCEPTION_SYNTHESIZE_H_
