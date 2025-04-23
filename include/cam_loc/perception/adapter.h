#ifndef CAM_LOC_PERCEPTION_ADAPTER_H_
#define CAM_LOC_PERCEPTION_ADAPTER_H_

/// Perception JSON path layout and tolerant frame loader (missing file → empty
/// polylines).

#include <string>

#include "cam_loc/kitti/types.h"
#include "cam_loc/types/status.h"

namespace cam_loc::perception {

/// Resolve perception JSON path for a sequence/frame.
std::string PerceptionJsonPath(const std::string& perception_root, int sequence,
                               int frame);

/// Load perception; returns empty polylines if file missing (not an error).
Status LoadFramePerception(const std::string& perception_root, int sequence,
                           int frame, kitti::FramePerception& out);

}  // namespace cam_loc::perception

#endif  // CAM_LOC_PERCEPTION_ADAPTER_H_
