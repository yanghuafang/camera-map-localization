#pragma once

/// Perception JSON path layout and tolerant frame loader (missing file → empty polylines).

#include <cam_loc/kitti/types.hpp>
#include <cam_loc/types/status.hpp>

#include <string>

namespace cam_loc::perception {

/// Resolve perception JSON path for a sequence/frame.
std::string perceptionJsonPath(const std::string& perception_root, int sequence, int frame);

/// Load perception; returns empty polylines if file missing (not an error).
Status loadFramePerception(const std::string& perception_root, int sequence, int frame,
                           kitti::FramePerception& out);

}  // namespace cam_loc::perception
