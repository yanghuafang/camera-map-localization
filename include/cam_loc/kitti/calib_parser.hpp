#pragma once

/// KITTI dataset I/O: calibration, poses, perception JSON, and path resolution.
///
/// Path helpers accept both odometry (`poses/`, `dataset/sequences/`) layouts.

#include <cam_loc/kitti/types.hpp>
#include <cam_loc/types/status.hpp>

#include <string>
#include <vector>

namespace cam_loc::kitti {

Status parseCalibrationFile(const std::string& path, Calibration& out);

/// One 3×4 pose matrix per line (KITTI odometry poses file format).
Status loadPosesFile(const std::string& path, std::vector<Pose>& out_poses);

/// Resolve poses file (supports `poses/XX.txt` and `dataset/poses/XX.txt` layouts).
std::string resolvePosesPath(const std::string& kitti_root, int sequence);

/// Resolve calib file under `dataset/sequences/XX/calib.txt`.
std::string resolveCalibPath(const std::string& kitti_root, int sequence);

/// Resolve cam0 grayscale image `dataset/sequences/XX/image_0/NNNNNN.png`.
std::string resolveImagePath(const std::string& kitti_root, int sequence, int frame);

Status loadPerceptionJson(const std::string& path, FramePerception& out);

/// Build egomotion for frame index from pose sequence.
Status buildEgomotion(const std::vector<Pose>& poses, int frame, Egomotion& out);

}  // namespace cam_loc::kitti
