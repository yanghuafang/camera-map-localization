#ifndef CAM_LOC_KITTI_CALIB_PARSER_H_
#define CAM_LOC_KITTI_CALIB_PARSER_H_

/// KITTI dataset I/O: calibration, poses, perception JSON, and path resolution.
///
/// The path helpers probe the two odometry layouts in the wild — poses under
/// `poses/` or under `dataset/poses/` — and return the first that exists.

#include <string>
#include <vector>

#include "cam_loc/kitti/types.h"
#include "cam_loc/types/status.h"

namespace cam_loc::kitti {

/// Parse `calib.txt`. Recognizes `P0`, `P1`, `R0_rect` and `Tr`; a missing
/// `R0_rect` defaults to identity.
Status ParseCalibrationFile(const std::string& path, Calibration& out);

/// Load a KITTI odometry poses file: one row-major 3×4 `[R|t]` per line.
///
/// Timestamps are synthesized at 10 Hz (`frame * 1e8` ns) because the odometry
/// poses carry none.
Status LoadPosesFile(const std::string& path, std::vector<Pose>& out_poses);

/// @return The first of `<root>/poses/XX.txt` and `<root>/dataset/poses/XX.txt`
///         that exists, or the first spelling if neither does — so the caller
///         reports a path a user will recognize.
std::string ResolvePosesPath(const std::string& kitti_root, int sequence);

/// Resolve calib file under `dataset/sequences/XX/calib.txt`.
std::string ResolveCalibPath(const std::string& kitti_root, int sequence);

/// Resolve cam0 grayscale image `dataset/sequences/XX/image_0/NNNNNN.png`.
std::string ResolveImagePath(const std::string& kitti_root, int sequence,
                             int frame);

/// Load one frame of perception polylines. See docs/KITTI_DATA.md for the
/// schema.
Status LoadPerceptionJson(const std::string& path, FramePerception& out);

/// Build the per-frame EKF input from a pose sequence.
///
/// @param frame Index into @p poses; frame 0 gets an identity relative motion.
/// @return `kInvalidArgument` when @p frame is out of range.
Status BuildEgomotion(const std::vector<Pose>& poses, int frame,
                      Egomotion& out);

}  // namespace cam_loc::kitti

#endif  // CAM_LOC_KITTI_CALIB_PARSER_H_
