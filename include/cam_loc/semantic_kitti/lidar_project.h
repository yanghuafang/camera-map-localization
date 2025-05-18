#ifndef CAM_LOC_SEMANTIC_KITTI_LIDAR_PROJECT_H_
#define CAM_LOC_SEMANTIC_KITTI_LIDAR_PROJECT_H_

/// LiDAR semantic labels projected to image-space perception polylines.
///
/// Reads velodyne .bin + per-point .label files, rasterizes lane/road classes,
/// then delegates polyline extraction to preprocess.h.

#include <string>
#include <vector>

#include "cam_loc/kitti/types.h"
#include "cam_loc/semantic_kitti/preprocess.h"
#include "cam_loc/types/status.h"

namespace cam_loc::semantic_kitti {

/// Root may be KITTI odometry (`dataset/sequences/XX`) or SemanticKITTI
/// (`sequences/XX`).
std::string ResolveSequenceDir(const std::string& kitti_root, int sequence);

std::string VelodyneScanPath(const std::string& kitti_root, int sequence,
                             int frame);
std::string SemanticLabelPath(const std::string& kitti_root, int sequence,
                              int frame);

Status LoadVelodyneScan(const std::string& path, std::vector<Vec3>& out_xyz);

Status LoadSemanticPointLabels(const std::string& path,
                               std::vector<uint16_t>& out_semantic);

/// Project LiDAR semantic labels into a 2-D label raster, then extract
/// polylines.
Status ProjectLidarLabelsToPerception(const kitti::Calibration& calib,
                                      const std::vector<Vec3>& velo_xyz,
                                      const std::vector<uint16_t>& semantic,
                                      const PreprocessOptions& opts,
                                      kitti::FramePerception& out);

Status ProjectFrameFromFiles(const std::string& kitti_root, int sequence,
                             int frame, const kitti::Calibration& calib,
                             const PreprocessOptions& opts,
                             kitti::FramePerception& out);

}  // namespace cam_loc::semantic_kitti

#endif  // CAM_LOC_SEMANTIC_KITTI_LIDAR_PROJECT_H_
