#pragma once

/// LiDAR semantic labels projected to image-space perception polylines.
///
/// Reads velodyne .bin + per-point .label files, rasterizes lane/road classes,
/// then delegates polyline extraction to preprocess.hpp.

#include <cam_loc/kitti/types.hpp>
#include <cam_loc/semantic_kitti/preprocess.hpp>
#include <cam_loc/types/status.hpp>

#include <string>
#include <vector>

namespace cam_loc::semantic_kitti {

/// Root may be KITTI odometry (`dataset/sequences/XX`) or SemanticKITTI (`sequences/XX`).
std::string resolveSequenceDir(const std::string& kitti_root, int sequence);

std::string velodyneScanPath(const std::string& kitti_root, int sequence, int frame);
std::string semanticLabelPath(const std::string& kitti_root, int sequence, int frame);

Status loadVelodyneScan(const std::string& path, std::vector<Vec3>& out_xyz);

Status loadSemanticPointLabels(const std::string& path, std::vector<uint16_t>& out_semantic);

/// Project LiDAR semantic labels into a 2-D label raster, then extract polylines.
Status projectLidarLabelsToPerception(const kitti::Calibration& calib,
                                      const std::vector<Vec3>& velo_xyz,
                                      const std::vector<uint16_t>& semantic,
                                      const PreprocessOptions& opts,
                                      kitti::FramePerception& out);

Status projectFrameFromFiles(const std::string& kitti_root, int sequence, int frame,
                             const kitti::Calibration& calib, const PreprocessOptions& opts,
                             kitti::FramePerception& out);

}  // namespace cam_loc::semantic_kitti
