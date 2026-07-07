/// Velodyne + semantic labels → image raster → FramePerception (lidar preprocess path).
#include <cam_loc/semantic_kitti/lidar_project.hpp>

#include <cam_loc/core/projection.hpp>

#include <cmath>
#include <fstream>
#include <filesystem>

namespace cam_loc::semantic_kitti {

namespace {

std::string seqId(int sequence) { return formatSequenceId(sequence); }

std::string frameName(int frame) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%06d", frame);
  return buf;
}

bool dirExists(const std::string& path) {
  return std::filesystem::is_directory(path);
}

}  // namespace

std::string resolveSequenceDir(const std::string& kitti_root, int sequence) {
  const std::string seq = seqId(sequence);
  const std::string a = kitti_root + "/dataset/sequences/" + seq;
  if (dirExists(a)) return a;
  const std::string b = kitti_root + "/sequences/" + seq;
  if (dirExists(b)) return b;
  return a;
}

std::string velodyneScanPath(const std::string& kitti_root, int sequence, int frame) {
  return resolveSequenceDir(kitti_root, sequence) + "/velodyne/" + frameName(frame) + ".bin";
}

std::string semanticLabelPath(const std::string& kitti_root, int sequence, int frame) {
  return resolveSequenceDir(kitti_root, sequence) + "/labels/" + frameName(frame) + ".label";
}

Status loadVelodyneScan(const std::string& path, std::vector<Vec3>& out_xyz) {
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) return Status::kIoError;

  in.seekg(0, std::ios::end);
  const auto nbytes = in.tellg();
  if (nbytes < 0 || nbytes % static_cast<std::streamoff>(sizeof(float) * 4) != 0) {
    return Status::kInvalidArgument;
  }
  const size_t npts = static_cast<size_t>(nbytes) / (sizeof(float) * 4);
  out_xyz.resize(npts);

  in.seekg(0, std::ios::beg);
  for (size_t i = 0; i < npts; ++i) {
    float xyzi[4];
    if (!in.read(reinterpret_cast<char*>(xyzi), sizeof(xyzi))) {
      return Status::kIoError;
    }
    out_xyz[i] = Vec3(static_cast<double>(xyzi[0]), static_cast<double>(xyzi[1]),
                      static_cast<double>(xyzi[2]));
  }
  return Status::kOk;
}

Status loadSemanticPointLabels(const std::string& path, std::vector<uint16_t>& out_semantic) {
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) return Status::kIoError;

  in.seekg(0, std::ios::end);
  const auto nbytes = in.tellg();
  if (nbytes < 0 || nbytes % static_cast<std::streamoff>(sizeof(uint32_t)) != 0) {
    return Status::kInvalidArgument;
  }
  const size_t n = static_cast<size_t>(nbytes) / sizeof(uint32_t);
  out_semantic.resize(n);

  in.seekg(0, std::ios::beg);
  for (size_t i = 0; i < n; ++i) {
    uint32_t raw = 0;
    if (!in.read(reinterpret_cast<char*>(&raw), sizeof(raw))) {
      return Status::kIoError;
    }
    // SemanticKITTI packs each label into 32 bits: low 16 = semantic class, high 16 = instance
    // id. Only the class is needed here, so mask off the instance half.
    out_semantic[i] = static_cast<uint16_t>(raw & 0xFFFFu);
  }
  return Status::kOk;
}

Status projectLidarLabelsToPerception(const kitti::Calibration& calib,
                                      const std::vector<Vec3>& velo_xyz,
                                      const std::vector<uint16_t>& semantic,
                                      const PreprocessOptions& opts,
                                      kitti::FramePerception& out) {
  if (velo_xyz.size() != semantic.size() || velo_xyz.empty()) {
    return Status::kInvalidArgument;
  }

  const Mat44 T_cam_velo = calib.T_cam0_velo();
  const Eigen::Matrix3d K = calib.intrinsicCam0();
  const double fx = K(0, 0);
  const double fy = K(1, 1);
  const double cx = K(0, 2);
  const double cy = K(1, 2);

  std::vector<uint16_t> raster(static_cast<size_t>(opts.image_width * opts.image_height), 0);

  for (size_t i = 0; i < velo_xyz.size(); ++i) {
    const uint16_t sem = semantic[i];
    if (sem != kLaneMarking && sem != kRoad && sem != kSidewalk && sem != kTerrain) {
      continue;
    }
    const Vec3 p_velo = velo_xyz[i];
    const Vec3 p_cam = (T_cam_velo * p_velo.homogeneous()).head<3>();
    if (p_cam.z() <= 0.5) continue;

    const double u = fx * p_cam.x() / p_cam.z() + cx;
    const double v = fy * p_cam.y() / p_cam.z() + cy;
    const int col = static_cast<int>(std::lround(u));
    const int row = static_cast<int>(std::lround(v));
    if (col < 0 || col >= opts.image_width || row < 0 || row >= opts.image_height) continue;

    raster[static_cast<size_t>(row * opts.image_width + col)] = sem;
  }

  // Thicken sparse LiDAR hits so row-scan polyline extraction can form runs.
  constexpr int kDilate = 3;
  const std::vector<uint16_t> src = raster;
  for (int y = 0; y < opts.image_height; ++y) {
    for (int x = 0; x < opts.image_width; ++x) {
      const uint16_t sem = src[static_cast<size_t>(y * opts.image_width + x)];
      if (sem == 0) continue;
      for (int dx = -kDilate; dx <= kDilate; ++dx) {
        const int nx = x + dx;
        if (nx < 0 || nx >= opts.image_width) continue;
        raster[static_cast<size_t>(y * opts.image_width + nx)] = sem;
      }
    }
  }

  return labelsToPerception(raster, opts.image_width, opts.image_height, opts, out);
}

Status projectFrameFromFiles(const std::string& kitti_root, int sequence, int frame,
                             const kitti::Calibration& calib, const PreprocessOptions& opts,
                             kitti::FramePerception& out) {
  std::vector<Vec3> xyz;
  std::vector<uint16_t> semantic;
  const std::string velo_path = velodyneScanPath(kitti_root, sequence, frame);
  const std::string label_path = semanticLabelPath(kitti_root, sequence, frame);

  if (loadVelodyneScan(velo_path, xyz) != Status::kOk) {
    return Status::kIoError;
  }
  if (loadSemanticPointLabels(label_path, semantic) != Status::kOk) {
    return Status::kIoError;
  }
  if (xyz.size() != semantic.size()) {
    return Status::kInvalidArgument;
  }

  PreprocessOptions frame_opts = opts;
  frame_opts.frame = frame;
  return projectLidarLabelsToPerception(calib, xyz, semantic, frame_opts, out);
}

}  // namespace cam_loc::semantic_kitti
