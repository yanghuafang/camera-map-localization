/// Velodyne + semantic labels → image raster → FramePerception (lidar
/// preprocess path).
#include "cam_loc/semantic_kitti/lidar_project.h"

#include <cmath>
#include <filesystem>
#include <fstream>

#include "cam_loc/core/projection.h"

namespace cam_loc::semantic_kitti {

namespace {

std::string SeqId(int sequence) { return FormatSequenceId(sequence); }

std::string FrameName(int frame) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%06d", frame);
  return buf;
}

bool DirExists(const std::string& path) {
  return std::filesystem::is_directory(path);
}

}  // namespace

std::string ResolveSequenceDir(const std::string& kitti_root, int sequence) {
  const std::string seq = SeqId(sequence);
  // Non-const so the returns move rather than copy: a const local is not
  // eligible for the implicit move, and `a` is returned from two places.
  std::string a = kitti_root + "/dataset/sequences/" + seq;
  if (DirExists(a)) return a;
  std::string b = kitti_root + "/sequences/" + seq;
  if (DirExists(b)) return b;
  return a;
}

std::string VelodyneScanPath(const std::string& kitti_root, int sequence,
                             int frame) {
  return ResolveSequenceDir(kitti_root, sequence) + "/velodyne/" +
         FrameName(frame) + ".bin";
}

std::string SemanticLabelPath(const std::string& kitti_root, int sequence,
                              int frame) {
  return ResolveSequenceDir(kitti_root, sequence) + "/labels/" +
         FrameName(frame) + ".label";
}

Status LoadVelodyneScan(const std::string& path, std::vector<Vec3>& out_xyz) {
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) return Status::kIoError;

  in.seekg(0, std::ios::end);
  const auto nbytes = in.tellg();
  if (nbytes < 0 ||
      nbytes % static_cast<std::streamoff>(sizeof(float) * 4) != 0) {
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
    out_xyz[i] =
        Vec3(static_cast<double>(xyzi[0]), static_cast<double>(xyzi[1]),
             static_cast<double>(xyzi[2]));
  }
  return Status::kOk;
}

Status LoadSemanticPointLabels(const std::string& path,
                               std::vector<uint16_t>& out_semantic) {
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) return Status::kIoError;

  in.seekg(0, std::ios::end);
  const auto nbytes = in.tellg();
  if (nbytes < 0 ||
      nbytes % static_cast<std::streamoff>(sizeof(uint32_t)) != 0) {
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
    // SemanticKITTI packs each label into 32 bits: low 16 = semantic class,
    // high 16 = instance id. Only the class is needed here, so mask off the
    // instance half.
    out_semantic[i] = static_cast<uint16_t>(raw & 0xFFFFu);
  }
  return Status::kOk;
}

Status ProjectLidarLabelsToPerception(const kitti::Calibration& calib,
                                      const std::vector<Vec3>& velo_xyz,
                                      const std::vector<uint16_t>& semantic,
                                      const PreprocessOptions& opts,
                                      kitti::FramePerception& out) {
  if (velo_xyz.size() != semantic.size() || velo_xyz.empty()) {
    return Status::kInvalidArgument;
  }

  const Mat44 T_cam_velo = calib.T_cam0_velo();
  const Eigen::Matrix3d K = calib.IntrinsicCam0();
  const double fx = K(0, 0);
  const double fy = K(1, 1);
  const double cx = K(0, 2);
  const double cy = K(1, 2);

  std::vector<uint16_t> raster(
      static_cast<size_t>(opts.image_width * opts.image_height), 0);

  for (size_t i = 0; i < velo_xyz.size(); ++i) {
    const uint16_t sem = semantic[i];
    // Only the classes the extractor reads. Poles and signs are included
    // because they are what pins along-track position.
    if (sem != kLaneMarking && sem != kRoad && sem != kSidewalk &&
        sem != kTerrain && sem != kPole && sem != kTrafficSign) {
      continue;
    }
    const Vec3& p_velo = velo_xyz[i];
    const Vec3 p_cam = (T_cam_velo * p_velo.homogeneous()).head<3>();
    if (p_cam.z() <= 0.5) continue;

    const double u = fx * p_cam.x() / p_cam.z() + cx;
    const double v = fy * p_cam.y() / p_cam.z() + cy;
    const int col = static_cast<int>(std::lround(u));
    const int row = static_cast<int>(std::lround(v));
    if (col < 0 || col >= opts.image_width || row < 0 ||
        row >= opts.image_height)
      continue;

    raster[static_cast<size_t>(row * opts.image_width + col)] = sem;
  }

  // A LiDAR scan projects to a sparse scattering of pixels, far too sparse for
  // a run-length scan to find anything. Dilate in both directions: horizontally
  // for the road markings the row scan traces, vertically for the poles and
  // signs the column scan traces.
  constexpr int kDilate = 3;
  const std::vector<uint16_t> src = raster;
  auto at = [&](int x, int y) -> size_t {
    return static_cast<size_t>(y) * static_cast<size_t>(opts.image_width) + x;
  };
  for (int y = 0; y < opts.image_height; ++y) {
    for (int x = 0; x < opts.image_width; ++x) {
      const uint16_t sem = src[at(x, y)];
      if (sem == 0) continue;
      for (int dy = -kDilate; dy <= kDilate; ++dy) {
        const int ny = y + dy;
        if (ny < 0 || ny >= opts.image_height) continue;
        for (int dx = -kDilate; dx <= kDilate; ++dx) {
          const int nx = x + dx;
          if (nx < 0 || nx >= opts.image_width) continue;
          raster[at(nx, ny)] = sem;
        }
      }
    }
  }

  return LabelsToPerception(raster, opts.image_width, opts.image_height, opts,
                            out);
}

Status ProjectFrameFromFiles(const std::string& kitti_root, int sequence,
                             int frame, const kitti::Calibration& calib,
                             const PreprocessOptions& opts,
                             kitti::FramePerception& out) {
  std::vector<Vec3> xyz;
  std::vector<uint16_t> semantic;
  const std::string velo_path = VelodyneScanPath(kitti_root, sequence, frame);
  const std::string label_path = SemanticLabelPath(kitti_root, sequence, frame);

  if (LoadVelodyneScan(velo_path, xyz) != Status::kOk) {
    return Status::kIoError;
  }
  if (LoadSemanticPointLabels(label_path, semantic) != Status::kOk) {
    return Status::kIoError;
  }
  if (xyz.size() != semantic.size()) {
    return Status::kInvalidArgument;
  }

  PreprocessOptions frame_opts = opts;
  frame_opts.frame = frame;
  return ProjectLidarLabelsToPerception(calib, xyz, semantic, frame_opts, out);
}

}  // namespace cam_loc::semantic_kitti
