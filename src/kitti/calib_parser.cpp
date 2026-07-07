/// KITTI calib/poses parsing, path resolution, and egomotion construction.
#include <cam_loc/kitti/calib_parser.hpp>

#include <fstream>
#include <sstream>

#include <cstdio>
#include <filesystem>

namespace cam_loc::kitti {

namespace {

bool fileExists(const std::string& path) {
  return std::filesystem::is_regular_file(path);
}

Status parseMat34Row(const std::string& line, Mat34& out) {
  std::istringstream iss(line);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 4; ++c) {
      if (!(iss >> out(r, c))) {
        return Status::kInvalidArgument;
      }
    }
  }
  return Status::kOk;
}

Status parseMat33Row(const std::string& line, Eigen::Matrix3d& out) {
  std::istringstream iss(line);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      if (!(iss >> out(r, c))) {
        return Status::kInvalidArgument;
      }
    }
  }
  return Status::kOk;
}

}  // namespace

std::string resolvePosesPath(const std::string& kitti_root, int sequence) {
  const std::string seq = formatSequenceId(sequence);
  const std::string a = kitti_root + "/poses/" + seq + ".txt";
  if (fileExists(a)) return a;
  const std::string b = kitti_root + "/dataset/poses/" + seq + ".txt";
  if (fileExists(b)) return b;
  return a;
}

std::string resolveCalibPath(const std::string& kitti_root, int sequence) {
  const std::string seq = formatSequenceId(sequence);
  return kitti_root + "/dataset/sequences/" + seq + "/calib.txt";
}

std::string resolveImagePath(const std::string& kitti_root, int sequence, int frame) {
  const std::string seq = formatSequenceId(sequence);
  char name[32];
  std::snprintf(name, sizeof(name), "%06d.png", frame);
  return kitti_root + "/dataset/sequences/" + seq + "/image_0/" + name;
}

Status parseCalibrationFile(const std::string& path, Calibration& out) {
  std::ifstream in(path);
  if (!in.is_open()) {
    return Status::kIoError;
  }

  std::string line;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    const auto colon = line.find(':');
    if (colon == std::string::npos) continue;

    const std::string key = line.substr(0, colon);
    const std::string values = line.substr(colon + 1);

    if (key == "P0") {
      if (parseMat34Row(values, out.P0) != Status::kOk) return Status::kInvalidArgument;
    } else if (key == "P1") {
      if (parseMat34Row(values, out.P1) != Status::kOk) return Status::kInvalidArgument;
    } else if (key == "R0_rect" || key == "R_rect") {
      if (parseMat33Row(values, out.R0_rect) != Status::kOk) return Status::kInvalidArgument;
    } else if (key == "Tr" || key == "Tr_velo_to_cam") {
      if (parseMat34Row(values, out.Tr_velo_to_cam0) != Status::kOk) return Status::kInvalidArgument;
    }
  }

  return Status::kOk;
}

Status loadPosesFile(const std::string& path, std::vector<Pose>& out_poses) {
  std::ifstream in(path);
  if (!in.is_open()) {
    return Status::kIoError;
  }

  out_poses.clear();
  std::string line;
  int frame = 0;
  constexpr int64_t kNsPerFrame = 100'000'000;  // 10 Hz

  while (std::getline(in, line)) {
    if (line.empty()) continue;
    Mat34 m;
    if (parseMat34Row(line, m) != Status::kOk) {
      return Status::kInvalidArgument;
    }
    Pose p;
    p.frame = frame;
    p.timestamp_ns = static_cast<int64_t>(frame) * kNsPerFrame;
    p.T_world_cam0 = mat34ToMat44(m);
    out_poses.push_back(p);
    ++frame;
  }

  if (out_poses.empty()) {
    return Status::kInvalidArgument;
  }
  return Status::kOk;
}

Status buildEgomotion(const std::vector<Pose>& poses, int frame, Egomotion& out) {
  if (frame < 0 || frame >= static_cast<int>(poses.size())) {
    return Status::kInvalidArgument;
  }

  out.global = poses[static_cast<size_t>(frame)];
  if (frame == 0) {
    out.T_curr_prev = Mat44::Identity();
  } else {
    // Relative transform consumed by LocalizationKF::predict each frame after the first.
    out.T_curr_prev =
        relativeTransform(poses[static_cast<size_t>(frame - 1)].T_world_cam0, out.global.T_world_cam0);
  }

  // Loose prior on global pose: translation ~0.5 m, rotation ~1 deg (diagonal approx).
  out.cov_global = Mat66::Identity();
  out.cov_global.block<3, 3>(0, 0) *= 0.25;  // ~0.5 m
  out.cov_global.block<3, 3>(3, 3) *= 0.0003;  // ~1 deg

  out.cov_relative = Mat66::Identity() * 0.001;
  return Status::kOk;
}

}  // namespace cam_loc::kitti
