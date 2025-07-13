/// KITTI calib/poses parsing, path resolution, and egomotion construction.
#include "cam_loc/kitti/calib_parser.h"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace cam_loc::kitti {

namespace {

bool FileExists(const std::string& path) {
  return std::filesystem::is_regular_file(path);
}

Status ParseMat34Row(const std::string& line, Mat34& out) {
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

Status ParseMat33Row(const std::string& line, Eigen::Matrix3d& out) {
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

std::string ResolvePosesPath(const std::string& kitti_root, int sequence) {
  const std::string seq = FormatSequenceId(sequence);
  // Non-const so the returns move rather than copy: a const local is not
  // eligible for the implicit move, and `a` is returned from two places.
  std::string a = kitti_root + "/poses/" + seq + ".txt";
  if (FileExists(a)) return a;
  std::string b = kitti_root + "/dataset/poses/" + seq + ".txt";
  if (FileExists(b)) return b;
  return a;
}

std::string ResolveCalibPath(const std::string& kitti_root, int sequence) {
  const std::string seq = FormatSequenceId(sequence);
  return kitti_root + "/dataset/sequences/" + seq + "/calib.txt";
}

std::string ResolveImagePath(const std::string& kitti_root, int sequence,
                             int frame) {
  const std::string seq = FormatSequenceId(sequence);
  char name[32];
  std::snprintf(name, sizeof(name), "%06d.png", frame);
  return kitti_root + "/dataset/sequences/" + seq + "/image_0/" + name;
}

Status ParseCalibrationFile(const std::string& path, Calibration& out) {
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
      if (ParseMat34Row(values, out.P0) != Status::kOk)
        return Status::kInvalidArgument;
    } else if (key == "P1") {
      if (ParseMat34Row(values, out.P1) != Status::kOk)
        return Status::kInvalidArgument;
    } else if (key == "R0_rect" || key == "R_rect") {
      if (ParseMat33Row(values, out.R0_rect) != Status::kOk)
        return Status::kInvalidArgument;
    } else if (key == "Tr" || key == "Tr_velo_to_cam") {
      if (ParseMat34Row(values, out.Tr_velo_to_cam0) != Status::kOk)
        return Status::kInvalidArgument;
    }
  }

  return Status::kOk;
}

Status LoadPosesFile(const std::string& path, std::vector<Pose>& out_poses) {
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
    if (ParseMat34Row(line, m) != Status::kOk) {
      return Status::kInvalidArgument;
    }
    Pose p;
    p.frame = frame;
    p.timestamp_ns = static_cast<int64_t>(frame) * kNsPerFrame;
    p.T_world_cam0 = Mat34ToMat44(m);
    out_poses.push_back(p);
    ++frame;
  }

  if (out_poses.empty()) {
    return Status::kInvalidArgument;
  }
  return Status::kOk;
}

Status BuildEgomotion(const std::vector<Pose>& poses, int frame,
                      Egomotion& out) {
  if (frame < 0 || frame >= static_cast<int>(poses.size())) {
    return Status::kInvalidArgument;
  }

  out.global = poses[static_cast<size_t>(frame)];
  if (frame == 0) {
    out.T_curr_prev = Mat44::Identity();
  } else {
    // Relative transform consumed by LocalizationKF::Predict each frame after
    // the first.
    out.T_curr_prev =
        RelativeTransform(poses[static_cast<size_t>(frame - 1)].T_world_cam0,
                          out.global.T_world_cam0);
  }

  // Loose prior on global pose: translation ~0.5 m, rotation ~1 deg (diagonal
  // approx).
  out.cov_global = Mat66::Identity();
  out.cov_global.block<3, 3>(0, 0) *= 0.25;    // ~0.5 m
  out.cov_global.block<3, 3>(3, 3) *= 0.0003;  // ~1 deg

  out.cov_relative = Mat66::Identity() * 0.001;
  return Status::kOk;
}

}  // namespace cam_loc::kitti
