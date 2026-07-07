// KITTI-style perception JSON path layout and tolerant load (missing file is OK).

#include <cam_loc/perception/adapter.hpp>

#include <cam_loc/kitti/calib_parser.hpp>

#include <iomanip>
#include <sstream>

namespace cam_loc::perception {

std::string perceptionJsonPath(const std::string& perception_root, int sequence, int frame) {
  std::ostringstream oss;
  oss << perception_root << "/" << formatSequenceId(sequence) << "/"
      << std::setfill('0') << std::setw(6) << frame << ".lanes.json";
  return oss.str();
}

Status loadFramePerception(const std::string& perception_root, int sequence, int frame,
                           kitti::FramePerception& out) {
  out.frame = frame;
  out.lane_lines.clear();
  out.road_boundaries.clear();

  if (perception_root.empty()) {
    return Status::kOk;
  }

  const std::string path = perceptionJsonPath(perception_root, sequence, frame);
  const Status st = kitti::loadPerceptionJson(path, out);
  if (st == Status::kIoError) {
    return Status::kOk;  // missing file is OK in v1
  }
  return st;
}

}  // namespace cam_loc::perception
