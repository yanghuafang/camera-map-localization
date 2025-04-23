// KITTI-style perception JSON path layout and tolerant load (missing file is
// OK).

#include "cam_loc/perception/adapter.h"

#include <iomanip>
#include <sstream>

#include "cam_loc/kitti/calib_parser.h"

namespace cam_loc::perception {

std::string PerceptionJsonPath(const std::string& perception_root, int sequence,
                               int frame) {
  std::ostringstream oss;
  oss << perception_root << "/" << FormatSequenceId(sequence) << "/"
      << std::setfill('0') << std::setw(6) << frame << ".lanes.json";
  return oss.str();
}

Status LoadFramePerception(const std::string& perception_root, int sequence,
                           int frame, kitti::FramePerception& out) {
  out.frame = frame;
  out.features.clear();

  if (perception_root.empty()) {
    return Status::kOk;
  }

  const std::string path = PerceptionJsonPath(perception_root, sequence, frame);
  const Status st = kitti::LoadPerceptionJson(path, out);
  if (st == Status::kIoError) {
    return Status::kOk;  // missing file is OK in v1
  }
  return st;
}

}  // namespace cam_loc::perception
