/// CLI: batch-convert SemanticKITTI labels to perception JSON (lidar or PNG
/// mode).
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>

#include "apps/common/arg_parse.h"
#include "cam_loc/kitti/calib_parser.h"
#include "cam_loc/semantic_kitti/lidar_project.h"
#include "cam_loc/semantic_kitti/preprocess.h"
#include "cam_loc/types/status.h"

namespace {

struct Options {
  std::string labels_root;
  std::string kitti_root;
  std::string output_root = "data/perception";
  std::string mode = "lidar";
  int sequence = 0;
  int start_frame = 0;
  int end_frame = -1;
};

Options ParseArgs(int argc, char** argv, bool* ok) {
  Options opt;
  *ok = true;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto need = [&](const char* name) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for " << name << "\n";
        *ok = false;
        return std::string();
      }
      return argv[++i];
    };
    // One lambda for every numeric type rather than one per flag: it reports
    // the flag and the text that failed, then leaves *ok false so ParseArgs
    // stops at the end of this iteration.
    auto need_number = [&](const char* name, auto parse, auto* slot) {
      const std::string text = need(name);
      if (*ok && !parse(text, slot)) {
        std::cerr << "Invalid number for " << name << ": " << text << "\n";
        *ok = false;
      }
    };
    if (arg == "--labels-root")
      opt.labels_root = need("--labels-root");
    else if (arg == "--kitti-root")
      opt.kitti_root = need("--kitti-root");
    else if (arg == "--output-root")
      opt.output_root = need("--output-root");
    else if (arg == "--mode")
      opt.mode = need("--mode");
    else if (arg == "--sequence")
      need_number("--sequence", cam_loc::apps::ParseInt, &opt.sequence);
    else if (arg == "--start")
      need_number("--start", cam_loc::apps::ParseInt, &opt.start_frame);
    else if (arg == "--end")
      need_number("--end", cam_loc::apps::ParseInt, &opt.end_frame);
    else if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: preprocess_kitti [options]\n"
                << "  --mode lidar|png       lidar: project SemanticKITTI "
                   ".bin+.label (default)\n"
                << "                         png: 16-bit label PNG row-scan\n"
                << "  --kitti-root PATH      KITTI root with velodyne/ + "
                   "labels/ (lidar mode)\n"
                << "  --labels-root PATH     PNG labels dir (png mode)\n"
                << "  --output-root PATH     Output JSON root (default "
                   "data/perception)\n"
                << "  --sequence N           Sequence id (default 0)\n"
                << "  --start N              Start frame\n"
                << "  --end N                End frame inclusive (-1 = all)\n";
      std::exit(0);
    }
  }
  if (opt.mode == "lidar" && opt.kitti_root.empty()) {
    std::cerr << "--kitti-root is required for lidar mode\n";
    *ok = false;
  }
  if (opt.mode == "png" && opt.labels_root.empty()) {
    std::cerr << "--labels-root is required for png mode\n";
    *ok = false;
  }
  return opt;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    bool args_ok = false;
    const Options opt = ParseArgs(argc, argv, &args_ok);
    if (!args_ok) return 2;
    const std::string seq = cam_loc::FormatSequenceId(opt.sequence);
    const std::string out_dir = opt.output_root + "/" + seq;
    std::filesystem::create_directories(out_dir);

    cam_loc::kitti::Calibration calib;
    if (opt.mode == "lidar") {
      const std::string calib_path =
          cam_loc::kitti::ResolveCalibPath(opt.kitti_root, opt.sequence);
      if (cam_loc::kitti::ParseCalibrationFile(calib_path, calib) !=
          cam_loc::Status::kOk) {
        std::cerr << "Failed to parse calib: " << calib_path << "\n";
        return 1;
      }
    }

    int frame = opt.start_frame;
    while (true) {
      if (opt.end_frame >= 0 && frame > opt.end_frame) break;

      cam_loc::kitti::FramePerception perception;
      cam_loc::Status st = cam_loc::Status::kInvalidArgument;

      if (opt.mode == "lidar") {
        cam_loc::semantic_kitti::PreprocessOptions popts;
        popts.frame = frame;
        st = cam_loc::semantic_kitti::ProjectFrameFromFiles(
            opt.kitti_root, opt.sequence, frame, calib, popts, perception);
      } else {
        char name[32];
        snprintf(name, sizeof(name), "%06d.label", frame);
        const std::string label_path =
            opt.labels_root + "/" + seq + "/labels/" + name;
        if (!std::filesystem::exists(label_path)) {
          if (opt.end_frame < 0 && frame > opt.start_frame) break;
          if (opt.end_frame >= 0) {
            ++frame;
            continue;
          }
          std::cerr << "Missing label: " << label_path << "\n";
          return 1;
        }
        std::vector<uint16_t> labels;
        cam_loc::semantic_kitti::PreprocessOptions popts;
        popts.frame = frame;
        if (cam_loc::semantic_kitti::LoadLabelImage16(
                label_path, popts.image_width, popts.image_height, labels) !=
            cam_loc::Status::kOk) {
          std::cerr << "Failed to load " << label_path << "\n";
          return 1;
        }
        st = cam_loc::semantic_kitti::LabelsToPerception(
            labels, popts.image_width, popts.image_height, popts, perception);
      }

      if (st == cam_loc::Status::kIoError) {
        if (opt.end_frame < 0 && frame > opt.start_frame) break;
        std::cerr << "Missing velodyne/labels for frame " << frame << "\n";
        return 1;
      }
      if (st != cam_loc::Status::kOk) {
        std::cerr << "Failed frame " << frame << "\n";
        return 1;
      }

      char out_name[32];
      snprintf(out_name, sizeof(out_name), "%06d.lanes.json", frame);
      const std::string out_path = out_dir + "/" + out_name;
      if (cam_loc::semantic_kitti::WritePerceptionJson(out_path, perception) !=
          cam_loc::Status::kOk) {
        std::cerr << "Failed to write " << out_path << "\n";
        return 1;
      }

      std::cout << "Wrote " << out_path << " (lanes="
                << perception.CountOf(cam_loc::kitti::PolylineType::kLaneSolid)
                << ", edges="
                << perception.CountOf(cam_loc::kitti::PolylineType::kRoadEdge)
                << ", poles="
                << perception.CountOf(cam_loc::kitti::PolylineType::kPole)
                << ", signs="
                << perception.CountOf(cam_loc::kitti::PolylineType::kSign)
                << ")\n";
      ++frame;
    }
    return 0;
  } catch (const std::exception& ex) {
    std::cerr << "Error: " << ex.what() << "\n";
    return 1;
  }
}
