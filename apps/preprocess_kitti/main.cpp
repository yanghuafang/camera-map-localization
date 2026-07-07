/// CLI: batch-convert SemanticKITTI labels to perception JSON (lidar or PNG mode).
#include <cam_loc/kitti/calib_parser.hpp>
#include <cam_loc/semantic_kitti/lidar_project.hpp>
#include <cam_loc/semantic_kitti/preprocess.hpp>
#include <cam_loc/types/status.hpp>

#include <filesystem>
#include <iostream>
#include <string>

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

Options parseArgs(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto need = [&](const char* name) {
      if (i + 1 >= argc) throw std::runtime_error(std::string("Missing value for ") + name);
      return std::string(argv[++i]);
    };
    if (arg == "--labels-root") opt.labels_root = need("--labels-root");
    else if (arg == "--kitti-root") opt.kitti_root = need("--kitti-root");
    else if (arg == "--output-root") opt.output_root = need("--output-root");
    else if (arg == "--mode") opt.mode = need("--mode");
    else if (arg == "--sequence") opt.sequence = std::stoi(need("--sequence"));
    else if (arg == "--start") opt.start_frame = std::stoi(need("--start"));
    else if (arg == "--end") opt.end_frame = std::stoi(need("--end"));
    else if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: preprocess_kitti [options]\n"
                << "  --mode lidar|png       lidar: project SemanticKITTI .bin+.label (default)\n"
                << "                         png: 16-bit label PNG row-scan\n"
                << "  --kitti-root PATH      KITTI root with velodyne/ + labels/ (lidar mode)\n"
                << "  --labels-root PATH     PNG labels dir (png mode)\n"
                << "  --output-root PATH     Output JSON root (default data/perception)\n"
                << "  --sequence N           Sequence id (default 0)\n"
                << "  --start N              Start frame\n"
                << "  --end N                End frame inclusive (-1 = all)\n";
      std::exit(0);
    }
  }
  if (opt.mode == "lidar" && opt.kitti_root.empty()) {
    throw std::runtime_error("--kitti-root is required for lidar mode");
  }
  if (opt.mode == "png" && opt.labels_root.empty()) {
    throw std::runtime_error("--labels-root is required for png mode");
  }
  return opt;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options opt = parseArgs(argc, argv);
    const std::string seq = cam_loc::formatSequenceId(opt.sequence);
    const std::string out_dir = opt.output_root + "/" + seq;
    std::filesystem::create_directories(out_dir);

    cam_loc::kitti::Calibration calib;
    if (opt.mode == "lidar") {
      const std::string calib_path =
          cam_loc::kitti::resolveCalibPath(opt.kitti_root, opt.sequence);
      if (cam_loc::kitti::parseCalibrationFile(calib_path, calib) != cam_loc::Status::kOk) {
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
        st = cam_loc::semantic_kitti::projectFrameFromFiles(opt.kitti_root, opt.sequence, frame,
                                                            calib, popts, perception);
      } else {
        char name[32];
        snprintf(name, sizeof(name), "%06d.label", frame);
        const std::string label_path = opt.labels_root + "/" + seq + "/labels/" + name;
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
        if (cam_loc::semantic_kitti::loadLabelImage16(label_path, popts.image_width,
                                                      popts.image_height, labels) !=
            cam_loc::Status::kOk) {
          std::cerr << "Failed to load " << label_path << "\n";
          return 1;
        }
        st = cam_loc::semantic_kitti::labelsToPerception(labels, popts.image_width,
                                                         popts.image_height, popts, perception);
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
      if (cam_loc::semantic_kitti::writePerceptionJson(out_path, perception) !=
          cam_loc::Status::kOk) {
        std::cerr << "Failed to write " << out_path << "\n";
        return 1;
      }

      std::cout << "Wrote " << out_path << " (lanes=" << perception.lane_lines.size()
                << ", boundaries=" << perception.road_boundaries.size() << ")\n";
      ++frame;
    }
    return 0;
  } catch (const std::exception& ex) {
    std::cerr << "Error: " << ex.what() << "\n";
    return 1;
  }
}
