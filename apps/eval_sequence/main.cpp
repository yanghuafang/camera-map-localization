/// CLI: full sequence evaluation with pose/matching metrics and optional per-frame CSV.
#include <cam_loc/kitti/calib_parser.hpp>
#include <cam_loc/kitti/sequence_eval.hpp>
#include <cam_loc/kitti/sequence_eval_runner.hpp>
#include <cam_loc/map/map_loader_util.hpp>
#include <cam_loc/perception/resolve.hpp>
#include <cam_loc/types/status.hpp>

#include <fstream>
#include <iostream>
#include <memory>
#include <string>

namespace {

struct Options {
  std::string kitti_root = ".";
  std::string perception_root;
  std::string map_path;
  std::string georef_path;
  double map_origin_lat = 0.0;
  double map_origin_lon = 0.0;
  bool map_origin_set = false;
  bool map_align_yaw = false;
  std::string perception_mode = "auto";
  std::string output_csv;
  int sequence = 0;
  int max_frames = -1;
  int skip_frames = 0;
  bool use_gt_prior = false;
  bool use_gt_plane = false;
  bool use_cuda = false;
  bool use_global_ego = false;
  double noise_px = 0.0;
  double noise_point_dropout = 0.0;
  double noise_polyline_dropout = 0.0;
  uint32_t noise_seed = 1;
  float cost_flat_threshold = 0.05f;
  float cost_softmax_scale = 0.5f;
  int aggregation_window = 70;
};

cam_loc::perception::PerceptionSource parsePerceptionMode(const std::string& s) {
  if (s == "auto") return cam_loc::perception::PerceptionSource::kAuto;
  if (s == "file") return cam_loc::perception::PerceptionSource::kFile;
  if (s == "oracle") return cam_loc::perception::PerceptionSource::kOracle;
  if (s == "noisy") return cam_loc::perception::PerceptionSource::kNoisy;
  throw std::runtime_error("Unknown --perception-mode: " + s);
}

Options parseArgs(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto need = [&](const char* name) -> std::string {
      if (i + 1 >= argc) {
        throw std::runtime_error(std::string("Missing value for ") + name);
      }
      return argv[++i];
    };
    if (arg == "--kitti-root") {
      opt.kitti_root = need("--kitti-root");
    } else if (arg == "--perception-root") {
      opt.perception_root = need("--perception-root");
    } else if (arg == "--perception-mode") {
      opt.perception_mode = need("--perception-mode");
    } else if (arg == "--map-path") {
      opt.map_path = need("--map-path");
    } else if (arg == "--map-georef") {
      opt.georef_path = need("--map-georef");
    } else if (arg == "--map-origin-lat") {
      opt.map_origin_lat = std::stod(need("--map-origin-lat"));
      opt.map_origin_set = true;
    } else if (arg == "--map-origin-lon") {
      opt.map_origin_lon = std::stod(need("--map-origin-lon"));
      opt.map_origin_set = true;
    } else if (arg == "--map-align-yaw") {
      opt.map_align_yaw = true;
    } else if (arg == "--output-csv") {
      opt.output_csv = need("--output-csv");
    } else if (arg == "--sequence") {
      opt.sequence = std::stoi(need("--sequence"));
    } else if (arg == "--max-frames") {
      opt.max_frames = std::stoi(need("--max-frames"));
    } else if (arg == "--skip-frames") {
      opt.skip_frames = std::stoi(need("--skip-frames"));
    } else if (arg == "--use-gt") {
      opt.use_gt_prior = true;
    } else if (arg == "--use-gt-plane") {
      opt.use_gt_plane = true;
    } else if (arg == "--use-cuda") {
      opt.use_cuda = true;
    } else if (arg == "--use-global-ego") {
      opt.use_global_ego = true;
    } else if (arg == "--noise-px") {
      opt.noise_px = std::stod(need("--noise-px"));
    } else if (arg == "--noise-point-dropout") {
      opt.noise_point_dropout = std::stod(need("--noise-point-dropout"));
    } else if (arg == "--noise-polyline-dropout") {
      opt.noise_polyline_dropout = std::stod(need("--noise-polyline-dropout"));
    } else if (arg == "--noise-seed") {
      opt.noise_seed = static_cast<uint32_t>(std::stoul(need("--noise-seed")));
    } else if (arg == "--cost-flat-threshold") {
      opt.cost_flat_threshold = std::stof(need("--cost-flat-threshold"));
    } else if (arg == "--cost-softmax-scale") {
      opt.cost_softmax_scale = std::stof(need("--cost-softmax-scale"));
    } else if (arg == "--aggregation-window") {
      opt.aggregation_window = std::stoi(need("--aggregation-window"));
    } else if (arg == "--help" || arg == "-h") {
      std::cout
          << "Usage: eval_sequence [options]\n"
          << "  --perception-mode MODE  auto|file|oracle|noisy (default auto)\n"
          << "  --noise-px N            Pixel noise for noisy mode\n"
          << "  --cost-flat-threshold F Map-matching flat gate (default 0.05)\n"
          << "  --cost-softmax-scale F  Sampling covariance scale (default 0.5)\n"
          << "  --aggregation-window N  Temporal window (default 70)\n"
          << "  (see eval_perception_compare --help for map/perception flags)\n";
      std::exit(0);
    }
  }
  return opt;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options opt = parseArgs(argc, argv);
    const std::string seq = cam_loc::formatSequenceId(opt.sequence);

    const std::string poses_path = cam_loc::kitti::resolvePosesPath(opt.kitti_root, opt.sequence);
    const std::string calib_path = cam_loc::kitti::resolveCalibPath(opt.kitti_root, opt.sequence);

    std::vector<cam_loc::kitti::Pose> poses;
    if (cam_loc::kitti::loadPosesFile(poses_path, poses) != cam_loc::Status::kOk) {
      std::cerr << "Failed to load poses: " << poses_path << "\n";
      return 1;
    }

    cam_loc::kitti::Calibration calib;
    if (cam_loc::kitti::parseCalibrationFile(calib_path, calib) != cam_loc::Status::kOk) {
      std::cerr << "Failed to parse calib: " << calib_path << "\n";
      return 1;
    }

    cam_loc::map::MapLoadOptions map_opt;
    map_opt.map_path = opt.map_path;
    map_opt.georef_path = opt.georef_path;
    map_opt.poses = &poses;
    map_opt.align_yaw_to_first_pose = opt.map_align_yaw;
    if (opt.map_origin_set) {
      map_opt.georef.origin_lat_deg = opt.map_origin_lat;
      map_opt.georef.origin_lon_deg = opt.map_origin_lon;
    }

    std::shared_ptr<cam_loc::map::IMapLoader> map;
    if (cam_loc::map::createMapLoader(map_opt, map) != cam_loc::Status::kOk) {
      std::cerr << "Failed to load map\n";
      return 1;
    }

    cam_loc::kitti::SequenceEvalConfig cfg;
    cfg.perception_root = opt.perception_root;
    cfg.perception_source = parsePerceptionMode(opt.perception_mode);
    cfg.sequence = opt.sequence;
    cfg.start_frame = opt.skip_frames;
    cfg.max_frames = opt.max_frames;
    cfg.noise_seed = opt.noise_seed;
    cfg.noise.pixel_std = opt.noise_px;
    cfg.noise.point_dropout = opt.noise_point_dropout;
    cfg.noise.polyline_dropout = opt.noise_polyline_dropout;
    cfg.localization.use_gt_global_prior = opt.use_gt_prior;
    cfg.localization.use_gt_sampling_plane = opt.use_gt_plane;
    cfg.localization.use_global_ego_measurement = opt.use_global_ego;
    cfg.localization.use_cuda = opt.use_cuda;
    cfg.localization.cost_flat_threshold = opt.cost_flat_threshold;
    cfg.localization.cost_softmax_scale = opt.cost_softmax_scale;
    cfg.localization.aggregation.window_size = opt.aggregation_window;

    std::vector<cam_loc::kitti::FrameEvalRecord> records;
    if (cam_loc::kitti::runSequenceEval(poses, calib, map, cfg, records) != cam_loc::Status::kOk) {
      std::cerr << "Sequence eval failed\n";
      return 1;
    }

    const auto summary = cam_loc::kitti::summarizeEval(records);

    std::ofstream csv;
    if (!opt.output_csv.empty()) {
      csv.open(opt.output_csv);
      if (!csv) {
        std::cerr << "Failed to open CSV: " << opt.output_csv << "\n";
        return 1;
      }
      csv << "frame,translation_m,yaw_deg,min_cost,cost_spread,match,flat,synth,offset_m\n";
    }

    for (const auto& r : records) {
      if (csv) {
        csv << r.frame << "," << r.pose_error.translation_m << "," << r.pose_error.yaw_deg << ","
            << r.min_cost << "," << r.cost_spread << "," << (r.sampling_applied ? 1 : 0) << ","
            << (r.cost_map_flat ? 1 : 0) << "," << (r.perception_synthesized ? 1 : 0) << ","
            << r.best_offset_m << "\n";
      }
    }

    const int total =
        opt.max_frames < 0 ? static_cast<int>(poses.size())
                           : std::min(opt.max_frames, static_cast<int>(poses.size()));

    std::cout << "Sequence " << seq << " frames [" << opt.skip_frames << ", " << total << ")\n";
    std::cout << "Perception mode: " << opt.perception_mode << "\n";
    std::cout << "Translation mean (m): " << summary.pose.mean_translation_m << "\n";
    std::cout << "Translation RMSE (m): " << summary.pose.rmse_translation_m << "\n";
    std::cout << "Translation max (m):  " << summary.pose.max_translation_m << "\n";
    std::cout << "Yaw mean (deg):       " << summary.pose.mean_yaw_deg << "\n";
    std::cout << "Yaw RMSE (deg):       " << summary.pose.rmse_yaw_deg << "\n";
    std::cout << "Mean min cost:        " << summary.matching.mean_min_cost << "\n";
    std::cout << "Mean cost spread:     " << summary.matching.mean_cost_spread << "\n";
    std::cout << "Match rate:           " << (100.0 * summary.matching.match_rate) << "%\n";
    std::cout << "Flat cost rate:       " << (100.0 * summary.matching.flat_rate) << "%\n";
    if (!opt.output_csv.empty()) {
      std::cout << "Wrote " << opt.output_csv << "\n";
    }

    return 0;
  } catch (const std::exception& ex) {
    std::cerr << "Error: " << ex.what() << "\n";
    return 1;
  }
}
