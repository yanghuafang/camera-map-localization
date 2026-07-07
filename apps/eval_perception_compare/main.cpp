/// CLI: compare oracle vs real vs noisy perception on one KITTI sequence.
#include <cam_loc/kitti/calib_parser.hpp>
#include <cam_loc/kitti/sequence_eval.hpp>
#include <cam_loc/kitti/sequence_eval_runner.hpp>
#include <cam_loc/map/map_loader_util.hpp>
#include <cam_loc/perception/resolve.hpp>
#include <cam_loc/types/status.hpp>

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

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
  std::string output_csv;
  int sequence = 0;
  int max_frames = 100;
  int skip_frames = 10;
  bool use_gt_plane = false;
  bool use_cuda = false;
  double noise_px = 4.0;
  double noise_point_dropout = 0.05;
  double noise_polyline_dropout = 0.1;
  uint32_t noise_seed = 42;
};

Options parseArgs(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto need = [&](const char* name) -> std::string {
      if (i + 1 >= argc) throw std::runtime_error(std::string("Missing value for ") + name);
      return argv[++i];
    };
    if (arg == "--kitti-root") {
      opt.kitti_root = need("--kitti-root");
    } else if (arg == "--perception-root") {
      opt.perception_root = need("--perception-root");
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
    } else if (arg == "--use-gt-plane") {
      opt.use_gt_plane = true;
    } else if (arg == "--use-cuda") {
      opt.use_cuda = true;
    } else if (arg == "--noise-px") {
      opt.noise_px = std::stod(need("--noise-px"));
    } else if (arg == "--noise-point-dropout") {
      opt.noise_point_dropout = std::stod(need("--noise-point-dropout"));
    } else if (arg == "--noise-polyline-dropout") {
      opt.noise_polyline_dropout = std::stod(need("--noise-polyline-dropout"));
    } else if (arg == "--noise-seed") {
      opt.noise_seed = static_cast<uint32_t>(std::stoul(need("--noise-seed")));
    } else if (arg == "--help" || arg == "-h") {
      std::cout
          << "Usage: eval_perception_compare [options]\n"
          << "Compare synthesized oracle lanes vs real/noisy perception on one sequence.\n"
          << "  --kitti-root PATH\n"
          << "  --perception-root PATH   Real perception JSON (optional)\n"
          << "  --map-path PATH          Optional map JSON/OSM (default corridor)\n"
          << "  --max-frames N           Default 100\n"
          << "  --skip-frames N          Default 10 (skip init)\n"
          << "  --use-gt-plane           Oracle pass uses GT sampling plane\n"
          << "  --noise-px N             Noisy pass pixel std (default 4)\n"
          << "  --noise-point-dropout P  Default 0.05\n"
          << "  --noise-polyline-dropout P Default 0.1\n"
          << "  --output-csv PATH        Write per-frame comparison CSV\n"
          << "  --use-cuda\n";
      std::exit(0);
    }
  }
  return opt;
}

void printSummary(const char* label, const cam_loc::kitti::SequenceEvalSummary& s) {
  std::cout << std::fixed << std::setprecision(4);
  std::cout << label << ":\n";
  std::cout << "  pose RMSE (m):     " << s.pose.rmse_translation_m << "\n";
  std::cout << "  yaw RMSE (deg):    " << s.pose.rmse_yaw_deg << "\n";
  std::cout << "  mean min cost:     " << s.matching.mean_min_cost << "\n";
  std::cout << "  mean cost spread:  " << s.matching.mean_cost_spread << "\n";
  std::cout << "  match rate:        " << (100.0 * s.matching.match_rate) << "%\n";
  std::cout << "  flat cost rate:    " << (100.0 * s.matching.flat_rate) << "%\n";
}

cam_loc::kitti::SequenceEvalConfig baseConfig(const Options& opt) {
  cam_loc::kitti::SequenceEvalConfig cfg;
  cfg.kitti_root = opt.kitti_root;
  cfg.perception_root = opt.perception_root;
  cfg.sequence = opt.sequence;
  cfg.start_frame = opt.skip_frames;
  cfg.max_frames = opt.max_frames;
  cfg.noise_seed = opt.noise_seed;
  cfg.localization.use_cuda = opt.use_cuda;
  return cfg;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options opt = parseArgs(argc, argv);
    const std::string seq = cam_loc::formatSequenceId(opt.sequence);

    std::vector<cam_loc::kitti::Pose> poses;
    if (cam_loc::kitti::loadPosesFile(
            cam_loc::kitti::resolvePosesPath(opt.kitti_root, opt.sequence), poses) !=
        cam_loc::Status::kOk) {
      std::cerr << "Failed to load poses\n";
      return 1;
    }

    cam_loc::kitti::Calibration calib;
    if (cam_loc::kitti::parseCalibrationFile(
            cam_loc::kitti::resolveCalibPath(opt.kitti_root, opt.sequence), calib) !=
        cam_loc::Status::kOk) {
      std::cerr << "Failed to parse calib\n";
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

    // Oracle: GT-projected map lanes (upper bound).
    auto oracle_cfg = baseConfig(opt);
    oracle_cfg.perception_source = cam_loc::perception::PerceptionSource::kOracle;
    oracle_cfg.localization.use_gt_sampling_plane = opt.use_gt_plane;

    std::vector<cam_loc::kitti::FrameEvalRecord> oracle_records;
    if (cam_loc::kitti::runSequenceEval(poses, calib, map, oracle_cfg, oracle_records) !=
        cam_loc::Status::kOk) {
      std::cerr << "Oracle eval failed\n";
      return 1;
    }
    const auto oracle_summary = cam_loc::kitti::summarizeEval(oracle_records);

    // Real file perception (Semantic KITTI JSON when available).
    auto real_cfg = baseConfig(opt);
    real_cfg.perception_source = cam_loc::perception::PerceptionSource::kFile;
    real_cfg.localization.use_gt_sampling_plane = false;

    std::vector<cam_loc::kitti::FrameEvalRecord> real_records;
    if (cam_loc::kitti::runSequenceEval(poses, calib, map, real_cfg, real_records) !=
        cam_loc::Status::kOk) {
      std::cerr << "Real perception eval failed\n";
      return 1;
    }
    const auto real_summary = cam_loc::kitti::summarizeEval(real_records);

    // Noisy: real if present else oracle synthesis + noise (KF sampling plane).
    auto noisy_cfg = baseConfig(opt);
    noisy_cfg.perception_source = cam_loc::perception::PerceptionSource::kNoisy;
    noisy_cfg.noise.pixel_std = opt.noise_px;
    noisy_cfg.noise.point_dropout = opt.noise_point_dropout;
    noisy_cfg.noise.polyline_dropout = opt.noise_polyline_dropout;
    noisy_cfg.localization.use_gt_sampling_plane = false;

    std::vector<cam_loc::kitti::FrameEvalRecord> noisy_records;
    if (cam_loc::kitti::runSequenceEval(poses, calib, map, noisy_cfg, noisy_records) !=
        cam_loc::Status::kOk) {
      std::cerr << "Noisy perception eval failed\n";
      return 1;
    }
    const auto noisy_summary = cam_loc::kitti::summarizeEval(noisy_records);

    std::cout << "Sequence " << seq << " frames [" << opt.skip_frames << ", " << opt.max_frames
              << ")\n\n";
    printSummary("Oracle (synthesized GT lanes)", oracle_summary);
    std::cout << "\n";
    printSummary("Real (file perception)", real_summary);
    std::cout << "\n";
    printSummary("Noisy (file or synth + noise)", noisy_summary);

    std::cout << "\nDelta vs oracle:\n";
    std::cout << "  real  pose RMSE delta (m):  "
              << (real_summary.pose.rmse_translation_m - oracle_summary.pose.rmse_translation_m)
              << "\n";
    std::cout << "  noisy pose RMSE delta (m):  "
              << (noisy_summary.pose.rmse_translation_m - oracle_summary.pose.rmse_translation_m)
              << "\n";
    std::cout << "  real  match rate delta:     "
              << (100.0 * (real_summary.matching.match_rate - oracle_summary.matching.match_rate))
              << " pp\n";
    std::cout << "  noisy match rate delta:     "
              << (100.0 * (noisy_summary.matching.match_rate - oracle_summary.matching.match_rate))
              << " pp\n";

    if (!opt.output_csv.empty()) {
      std::ofstream csv(opt.output_csv);
      if (!csv) {
        std::cerr << "Failed to open CSV: " << opt.output_csv << "\n";
        return 1;
      }
      csv << "frame,oracle_te_m,oracle_yaw_deg,oracle_min_cost,oracle_match,"
          << "real_te_m,real_yaw_deg,real_min_cost,real_match,"
          << "noisy_te_m,noisy_yaw_deg,noisy_min_cost,noisy_match\n";
      const size_t n = oracle_records.size();
      for (size_t i = 0; i < n; ++i) {
        const auto& o = oracle_records[i];
        const auto& r = real_records[i];
        const auto& y = noisy_records[i];
        csv << o.frame << "," << o.pose_error.translation_m << "," << o.pose_error.yaw_deg << ","
            << o.min_cost << "," << (o.sampling_applied ? 1 : 0) << "," << r.pose_error.translation_m
            << "," << r.pose_error.yaw_deg << "," << r.min_cost << ","
            << (r.sampling_applied ? 1 : 0) << "," << y.pose_error.translation_m << ","
            << y.pose_error.yaw_deg << "," << y.min_cost << "," << (y.sampling_applied ? 1 : 0)
            << "\n";
      }
      std::cout << "\nWrote " << opt.output_csv << "\n";
    }

    return 0;
  } catch (const std::exception& ex) {
    std::cerr << "Error: " << ex.what() << "\n";
    return 1;
  }
}
