/// CLI: full sequence evaluation with pose/matching metrics and optional
/// per-frame CSV.
#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include "apps/common/arg_parse.h"
#include "cam_loc/kitti/calib_parser.h"
#include "cam_loc/kitti/sequence_eval.h"
#include "cam_loc/kitti/sequence_eval_runner.h"
#include "cam_loc/map/map_loader_util.h"
#include "cam_loc/perception/resolve.h"
#include "cam_loc/types/status.h"

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
  bool enable_bev = false;
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

/// Returns false, leaving @p out untouched, when @p s names no known mode.
bool ParsePerceptionMode(const std::string& s,
                         cam_loc::perception::PerceptionSource* out) {
  if (s == "auto") {
    *out = cam_loc::perception::PerceptionSource::kAuto;
  } else if (s == "file") {
    *out = cam_loc::perception::PerceptionSource::kFile;
  } else if (s == "oracle") {
    *out = cam_loc::perception::PerceptionSource::kOracle;
  } else if (s == "noisy") {
    *out = cam_loc::perception::PerceptionSource::kNoisy;
  } else {
    std::cerr << "Unknown --perception-mode: " << s << "\n";
    return false;
  }
  return true;
}

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
    // One lambda per numeric type rather than one per flag: each reports the
    // flag and the text that failed, then leaves *ok false so ParseArgs stops
    // at the end of this iteration.
    auto need_number = [&](const char* name, auto parse, auto* slot) {
      const std::string text = need(name);
      if (*ok && !parse(text, slot)) {
        std::cerr << "Invalid number for " << name << ": " << text << "\n";
        *ok = false;
      }
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
      need_number("--map-origin-lat", cam_loc::apps::ParseDouble,
                  &opt.map_origin_lat);
      opt.map_origin_set = true;
    } else if (arg == "--map-origin-lon") {
      need_number("--map-origin-lon", cam_loc::apps::ParseDouble,
                  &opt.map_origin_lon);
      opt.map_origin_set = true;
    } else if (arg == "--map-align-yaw") {
      opt.map_align_yaw = true;
    } else if (arg == "--output-csv") {
      opt.output_csv = need("--output-csv");
    } else if (arg == "--sequence") {
      need_number("--sequence", cam_loc::apps::ParseInt, &opt.sequence);
    } else if (arg == "--max-frames") {
      need_number("--max-frames", cam_loc::apps::ParseInt, &opt.max_frames);
    } else if (arg == "--skip-frames") {
      need_number("--skip-frames", cam_loc::apps::ParseInt, &opt.skip_frames);
    } else if (arg == "--use-gt") {
      opt.use_gt_prior = true;
    } else if (arg == "--use-gt-plane") {
      opt.use_gt_plane = true;
    } else if (arg == "--bev") {
      opt.enable_bev = true;
    } else if (arg == "--use-cuda") {
      opt.use_cuda = true;
    } else if (arg == "--use-global-ego") {
      opt.use_global_ego = true;
    } else if (arg == "--noise-px") {
      need_number("--noise-px", cam_loc::apps::ParseDouble, &opt.noise_px);
    } else if (arg == "--noise-point-dropout") {
      need_number("--noise-point-dropout", cam_loc::apps::ParseDouble,
                  &opt.noise_point_dropout);
    } else if (arg == "--noise-polyline-dropout") {
      need_number("--noise-polyline-dropout", cam_loc::apps::ParseDouble,
                  &opt.noise_polyline_dropout);
    } else if (arg == "--noise-seed") {
      need_number("--noise-seed", cam_loc::apps::ParseUint32, &opt.noise_seed);
    } else if (arg == "--cost-flat-threshold") {
      need_number("--cost-flat-threshold", cam_loc::apps::ParseFloat,
                  &opt.cost_flat_threshold);
    } else if (arg == "--cost-softmax-scale") {
      need_number("--cost-softmax-scale", cam_loc::apps::ParseFloat,
                  &opt.cost_softmax_scale);
    } else if (arg == "--aggregation-window") {
      need_number("--aggregation-window", cam_loc::apps::ParseInt,
                  &opt.aggregation_window);
    } else if (arg == "--help" || arg == "-h") {
      std::cout
          << "Usage: eval_sequence [options]\n"
          << "  --perception-mode MODE  auto|file|oracle|noisy (default auto)\n"
          << "  --noise-px N            Pixel noise for noisy mode\n"
          << "  --cost-flat-threshold F Map-matching flat gate (default 0.05)\n"
          << "  --cost-softmax-scale F  Sampling covariance scale (default "
             "0.5)\n"
          << "  --aggregation-window N  Temporal window (default 70)\n"
          << "  --bev                   Also score the bird's-eye branch\n"
          << "                          (off by default; see "
             "LocalizationParams)\n"
          << "  (see eval_perception_compare --help for map/perception "
             "flags)\n";
      std::exit(0);
    }
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

    const std::string poses_path =
        cam_loc::kitti::ResolvePosesPath(opt.kitti_root, opt.sequence);
    const std::string calib_path =
        cam_loc::kitti::ResolveCalibPath(opt.kitti_root, opt.sequence);

    std::vector<cam_loc::kitti::Pose> poses;
    if (cam_loc::kitti::LoadPosesFile(poses_path, poses) !=
        cam_loc::Status::kOk) {
      std::cerr << "Failed to load poses: " << poses_path << "\n";
      return 1;
    }

    cam_loc::kitti::Calibration calib;
    if (cam_loc::kitti::ParseCalibrationFile(calib_path, calib) !=
        cam_loc::Status::kOk) {
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
    if (cam_loc::map::CreateMapLoader(map_opt, map) != cam_loc::Status::kOk) {
      std::cerr << "Failed to load map\n";
      return 1;
    }

    cam_loc::kitti::SequenceEvalConfig cfg;
    cfg.perception_root = opt.perception_root;
    if (!ParsePerceptionMode(opt.perception_mode, &cfg.perception_source)) {
      return 2;
    }
    cfg.sequence = opt.sequence;
    cfg.start_frame = opt.skip_frames;
    cfg.max_frames = opt.max_frames;
    cfg.noise_seed = opt.noise_seed;
    cfg.noise.pixel_std = opt.noise_px;
    cfg.noise.point_dropout = opt.noise_point_dropout;
    cfg.noise.polyline_dropout = opt.noise_polyline_dropout;
    cfg.localization.use_gt_global_prior = opt.use_gt_prior;
    cfg.localization.use_gt_sampling_plane = opt.use_gt_plane;
    cfg.localization.enable_bev = opt.enable_bev;
    cfg.localization.use_global_ego_measurement = opt.use_global_ego;
    cfg.localization.use_cuda = opt.use_cuda;
    cfg.localization.cost_flat_threshold = opt.cost_flat_threshold;
    cfg.localization.cost_softmax_scale = opt.cost_softmax_scale;
    cfg.localization.aggregation.window_size = opt.aggregation_window;

    std::vector<cam_loc::kitti::FrameEvalRecord> records;
    if (cam_loc::kitti::RunSequenceEval(poses, calib, map, cfg, records) !=
        cam_loc::Status::kOk) {
      std::cerr << "Sequence eval failed\n";
      return 1;
    }

    const auto summary = cam_loc::kitti::SummarizeEval(records);

    std::ofstream csv;
    if (!opt.output_csv.empty()) {
      csv.open(opt.output_csv);
      if (!csv) {
        std::cerr << "Failed to open CSV: " << opt.output_csv << "\n";
        return 1;
      }
      csv << "frame,translation_m,lateral_m,longitudinal_m,vertical_m,"
             "yaw_deg,min_cost,cost_spread,match,flat,synth,offset_m\n";
    }

    for (const auto& r : records) {
      if (csv) {
        csv << r.frame << "," << r.pose_error.translation_m << ","
            << r.pose_error.lateral_m << "," << r.pose_error.longitudinal_m
            << "," << r.pose_error.vertical_m << "," << r.pose_error.yaw_deg
            << "," << r.min_cost << "," << r.cost_spread << ","
            << (r.sampling_applied ? 1 : 0) << "," << (r.cost_map_flat ? 1 : 0)
            << "," << (r.perception_synthesized ? 1 : 0) << ","
            << r.best_offset_m << "\n";
      }
    }

    const int total =
        opt.max_frames < 0
            ? static_cast<int>(poses.size())
            : std::min(opt.max_frames, static_cast<int>(poses.size()));

    std::cout << "Sequence " << seq << " frames [" << opt.skip_frames << ", "
              << total << ")\n";
    std::cout << "Perception mode: " << opt.perception_mode << "\n";
    std::cout << "Translation mean (m):      "
              << summary.pose.mean_translation_m << "\n";
    std::cout << "Translation RMSE (m):      "
              << summary.pose.rmse_translation_m << "\n";
    std::cout << "Translation max (m):       " << summary.pose.max_translation_m
              << "\n";
    std::cout << "Lateral RMSE (m):          " << summary.pose.rmse_lateral_m
              << "\n";
    std::cout << "Lateral bias (m):          " << summary.pose.bias_lateral_m
              << "\n";
    std::cout << "Lateral max abs (m):       " << summary.pose.max_abs_lateral_m
              << "\n";
    std::cout << "Longitudinal RMSE (m):     "
              << summary.pose.rmse_longitudinal_m << "\n";
    std::cout << "Longitudinal bias (m):     "
              << summary.pose.bias_longitudinal_m << "\n";
    std::cout << "Longitudinal max abs (m):  "
              << summary.pose.max_abs_longitudinal_m << "\n";
    std::cout << "Vertical RMSE (m):         " << summary.pose.rmse_vertical_m
              << "\n";
    std::cout << "Yaw mean (deg):            " << summary.pose.mean_yaw_deg
              << "\n";
    std::cout << "Yaw RMSE (deg):            " << summary.pose.rmse_yaw_deg
              << "\n";
    std::cout << "Mean min cost:             " << summary.matching.mean_min_cost
              << "\n";
    std::cout << "Mean cost spread:          "
              << summary.matching.mean_cost_spread << "\n";
    std::cout << "Match rate:                "
              << (100.0 * summary.matching.match_rate) << "%\n";
    std::cout << "Flat cost rate:            "
              << (100.0 * summary.matching.flat_rate) << "%\n";
    if (!opt.output_csv.empty()) {
      std::cout << "Wrote " << opt.output_csv << "\n";
    }

    return 0;
  } catch (const std::exception& ex) {
    std::cerr << "Error: " << ex.what() << "\n";
    return 1;
  }
}
