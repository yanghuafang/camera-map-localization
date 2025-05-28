/// CLI: compare oracle vs real vs noisy perception on one KITTI sequence.
#include <cmath>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

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
    if (arg == "--kitti-root") {
      opt.kitti_root = need("--kitti-root");
    } else if (arg == "--perception-root") {
      opt.perception_root = need("--perception-root");
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
    } else if (arg == "--use-gt-plane") {
      opt.use_gt_plane = true;
    } else if (arg == "--use-cuda") {
      opt.use_cuda = true;
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
    } else if (arg == "--help" || arg == "-h") {
      std::cout
          << "Usage: eval_perception_compare [options]\n"
          << "Compare synthesized oracle lanes vs real/noisy perception on one "
             "sequence.\n"
          << "  --kitti-root PATH\n"
          << "  --perception-root PATH   Real perception JSON (optional)\n"
          << "  --map-path PATH          Optional map JSON/OSM (default "
             "corridor)\n"
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

void PrintSummary(const char* label,
                  const cam_loc::kitti::SequenceEvalSummary& s) {
  std::cout << std::fixed << std::setprecision(4);
  std::cout << label << ":\n";
  std::cout << "  pose RMSE (m):         " << s.pose.rmse_translation_m << "\n";
  std::cout << "  lateral RMSE (m):      " << s.pose.rmse_lateral_m << "\n";
  std::cout << "  longitudinal RMSE (m): " << s.pose.rmse_longitudinal_m
            << "\n";
  std::cout << "  longitudinal bias (m): " << s.pose.bias_longitudinal_m
            << "\n";
  std::cout << "  yaw RMSE (deg):        " << s.pose.rmse_yaw_deg << "\n";
  std::cout << "  mean min cost:         " << s.matching.mean_min_cost << "\n";
  std::cout << "  mean cost spread:      " << s.matching.mean_cost_spread
            << "\n";
  std::cout << "  match rate:            " << (100.0 * s.matching.match_rate)
            << "%\n";
  std::cout << "  flat cost rate:        " << (100.0 * s.matching.flat_rate)
            << "%\n";
}

cam_loc::kitti::SequenceEvalConfig BaseConfig(const Options& opt) {
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
    bool args_ok = false;
    const Options opt = ParseArgs(argc, argv, &args_ok);
    if (!args_ok) return 2;
    const std::string seq = cam_loc::FormatSequenceId(opt.sequence);

    std::vector<cam_loc::kitti::Pose> poses;
    if (cam_loc::kitti::LoadPosesFile(
            cam_loc::kitti::ResolvePosesPath(opt.kitti_root, opt.sequence),
            poses) != cam_loc::Status::kOk) {
      std::cerr << "Failed to load poses\n";
      return 1;
    }

    cam_loc::kitti::Calibration calib;
    if (cam_loc::kitti::ParseCalibrationFile(
            cam_loc::kitti::ResolveCalibPath(opt.kitti_root, opt.sequence),
            calib) != cam_loc::Status::kOk) {
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
    if (cam_loc::map::CreateMapLoader(map_opt, map) != cam_loc::Status::kOk) {
      std::cerr << "Failed to load map\n";
      return 1;
    }

    // Oracle: GT-projected map lanes (upper bound).
    auto oracle_cfg = BaseConfig(opt);
    oracle_cfg.perception_source =
        cam_loc::perception::PerceptionSource::kOracle;
    oracle_cfg.localization.use_gt_sampling_plane = opt.use_gt_plane;

    std::vector<cam_loc::kitti::FrameEvalRecord> oracle_records;
    if (cam_loc::kitti::RunSequenceEval(poses, calib, map, oracle_cfg,
                                        oracle_records) !=
        cam_loc::Status::kOk) {
      std::cerr << "Oracle eval failed\n";
      return 1;
    }
    const auto oracle_summary = cam_loc::kitti::SummarizeEval(oracle_records);

    // Real file perception (Semantic KITTI JSON when available).
    auto real_cfg = BaseConfig(opt);
    real_cfg.perception_source = cam_loc::perception::PerceptionSource::kFile;
    real_cfg.localization.use_gt_sampling_plane = false;

    std::vector<cam_loc::kitti::FrameEvalRecord> real_records;
    if (cam_loc::kitti::RunSequenceEval(poses, calib, map, real_cfg,
                                        real_records) != cam_loc::Status::kOk) {
      std::cerr << "Real perception eval failed\n";
      return 1;
    }
    const auto real_summary = cam_loc::kitti::SummarizeEval(real_records);

    // Noisy: real if present else oracle synthesis + noise (KF sampling plane).
    auto noisy_cfg = BaseConfig(opt);
    noisy_cfg.perception_source = cam_loc::perception::PerceptionSource::kNoisy;
    noisy_cfg.noise.pixel_std = opt.noise_px;
    noisy_cfg.noise.point_dropout = opt.noise_point_dropout;
    noisy_cfg.noise.polyline_dropout = opt.noise_polyline_dropout;
    noisy_cfg.localization.use_gt_sampling_plane = false;

    std::vector<cam_loc::kitti::FrameEvalRecord> noisy_records;
    if (cam_loc::kitti::RunSequenceEval(poses, calib, map, noisy_cfg,
                                        noisy_records) !=
        cam_loc::Status::kOk) {
      std::cerr << "Noisy perception eval failed\n";
      return 1;
    }
    const auto noisy_summary = cam_loc::kitti::SummarizeEval(noisy_records);

    std::cout << "Sequence " << seq << " frames [" << opt.skip_frames << ", "
              << opt.max_frames << ")\n\n";
    PrintSummary("Oracle (synthesized GT lanes)", oracle_summary);
    std::cout << "\n";
    PrintSummary("Real (file perception)", real_summary);
    std::cout << "\n";
    PrintSummary("Noisy (file or synth + noise)", noisy_summary);

    std::cout << "\nDelta vs oracle:\n";
    std::cout << "  real  pose RMSE delta (m):  "
              << (real_summary.pose.rmse_translation_m -
                  oracle_summary.pose.rmse_translation_m)
              << "\n";
    std::cout << "  noisy pose RMSE delta (m):  "
              << (noisy_summary.pose.rmse_translation_m -
                  oracle_summary.pose.rmse_translation_m)
              << "\n";
    std::cout << "  real  match rate delta:     "
              << (100.0 * (real_summary.matching.match_rate -
                           oracle_summary.matching.match_rate))
              << " pp\n";
    std::cout << "  noisy match rate delta:     "
              << (100.0 * (noisy_summary.matching.match_rate -
                           oracle_summary.matching.match_rate))
              << " pp\n";

    if (!opt.output_csv.empty()) {
      std::ofstream csv(opt.output_csv);
      if (!csv) {
        std::cerr << "Failed to open CSV: " << opt.output_csv << "\n";
        return 1;
      }
      // The three series share a column group. Naming it once keeps the
      // header and the rows from drifting apart as columns are added.
      const auto series_header = [](const std::string& p) {
        return p + "_te_m," + p + "_lat_m," + p + "_lon_m," + p + "_yaw_deg," +
               p + "_min_cost," + p + "_match";
      };
      csv << "frame," << series_header("oracle") << "," << series_header("real")
          << "," << series_header("noisy") << "\n";

      const auto series_row = [&csv](const cam_loc::kitti::FrameEvalRecord& r) {
        csv << r.pose_error.translation_m << "," << r.pose_error.lateral_m
            << "," << r.pose_error.longitudinal_m << "," << r.pose_error.yaw_deg
            << "," << r.min_cost << "," << (r.sampling_applied ? 1 : 0);
      };

      const size_t n = oracle_records.size();
      for (size_t i = 0; i < n; ++i) {
        csv << oracle_records[i].frame << ",";
        series_row(oracle_records[i]);
        csv << ",";
        series_row(real_records[i]);
        csv << ",";
        series_row(noisy_records[i]);
        csv << "\n";
      }
      std::cout << "\nWrote " << opt.output_csv << "\n";
    }

    return 0;
  } catch (const std::exception& ex) {
    std::cerr << "Error: " << ex.what() << "\n";
    return 1;
  }
}
