/// CLI: playback localization on a KITTI sequence; prints mean pose error vs
/// GT.
#include <algorithm>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "apps/common/arg_parse.h"
#include "cam_loc/core/localization_engine.h"
#include "cam_loc/kitti/calib_parser.h"
#include "cam_loc/kitti/eval_metrics.h"
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
  int sequence = 0;
  int max_frames = -1;
  bool use_gt = false;
  bool use_gt_plane = false;
  bool use_cuda = false;
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
    } else if (arg == "--sequence") {
      need_number("--sequence", cam_loc::apps::ParseInt, &opt.sequence);
    } else if (arg == "--max-frames") {
      need_number("--max-frames", cam_loc::apps::ParseInt, &opt.max_frames);
    } else if (arg == "--use-gt") {
      opt.use_gt = true;
    } else if (arg == "--use-gt-plane") {
      opt.use_gt_plane = true;
    } else if (arg == "--use-cuda") {
      opt.use_cuda = true;
    } else if (arg == "--help" || arg == "-h") {
      std::cout
          << "Usage: run_sequence [options]\n"
          << "  --kitti-root PATH       KITTI odometry root (contains dataset/ "
             "and poses/)\n"
          << "  --perception-root PATH  Optional JSON polylines root\n"
          << "  --map-path PATH         Map JSON or native OSM (.osm/.xml)\n"
          << "  --map-georef PATH       Georef JSON for OSM (origin "
             "lat/lon/yaw)\n"
          << "  --map-origin-lat DEG    OSM origin latitude (KITTI world "
             "origin)\n"
          << "  --map-origin-lon DEG    OSM origin longitude\n"
          << "  --map-align-yaw         Align OSM yaw to frame-0 GT motion\n"
          << "  --sequence N            Sequence id (default 0)\n"
          << "  --max-frames N          Limit frames (-1 = all)\n"
          << "  --use-gt                Fuse GT pose (debug baseline)\n"
          << "  --use-gt-plane          Pose grid at GT (oracle matching)\n"
          << "  --use-cuda              GPU pose-grid cost evaluation\n";
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
      std::cerr << "Failed to load map";
      if (!opt.map_path.empty()) {
        std::cerr << ": " << opt.map_path;
      }
      std::cerr << "\n";
      return 1;
    }

    cam_loc::LocalizationParams params;
    params.use_gt_global_prior = opt.use_gt;
    params.use_gt_sampling_plane = opt.use_gt_plane;
    params.use_cuda = opt.use_cuda;
    cam_loc::core::LocalizationEngine engine(params);
    engine.set_map_loader(map);
    engine.SetCalibration(calib);
    const cam_loc::core::Projection projection(calib);

    const int nframes =
        opt.max_frames < 0
            ? static_cast<int>(poses.size())
            : std::min(opt.max_frames, static_cast<int>(poses.size()));

    std::vector<cam_loc::kitti::TrajectoryError> errors;
    errors.reserve(static_cast<size_t>(nframes));

    for (int f = 0; f < nframes; ++f) {
      const cam_loc::Mat44& gt = poses[static_cast<size_t>(f)].T_world_cam0;
      cam_loc::kitti::Egomotion ego;
      if (cam_loc::kitti::BuildEgomotion(poses, f, ego) !=
          cam_loc::Status::kOk) {
        std::cerr << "Egomotion failed at frame " << f << "\n";
        return 1;
      }

      // Files when a perception root was given, the ground-truth oracle
      // otherwise. The engine never manufactures perception itself.
      cam_loc::kitti::FramePerception perception;
      cam_loc::perception::PerceptionResolveInfo pinfo;
      cam_loc::perception::ResolvePerception(
          cam_loc::perception::PerceptionSource::kAuto, opt.perception_root,
          opt.sequence, f, *map, params.map_query_radius_m, projection, gt, {},
          1, perception, pinfo);

      if (engine.ProcessFrame(ego, perception) != cam_loc::Status::kOk) {
        std::cerr << "ProcessFrame failed at frame " << f << "\n";
        return 1;
      }

      errors.push_back(
          cam_loc::kitti::PoseError(engine.result().T_world_rig, gt));
    }

    const auto summary = cam_loc::kitti::SummarizeErrors(errors);
    std::cout << "Sequence " << seq << " frames " << nframes << "\n";
    std::cout << "Mean translation error (m): " << summary.mean_translation_m
              << "\n";
    std::cout << "Mean yaw error (deg): " << summary.mean_yaw_deg << "\n";
    std::cout << "Lateral RMSE / bias (m): " << summary.rmse_lateral_m << " / "
              << summary.bias_lateral_m << "\n";
    std::cout << "Longitudinal RMSE / bias (m): " << summary.rmse_longitudinal_m
              << " / " << summary.bias_longitudinal_m << "\n";
    if (!opt.map_path.empty()) {
      std::cout << "Map: " << opt.map_path << "\n";
    } else {
      std::cout << "Map: trajectory corridor from GT poses\n";
    }
    if (!opt.perception_root.empty()) {
      std::cout << "Perception: " << opt.perception_root << "\n";
    } else {
      std::cout << "Perception: oracle (map projected at the ground-truth "
                   "pose)\n";
    }

    return 0;
  } catch (const std::exception& ex) {
    std::cerr << "Error: " << ex.what() << "\n";
    return 1;
  }
}
