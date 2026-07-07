/// CLI: render offline PNG debug panels for one frame (and optional trajectory overlay).
#include <cam_loc/core/localization_engine.hpp>
#include <cam_loc/kitti/calib_parser.hpp>
#include <cam_loc/map/map_loader_util.hpp>
#include <cam_loc/perception/resolve.hpp>
#include <cam_loc/viz/frame_viz.hpp>

#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

namespace {

struct Options {
  std::string kitti_root = ".";
  std::string perception_root;
  std::string map_path;
  std::string georef_path;
  std::string output_dir = "data/viz";
  std::string perception_mode = "auto";
  double map_origin_lat = 0.0;
  double map_origin_lon = 0.0;
  bool map_origin_set = false;
  bool map_align_yaw = false;
  int sequence = 0;
  int frame = 10;
  int max_frames = -1;
  bool use_gt_plane = false;
  bool use_cuda = false;
  bool trajectory = false;
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
    auto need = [&](const char* name) {
      if (i + 1 >= argc) throw std::runtime_error(std::string("Missing value for ") + name);
      return std::string(argv[++i]);
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
    } else if (arg == "--output-dir") {
      opt.output_dir = need("--output-dir");
    } else if (arg == "--sequence") {
      opt.sequence = std::stoi(need("--sequence"));
    } else if (arg == "--frame") {
      opt.frame = std::stoi(need("--frame"));
    } else if (arg == "--max-frames") {
      opt.max_frames = std::stoi(need("--max-frames"));
    } else if (arg == "--use-gt-plane") {
      opt.use_gt_plane = true;
    } else if (arg == "--use-cuda") {
      opt.use_cuda = true;
    } else if (arg == "--trajectory") {
      opt.trajectory = true;
    } else if (arg == "--help" || arg == "-h") {
      std::cout
          << "Usage: viz_frame [options]\n"
          << "  Render map, perception, DT, cost grid, and pose for one frame (or trajectory).\n"
          << "  --kitti-root PATH       KITTI root (poses + calib; images optional)\n"
          << "  --output-dir PATH       Output directory (default data/viz)\n"
          << "  --frame N               Frame to visualize (default 10)\n"
          << "  --max-frames N          Frames to run (-1 = through --frame)\n"
          << "  --perception-mode MODE  auto|file|oracle|noisy\n"
          << "  --perception-root PATH  Perception JSON root\n"
          << "  --map-path PATH         Map JSON/OSM override\n"
          << "  --use-gt-plane          Pose grid at GT\n"
          << "  --use-cuda              GPU pose-grid costs\n"
          << "  --trajectory            Also write trajectory_gt_est.png for processed frames\n"
          << "  (map georef flags match run_sequence)\n";
      std::exit(0);
    }
  }
  return opt;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options opt = parseArgs(argc, argv);

    const std::string poses_path =
        cam_loc::kitti::resolvePosesPath(opt.kitti_root, opt.sequence);
    std::vector<cam_loc::kitti::Pose> poses;
    if (cam_loc::kitti::loadPosesFile(poses_path, poses) != cam_loc::Status::kOk) {
      std::cerr << "Failed to load poses: " << poses_path << "\n";
      return 1;
    }

    cam_loc::kitti::Calibration calib;
    const std::string calib_path =
        cam_loc::kitti::resolveCalibPath(opt.kitti_root, opt.sequence);
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

    std::shared_ptr<cam_loc::map::IMapLoader> map_loader;
    if (cam_loc::map::createMapLoader(map_opt, map_loader) != cam_loc::Status::kOk) {
      std::cerr << "Failed to load map\n";
      return 1;
    }

    cam_loc::LocalizationParams params;
    params.use_gt_sampling_plane = opt.use_gt_plane;
    params.use_cuda = opt.use_cuda;
    cam_loc::core::LocalizationEngine engine(params);
    engine.setMapLoader(map_loader);
    engine.setCalibration(calib);

    cam_loc::core::Projection projection(calib);
    const cam_loc::perception::PerceptionSource pmode = parsePerceptionMode(opt.perception_mode);

    const int last_frame =
        opt.max_frames < 0 ? opt.frame
                           : std::min(opt.max_frames - 1, static_cast<int>(poses.size()) - 1);
    if (opt.frame < 0 || opt.frame >= static_cast<int>(poses.size()) || last_frame < 0) {
      std::cerr << "Invalid frame range\n";
      return 1;
    }

    cam_loc::viz::TrajectoryVizInput traj;
    traj.gt.reserve(static_cast<size_t>(last_frame + 1));
    traj.estimate.reserve(static_cast<size_t>(last_frame + 1));

    for (int f = 0; f <= last_frame; ++f) {
      cam_loc::kitti::Egomotion ego;
      if (cam_loc::kitti::buildEgomotion(poses, f, ego) != cam_loc::Status::kOk) {
        std::cerr << "Egomotion failed at frame " << f << "\n";
        return 1;
      }

      cam_loc::kitti::FramePerception perception;
      cam_loc::perception::PerceptionResolveInfo pinfo;
      const auto& T_gt = poses[static_cast<size_t>(f)].T_world_cam0;
      if (cam_loc::perception::resolvePerception(
              pmode, opt.perception_root, opt.sequence, f, *map_loader,
              params.map_query_radius_m, projection, T_gt, {}, 1, perception,
              pinfo) != cam_loc::Status::kOk &&
          pmode == cam_loc::perception::PerceptionSource::kOracle) {
        std::cerr << "Perception resolve failed at frame " << f << "\n";
        return 1;
      }

      engine.setDebugCapture(f == opt.frame);
      if (engine.processFrame(ego, perception) != cam_loc::Status::kOk) {
        std::cerr << "processFrame failed at frame " << f << "\n";
        return 1;
      }

      if (opt.trajectory) {
        cam_loc::viz::TrajectoryPoint gt_pt;
        gt_pt.x = T_gt(0, 3);
        gt_pt.z = T_gt(2, 3);
        traj.gt.push_back(gt_pt);
        cam_loc::viz::TrajectoryPoint est_pt;
        est_pt.x = engine.result().T_world_rig(0, 3);
        est_pt.z = engine.result().T_world_rig(2, 3);
        traj.estimate.push_back(est_pt);
      }
    }

    if (!engine.debugSnapshot().valid) {
      std::cerr << "Debug snapshot missing for frame " << opt.frame << "\n";
      return 1;
    }

    cam_loc::viz::FrameVizInput viz_in;
    viz_in.frame = opt.frame;
    viz_in.debug = &engine.debugSnapshot();
    viz_in.projection = &projection;
    viz_in.result = &engine.result();
    viz_in.gt_pose = &poses[static_cast<size_t>(opt.frame)];

    const std::string image_path =
        cam_loc::kitti::resolveImagePath(opt.kitti_root, opt.sequence, opt.frame);
    if (std::filesystem::is_regular_file(image_path)) {
      if (cam_loc::viz::loadRgbImage(image_path, viz_in.camera) != cam_loc::Status::kOk) {
        std::cerr << "Warning: failed to load image " << image_path << "\n";
      }
    }

    cam_loc::viz::FrameVizOutput viz_out;
    if (cam_loc::viz::renderFrameViz(viz_in, opt.output_dir, viz_out) != cam_loc::Status::kOk) {
      std::cerr << "renderFrameViz failed\n";
      return 1;
    }

    std::cout << "Wrote visualization for frame " << opt.frame << " to " << opt.output_dir << ":\n";
    for (const auto& path : viz_out.written_files) {
      std::cout << "  " << path << "\n";
    }

    if (opt.trajectory) {
      const std::string traj_path = opt.output_dir + "/trajectory_gt_est.png";
      if (cam_loc::viz::renderTrajectoryViz(traj, traj_path) != cam_loc::Status::kOk) {
        std::cerr << "renderTrajectoryViz failed\n";
        return 1;
      }
      std::cout << "  " << traj_path << "\n";
    }

    return 0;
  } catch (const std::exception& ex) {
    std::cerr << "Error: " << ex.what() << "\n";
    return 1;
  }
}
