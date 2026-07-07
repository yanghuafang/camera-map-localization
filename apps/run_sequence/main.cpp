/// CLI: playback localization on a KITTI sequence; prints mean pose error vs GT.
#include <cam_loc/core/localization_engine.hpp>
#include <cam_loc/kitti/calib_parser.hpp>
#include <cam_loc/map/map_loader_util.hpp>
#include <cam_loc/perception/adapter.hpp>
#include <cam_loc/types/status.hpp>

#include <cmath>
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
  int sequence = 0;
  int max_frames = -1;
  bool use_gt = false;
  bool use_gt_plane = false;
  bool use_cuda = false;
};

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
    } else if (arg == "--sequence") {
      opt.sequence = std::stoi(need("--sequence"));
    } else if (arg == "--max-frames") {
      opt.max_frames = std::stoi(need("--max-frames"));
    } else if (arg == "--use-gt") {
      opt.use_gt = true;
    } else if (arg == "--use-gt-plane") {
      opt.use_gt_plane = true;
    } else if (arg == "--use-cuda") {
      opt.use_cuda = true;
    } else if (arg == "--help" || arg == "-h") {
      std::cout
          << "Usage: run_sequence [options]\n"
          << "  --kitti-root PATH       KITTI odometry root (contains dataset/ and poses/)\n"
          << "  --perception-root PATH  Optional JSON polylines root\n"
          << "  --map-path PATH         Map JSON or native OSM (.osm/.xml)\n"
          << "  --map-georef PATH       Georef JSON for OSM (origin lat/lon/yaw)\n"
          << "  --map-origin-lat DEG    OSM origin latitude (KITTI world origin)\n"
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

double translationError(const cam_loc::Mat44& a, const cam_loc::Mat44& b) {
  return (a.block<3, 1>(0, 3) - b.block<3, 1>(0, 3)).norm();
}

double yawErrorDeg(const cam_loc::Mat44& est, const cam_loc::Mat44& gt) {
  const double ye = cam_loc::yawFromRotation(est.block<3, 3>(0, 0));
  const double yg = cam_loc::yawFromRotation(gt.block<3, 3>(0, 0));
  double d = std::abs(ye - yg) * 180.0 / M_PI;
  if (d > 180.0) d = 360.0 - d;
  return d;
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
    engine.setMapLoader(map);
    engine.setCalibration(calib);

    const int nframes =
        opt.max_frames < 0 ? static_cast<int>(poses.size())
                           : std::min(opt.max_frames, static_cast<int>(poses.size()));

    double sum_te = 0.0;
    double sum_yaw = 0.0;

    for (int f = 0; f < nframes; ++f) {
      cam_loc::kitti::Egomotion ego;
      if (cam_loc::kitti::buildEgomotion(poses, f, ego) != cam_loc::Status::kOk) {
        std::cerr << "Egomotion failed at frame " << f << "\n";
        return 1;
      }

      cam_loc::kitti::FramePerception perception;
      cam_loc::perception::loadFramePerception(opt.perception_root, opt.sequence, f,
                                               perception);

      if (engine.processFrame(ego, perception) != cam_loc::Status::kOk) {
        std::cerr << "processFrame failed at frame " << f << "\n";
        return 1;
      }

      const auto& res = engine.result();
      const cam_loc::Mat44& gt = poses[static_cast<size_t>(f)].T_world_cam0;
      sum_te += translationError(res.T_world_rig, gt);
      sum_yaw += yawErrorDeg(res.T_world_rig, gt);
    }

    std::cout << "Sequence " << seq << " frames " << nframes << "\n";
    std::cout << "Mean translation error (m): " << sum_te / nframes << "\n";
    std::cout << "Mean yaw error (deg): " << sum_yaw / nframes << "\n";
    if (!opt.map_path.empty()) {
      std::cout << "Map: " << opt.map_path << "\n";
    } else {
      std::cout << "Map: trajectory corridor from GT poses\n";
    }
    if (!opt.perception_root.empty()) {
      std::cout << "Perception: " << opt.perception_root << "\n";
    } else {
      std::cout << "Perception: synthesized from map projection at GT pose\n";
    }

    return 0;
  } catch (const std::exception& ex) {
    std::cerr << "Error: " << ex.what() << "\n";
    return 1;
  }
}
