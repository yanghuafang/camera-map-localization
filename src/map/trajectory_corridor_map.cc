// Synthetic HD map: lane geometry on the road surface, plus poles and signs
// beside it, all offset from a ground-truth camera path.

#include "cam_loc/map/trajectory_corridor_map.h"

#include <cmath>

#include "cam_loc/core/frames.h"

namespace cam_loc::map {

namespace {

using core::Frames;

/// One sample of the path: where the camera was, and which way it faced.
struct PathSample {
  Vec3 camera;   ///< Camera position, world (cam0) metres.
  Vec3 forward;  ///< Unit heading, projected onto the road plane.
  Vec3 left;     ///< Unit left, perpendicular to forward in the road plane.
  double travel_m = 0.0;  ///< Distance along the path to this sample.
};

/// Resample the path at a fixed spacing, carrying a frame at each sample.
///
/// Poses closer together than a millimetre are skipped rather than normalized:
/// a stationary vehicle has no heading to read off, and dividing by that gap
/// would manufacture one out of noise.
std::vector<PathSample> SamplePath(const std::vector<kitti::Pose>& poses,
                                   double step_m) {
  const Vec3 up = Frames::UpCam0();
  std::vector<PathSample> samples;
  double travel = 0.0;
  double since_sample = step_m;  // emit at the first usable pose

  for (size_t i = 1; i < poses.size(); ++i) {
    const Vec3 prev = poses[i - 1].T_world_cam0.block<3, 1>(0, 3);
    const Vec3 curr = poses[i].T_world_cam0.block<3, 1>(0, 3);
    const Vec3 step = curr - prev;
    const double len = step.norm();
    if (len < 1e-3) continue;

    travel += len;
    since_sample += len;
    if (since_sample < step_m) continue;
    since_sample = 0.0;

    PathSample s;
    s.camera = curr;
    s.forward = step / len;
    // Left is up × forward in a right-handed frame. The pair is degenerate only
    // if the vehicle is climbing vertically, which a road does not.
    s.left = up.cross(s.forward);
    if (s.left.norm() < 1e-6) continue;
    s.left.normalize();
    s.travel_m = travel;
    samples.push_back(s);
  }
  return samples;
}

/// Point offset from a path sample, in vehicle terms.
///
/// @param left_m Metres to the left of the path.
/// @param up_m   Metres above the *road*, which is ground_height_m below the
///               camera the path is made of.
Vec3 OffsetPoint(const PathSample& s, double left_m, double up_m,
                 double ground_height_m) {
  return s.camera + s.left * left_m +
         Frames::UpCam0() * (up_m - ground_height_m);
}

void AppendPolyline(kitti::MapChunk& map, uint64_t& next_id,
                    kitti::PolylineType type, std::vector<Vec3> points) {
  if (points.size() < 2) return;
  kitti::MapPolyline3D pl;
  pl.id = next_id++;
  pl.type = type;
  pl.points = std::move(points);
  map.polylines.push_back(std::move(pl));
}

}  // namespace

Status TrajectoryCorridorMap::BuildFromPoses(
    const std::vector<kitti::Pose>& poses, const CorridorMapOptions& options) {
  if (poses.size() < 2) {
    return Status::kInvalidArgument;
  }

  const std::vector<PathSample> path = SamplePath(poses, options.sample_step_m);
  if (path.size() < 2) {
    return Status::kInvalidArgument;
  }

  map_.polylines.clear();
  uint64_t next_id = 0;

  // Lane geometry: two solid boundaries and a dashed centreline, all on the
  // road surface rather than at camera height. Height matters more than it
  // looks -- lane points level with the camera project onto the horizon, where
  // they carry no perspective and every hypothesis looks alike.
  std::vector<Vec3> left_edge;
  std::vector<Vec3> right_edge;
  std::vector<Vec3> centre;
  left_edge.reserve(path.size());
  right_edge.reserve(path.size());
  centre.reserve(path.size());
  for (const PathSample& s : path) {
    left_edge.push_back(
        OffsetPoint(s, options.half_width_m, 0.0, options.ground_height_m));
    right_edge.push_back(
        OffsetPoint(s, -options.half_width_m, 0.0, options.ground_height_m));
    centre.push_back(OffsetPoint(s, 0.0, 0.0, options.ground_height_m));
  }
  AppendPolyline(map_, next_id, kitti::PolylineType::kLaneSolid,
                 std::move(left_edge));
  AppendPolyline(map_, next_id, kitti::PolylineType::kLaneSolid,
                 std::move(right_edge));
  AppendPolyline(map_, next_id, kitti::PolylineType::kLaneDashed,
                 std::move(centre));

  // Upright landmarks. Each is its own short polyline rather than a point,
  // because the matcher scores polyline vertices and a pole seen at range
  // needs more than one of them to pull on the cost surface.
  double next_pole_m = options.pole_spacing_m;
  double next_sign_m = options.sign_spacing_m;
  bool pole_on_left = true;

  for (const PathSample& s : path) {
    if (options.pole_spacing_m > 0.0 && s.travel_m >= next_pole_m) {
      next_pole_m += options.pole_spacing_m;
      const double side = pole_on_left ? 1.0 : -1.0;
      pole_on_left = !pole_on_left;
      const double lateral = side * options.pole_offset_m;
      // Base, mid and top: three vertices so the pole still has extent after
      // the near-plane clip removes the base at close range.
      AppendPolyline(map_, next_id, kitti::PolylineType::kPole,
                     {OffsetPoint(s, lateral, 0.0, options.ground_height_m),
                      OffsetPoint(s, lateral, 0.5 * options.pole_height_m,
                                  options.ground_height_m),
                      OffsetPoint(s, lateral, options.pole_height_m,
                                  options.ground_height_m)});
    }

    if (options.sign_spacing_m > 0.0 && s.travel_m >= next_sign_m) {
      next_sign_m += options.sign_spacing_m;
      const double half = 0.5 * options.sign_width_m;
      // A horizontal panel edge: signs constrain along-track position the same
      // way poles do, but their lateral extent also pins heading.
      AppendPolyline(
          map_, next_id, kitti::PolylineType::kSign,
          {OffsetPoint(s, -options.sign_offset_m - half, options.sign_height_m,
                       options.ground_height_m),
           OffsetPoint(s, -options.sign_offset_m + half, options.sign_height_m,
                       options.ground_height_m)});
    }
  }

  RebuildSpatialIndex();
  return Status::kOk;
}

}  // namespace cam_loc::map
