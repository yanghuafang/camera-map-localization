// Synthetic HD map: lane geometry on the road surface, plus poles and signs
// beside it, all offset from a ground-truth camera path.

#include "cam_loc/map/trajectory_corridor_map.h"

#include <cmath>
#include <utility>

#include "cam_loc/core/frames.h"
#include "cam_loc/types/random.h"

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
/// @param shift  Survey error at this sample, world metres: zero for the world,
///               non-zero for the map that surveyed it.
Vec3 OffsetPoint(const PathSample& s, double left_m, double up_m,
                 double ground_height_m, const Vec3& shift) {
  return s.camera + s.left * left_m +
         Frames::UpCam0() * (up_m - ground_height_m) + shift;
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

/// Survey error at each path sample, as a world-frame displacement.
///
/// An Ornstein-Uhlenbeck walk along arc length, not an independent draw per
/// sample: `rho = exp(-step / L)` carries part of the previous station forward,
/// so every station has standard deviation `sigma` and two stations decorrelate
/// over `L`. Independent draws would average out inside a single map query and
/// put no floor under anything -- the search would still find the true pose,
/// just against a noisier surface.
std::vector<Vec3> SurveyShifts(const std::vector<PathSample>& path,
                               const MapSurveyErrorParams& params,
                               double step_m) {
  std::vector<Vec3> shifts(path.size(), Vec3::Zero());
  if (!params.Enabled() || path.empty()) return shifts;

  const double length = std::max(params.correlation_length_m, 1e-3);
  const double rho = std::exp(-std::max(step_m, 1e-6) / length);
  const double innovation = std::sqrt(std::max(0.0, 1.0 - rho * rho));

  Sampler rng(params.seed);
  // Start on the stationary distribution rather than at zero, so the first
  // stretch of the route is not surveyed better than the rest.
  double lateral = rng.Gaussian();
  double longitudinal = rng.Gaussian();

  for (size_t i = 0; i < path.size(); ++i) {
    if (i > 0) {
      lateral = rho * lateral + innovation * rng.Gaussian();
      longitudinal = rho * longitudinal + innovation * rng.Gaussian();
    }
    shifts[i] = path[i].left * (params.lateral_sigma_m * lateral) +
                path[i].forward * (params.longitudinal_sigma_m * longitudinal);
  }
  return shifts;
}

/// Lay the corridor out along @p path, each sample displaced by @p shifts.
void BuildCorridor(const std::vector<PathSample>& path,
                   const CorridorMapOptions& options,
                   const std::vector<Vec3>& shifts, kitti::MapChunk& out) {
  out.polylines.clear();
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
  for (size_t i = 0; i < path.size(); ++i) {
    const PathSample& s = path[i];
    const Vec3& shift = shifts[i];
    left_edge.push_back(OffsetPoint(s, options.half_width_m, 0.0,
                                    options.ground_height_m, shift));
    right_edge.push_back(OffsetPoint(s, -options.half_width_m, 0.0,
                                     options.ground_height_m, shift));
    centre.push_back(OffsetPoint(s, 0.0, 0.0, options.ground_height_m, shift));
  }
  AppendPolyline(out, next_id, kitti::PolylineType::kLaneSolid,
                 std::move(left_edge));
  AppendPolyline(out, next_id, kitti::PolylineType::kLaneSolid,
                 std::move(right_edge));
  AppendPolyline(out, next_id, kitti::PolylineType::kLaneDashed,
                 std::move(centre));

  // Upright landmarks. Each is its own short polyline rather than a point,
  // because the matcher scores polyline vertices and a pole seen at range
  // needs more than one of them to pull on the cost surface.
  double next_pole_m = options.pole_spacing_m;
  double next_sign_m = options.sign_spacing_m;
  bool pole_on_left = true;

  for (size_t i = 0; i < path.size(); ++i) {
    const PathSample& s = path[i];
    const Vec3& shift = shifts[i];
    if (options.pole_spacing_m > 0.0 && s.travel_m >= next_pole_m) {
      next_pole_m += options.pole_spacing_m;
      const double side = pole_on_left ? 1.0 : -1.0;
      pole_on_left = !pole_on_left;
      const double lateral = side * options.pole_offset_m;
      // Base, mid and top: three vertices so the pole still has extent after
      // the near-plane clip removes the base at close range.
      AppendPolyline(
          out, next_id, kitti::PolylineType::kPole,
          {OffsetPoint(s, lateral, 0.0, options.ground_height_m, shift),
           OffsetPoint(s, lateral, 0.5 * options.pole_height_m,
                       options.ground_height_m, shift),
           OffsetPoint(s, lateral, options.pole_height_m,
                       options.ground_height_m, shift)});
    }

    if (options.sign_spacing_m > 0.0 && s.travel_m >= next_sign_m) {
      next_sign_m += options.sign_spacing_m;
      const double half = 0.5 * options.sign_width_m;
      // A horizontal panel edge: signs constrain along-track position the same
      // way poles do, but their lateral extent also pins heading.
      AppendPolyline(
          out, next_id, kitti::PolylineType::kSign,
          {OffsetPoint(s, -options.sign_offset_m - half, options.sign_height_m,
                       options.ground_height_m, shift),
           OffsetPoint(s, -options.sign_offset_m + half, options.sign_height_m,
                       options.ground_height_m, shift)});
    }
  }
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

  world_.reset();
  if (options.survey_error.Enabled()) {
    // The world: the same landmarks, where they actually are. Perception is cut
    // from this and the matcher scores the survey below, so the gap between the
    // two is what map matching has to close. Built only when there is a gap --
    // otherwise `world()` returns the map and the two are the same object.
    kitti::MapChunk truth;
    BuildCorridor(path, options, std::vector<Vec3>(path.size(), Vec3::Zero()),
                  truth);
    world_ = std::make_unique<PolylineMap>();
    world_->SetMap(std::move(truth));
  }

  BuildCorridor(path, options,
                SurveyShifts(path, options.survey_error, options.sample_step_m),
                map_);
  RebuildSpatialIndex();

  // Publish what the survey was worth, so the filter can widen by it rather
  // than trusting a cost surface that is sharp about the wrong place. The
  // heading term is the displacement's gradient along the route, sigma over the
  // correlation length: a map displaced by the same amount at both ends of a
  // query is not rotated, one displaced by differing amounts is.
  uncertainty_ = MapUncertainty{};
  if (options.survey_error.Enabled()) {
    const double length =
        std::max(options.survey_error.correlation_length_m, 1e-3);
    uncertainty_.lateral_sigma_m = options.survey_error.lateral_sigma_m;
    uncertainty_.longitudinal_sigma_m =
        options.survey_error.longitudinal_sigma_m;
    uncertainty_.yaw_sigma_rad = options.survey_error.lateral_sigma_m / length;
  }
  return Status::kOk;
}

}  // namespace cam_loc::map
