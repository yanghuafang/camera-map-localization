/// Per-frame localization orchestrator: EKF predict → map matching → optional
/// global update.
#include "cam_loc/core/localization_engine.h"

#include "cam_loc/core/cost_modes.h"
#include "cam_loc/core/frames.h"
#include "cam_loc/core/sampling_covariance.h"
#include "cam_loc/map/map_loader.h"

#ifdef CAMLOC_CUDA_ENABLED
#include "cam_loc/cuda/distance_transform.h"
#endif

#include <algorithm>
#include <cmath>
#include <limits>

namespace cam_loc::core {

namespace {

/// Add the map's own uncertainty to a world-frame pose covariance, in place.
void AddMapUncertaintyTo(const map::MapUncertainty& map_error,
                         const Mat44& T_world_rig, Mat66& cov) {
  if (!map_error.Enabled()) return;
  const Eigen::Matrix3d R_world_vehicle =
      T_world_rig.block<3, 3>(0, 0) * Frames::RotCam0Vehicle();
  const Eigen::Matrix3d survey_translation(
      Vec3(map_error.longitudinal_sigma_m * map_error.longitudinal_sigma_m,
           map_error.lateral_sigma_m * map_error.lateral_sigma_m, 0.0)
          .asDiagonal());
  Eigen::Matrix3d survey_rotation = Eigen::Matrix3d::Zero();
  survey_rotation(2, 2) = map_error.yaw_sigma_rad * map_error.yaw_sigma_rad;
  cov.block<3, 3>(0, 0) +=
      R_world_vehicle * survey_translation * R_world_vehicle.transpose();
  cov.block<3, 3>(3, 3) +=
      R_world_vehicle * survey_rotation * R_world_vehicle.transpose();
}

}  // namespace

LocalizationEngine::LocalizationEngine(LocalizationParams params)
    : params_(params),
      pose_sampler_(params_),
      aggregator_(params_.aggregation) {}

void LocalizationEngine::set_map_loader(
    std::shared_ptr<map::IMapLoader> loader) {
  map_loader_ = std::move(loader);
}

void LocalizationEngine::SetCalibration(const kitti::Calibration& calib) {
  projection_.emplace(calib);
  pose_sampler_.set_projection(*projection_);
}

Mat44 LocalizationEngine::SamplingPlanePose(const kitti::Egomotion& ego) const {
  // Anchor for the (x, y, yaw) cost grid: GT pose (oracle) or current KF
  // estimate.
  if (params_.use_gt_sampling_plane || !kf_.initialized()) {
    return ego.global.T_world_cam0;
  }
  const SE3State s = kf_.state();
  Mat44 T = Mat44::Identity();
  T.block<3, 3>(0, 0) = s.rotation;
  T.block<3, 1>(0, 3) = s.translation;
  return T;
}

namespace {

/// Whether @p argmin sits on the boundary of the searched window.
bool TouchesWindowEdge(const CostGrid& grid, const SearchWindow& window,
                       const CostGrid::ArgMinResult& argmin) {
  auto on_edge = [](int index, int centre, int dim, int half) {
    if (half < 0) return false;  // the whole axis was searched
    const int lo = std::max(0, centre - half);
    const int hi = std::min(dim, centre + half + 1) - 1;
    return index <= lo || index >= hi;
  };
  return on_edge(argmin.ix, grid.nx(), grid.DimX(), window.half_x) ||
         on_edge(argmin.iy, grid.ny(), grid.DimY(), window.half_y) ||
         on_edge(argmin.iw, grid.nw(), grid.DimW(), window.half_yaw);
}

}  // namespace

Status LocalizationEngine::RunMapMatching(
    const kitti::Egomotion& ego, const kitti::FramePerception& perception,
    kitti::MapChunk& local_map, kitti::FramePerception& active_perception) {
  // --- Observation generation (map matching) ---
  // Builds a pose measurement for the EKF from perception + local map
  // alignment. Skips kf_.Update only when the surface has no separated
  // minimum at all, or its best fit is bad everywhere; an ambiguous but real
  // minimum widens the covariance instead.
  if (!projection_) {
    return Status::kInvalidArgument;
  }
  if (!map_loader_) {
    return Status::kInvalidArgument;
  }

  const Mat44 T_plane = SamplingPlanePose(ego);
  if (map_loader_->QueryLocalMap(T_plane, params_.map_query_radius_m,
                                 local_map) != Status::kOk) {
    return Status::kInvalidArgument;
  }
  if (local_map.polylines.empty()) {
    return Status::kInvalidArgument;
  }

  // Perception is an input, never manufactured here. Synthesizing it from the
  // map at the sampling plane -- which is the filter's own estimate -- produces
  // a perfectly self-consistent loop: the observation moves with the estimate,
  // so the match reports zero error whatever the estimate is doing. Oracle
  // perception belongs in perception::ResolvePerception, which projects the map
  // at the *ground-truth* pose and says so.
  active_perception = perception;
  if (active_perception.empty()) {
    return Status::kInvalidArgument;
  }

  LabelledDistanceTransform image_dt;
  if (pose_sampler_.BuildImageDt(active_perception, image_dt) != Status::kOk) {
    return Status::kInvalidArgument;
  }

  // Score each (x, y, yaw) hypothesis: project map points, sample perception
  // DT.
  // Score the two branches, then average whichever ran. Renormalizing by the
  // number of branches matters: with a fixed 0.5/0.5 blend, disabling one
  // halved every cost rather than leaving the other's shape alone.
  CostGrid raw_costs(params_.grid);
  raw_costs.Fill(0.f);
  int branches = 0;

  const SearchWindow window = AdaptiveWindow();
  result_.cells_searched = 0;

  if (params_.enable_image) {
    CostGrid image_costs(params_.grid);
    const Status st = pose_sampler_.ComputeImageCosts(
        local_map, T_plane, image_dt, image_costs, window);
    if (st != Status::kOk) return st;
    for (size_t i = 0; i < raw_costs.data().size(); ++i) {
      raw_costs.data()[i] += image_costs.data()[i];
    }
    ++branches;
  }

  LabelledDistanceTransform bev_dt;
  bool has_bev_dt = false;
  if (params_.enable_bev) {
    if (pose_sampler_.BuildBevDtFromImagePerception(active_perception,
                                                    bev_dt) == Status::kOk) {
      has_bev_dt = true;
      CostGrid bev_costs(params_.grid);
      if (pose_sampler_.ComputeBevCosts(local_map, T_plane, bev_dt, bev_costs,
                                        window) == Status::kOk) {
        for (size_t i = 0; i < raw_costs.data().size(); ++i) {
          raw_costs.data()[i] += bev_costs.data()[i];
        }
        ++branches;
      }
    }
  }

  if (branches == 0) return Status::kInvalidArgument;
  if (branches > 1) {
    const float inv = 1.f / static_cast<float>(branches);
    for (float& c : raw_costs.data()) c *= inv;
  }

  CostGrid aggregated = raw_costs;
  bool use_gpu = params_.use_cuda;
#ifdef CAMLOC_CUDA_ENABLED
  use_gpu = use_gpu && cuda::IsAvailable();
#endif
  // Temporal fusion: warp past cost volumes into the current sampling plane.
  aggregator_.Aggregate(aggregated, T_plane, total_travel_m_, use_gpu);
  aggregator_.PushHistory(raw_costs, T_plane, ego.global.frame,
                          total_travel_m_);

  const auto argmin = aggregated.Argmin(use_gpu);

  // The winner sitting on the window's edge is the search saying the answer
  // may lie outside it. That is the one signal available online that the
  // covariance the window came from is too small -- and it has to exist,
  // because a filter that is over-confident sizes the window from its own
  // mistake and would otherwise never look far enough to notice.
  if (!window.IsWholeGrid() && TouchesWindowEdge(aggregated, window, argmin)) {
    full_extent_frames_ = params_.adaptive_extent.reexpand_frames;
  }

  result_.aggregate_min_cost = argmin.cost;
  result_.best_sample_xyyaw = aggregated.RefinedOffset(argmin);
  result_.best_offset_norm_m = result_.best_sample_xyyaw.head<2>().norm();

  if (debug_capture_) {
    debug_.valid = true;
    debug_.T_world_plane = T_plane;
    debug_.local_map = local_map;
    debug_.perception = active_perception;
    debug_.image_dt = image_dt;
    debug_.bev_dt = bev_dt;
    debug_.has_bev_dt = has_bev_dt;
    debug_.aggregated_costs = aggregated;
    debug_.argmin = argmin;
  }

  float max_c = argmin.cost;
  float min_c = argmin.cost;
  for (float c : aggregated.data()) {
    max_c = std::max(max_c, c);
    min_c = std::min(min_c, c);
  }
  result_.cost_map_spread = max_c - min_c;

  // How isolated the winner is, as opposed to how sharp it is: a second basin
  // a few centimetres of cost away is a competing pose, not a wider one.
  const auto modes = FindCostModes(aggregated);
  result_.num_cost_modes = static_cast<int>(modes.size());
  result_.second_mode_xyyaw = Vec3::Zero();
  result_.mode_margin = result_.cost_map_spread;
  if (modes.size() >= 2) {
    result_.mode_margin = modes[1].cost - modes[0].cost;
    result_.second_mode_xyyaw = modes[1].offset;
  }

  // No separated minimum means no hypothesis, for either of two reasons: every
  // pose fits alike and the argmin is whichever cell rounding favoured, or the
  // cost falls away to an edge and the pose that would win is outside the
  // extent searched. A merely shallow surface still has a minimum, and is
  // handled by widening the covariance rather than by dropping the frame.
  result_.cost_map_flat = modes.empty();
  result_.match_cost_too_high = argmin.cost > params_.max_match_cost;
  result_.sampling_measurement_applied = false;

  {
    // Report what the window actually cost, so the saving is measured rather
    // than inferred from the covariance that produced it.
    auto span = [](int centre, int dim, int half) {
      if (half < 0) return dim;
      return std::min(dim, centre + half + 1) - std::max(0, centre - half);
    };
    result_.cells_searched =
        span(aggregated.nx(), aggregated.DimX(), window.half_x) *
        span(aggregated.ny(), aggregated.DimY(), window.half_y) *
        span(aggregated.nw(), aggregated.DimW(), window.half_yaw);
  }

  // Two ways a frame has nothing to say: no minimum at all, or a minimum that
  // fits badly everywhere. Anything between the two is a measurement of some
  // quality, and its quality is expressed in the covariance.
  if (result_.cost_map_flat || result_.match_cost_too_high) {
    // A gated frame is the filter saying it cannot see where it is. Shrinking
    // the search on a covariance that has not been corrected since is how a
    // localizer loses lock and then cannot find its way back.
    full_extent_frames_ = params_.adaptive_extent.reexpand_frames;
    return Status::kOk;
  }

  result_.sampling_measurement_applied = true;
  if (full_extent_frames_ > 0) --full_extent_frames_;
  return ApplySamplingMeasurement(T_plane, aggregated, argmin, modes);
}

SearchWindow LocalizationEngine::AdaptiveWindow() const {
  const auto& adaptive = params_.adaptive_extent;
  if (!adaptive.enabled || !kf_.initialized() || full_extent_frames_ > 0) {
    return SearchWindow{};
  }

  // The grid is anchored on the filter estimate, so the offset that reaches
  // truth is distributed as the filter's covariance -- resolved onto the
  // vehicle axes the grid is indexed by.
  const Mat66 cov = kf_.covariance();
  const Eigen::Matrix3d R_world_vehicle =
      kf_.state().rotation * Frames::RotCam0Vehicle();
  const Eigen::Matrix3d translation =
      R_world_vehicle.transpose() * cov.block<3, 3>(0, 0) * R_world_vehicle;
  const Eigen::Matrix3d rotation =
      R_world_vehicle.transpose() * cov.block<3, 3>(3, 3) * R_world_vehicle;

  const auto half_cells = [&adaptive](double variance, double step, int cap) {
    const double reach =
        adaptive.sigma_multiple * std::sqrt(std::max(variance, 0.0));
    const int cells = static_cast<int>(std::ceil(reach / step));
    return std::clamp(cells, adaptive.min_half_cells, cap);
  };

  const double step_yaw_rad = params_.grid.step_yaw_deg * M_PI / 180.0;
  SearchWindow window;
  window.half_x = half_cells(translation(0, 0), params_.grid.step_x_m,
                             (params_.grid.num_x - 1) / 2);
  window.half_y = half_cells(translation(1, 1), params_.grid.step_y_m,
                             (params_.grid.num_y - 1) / 2);
  window.half_yaw =
      half_cells(rotation(2, 2), step_yaw_rad, (params_.grid.num_yaw - 1) / 2);
  return window;
}

Status LocalizationEngine::ApplySamplingMeasurement(
    const Mat44& T_world_plane, const CostGrid& aggregated,
    const CostGrid::ArgMinResult& argmin, const std::vector<CostMode>& modes) {
  // Map-matching observation: best grid cell → full SE(3) pose in world frame.
  const Vec3 offset = result_.best_sample_xyyaw;
  const Mat44 T_sample =
      T_world_plane *
      Frames::OffsetToCam0Transform(offset.x(), offset.y(), offset.z());

  SE3State meas;
  meas.translation = T_sample.block<3, 1>(0, 3);
  meas.rotation = T_sample.block<3, 3>(0, 0);

  const auto conf = SamplingCovariance::Compute(aggregated, argmin,
                                                params_.cost_softmax_scale);

  // The aggregate is built from frames the previous updates already used, and
  // consecutive aggregates share all but one of them. The filter adds each
  // measurement as if it were independent, so widen the covariance by the
  // number of times the same evidence reaches it.
  const double reuse = aggregator_.effective_frames();

  // Vehicle error basis: [forward, left, up, roll, pitch, yaw]. The grid
  // measures three of the six -- forward, left and yaw -- and the basin
  // covariance couples them, so it goes in whole, cross terms included. Taking
  // only its diagonal assumes the uncertainty lines up with the axes, and the
  // direction lane geometry fails to constrain runs along the road, not along
  // whichever axis the grid was indexed by.
  //
  // Where the volume constrains a direction poorly, SamplingCovariance has
  // already pinned that eigenvalue at the extent searched. The update along it
  // is then negligible while the well-constrained directions still correct the
  // filter -- a frame that fixes heading and lateral offset but cannot place
  // the vehicle along the road should do the two it can.
  constexpr int kMeasured[3] = {0, 1, 5};              // forward, left, yaw
  constexpr double kUnmeasuredTranslationVar = 100.0;  // m²
  constexpr double kUnmeasuredRotationVar = 1.0;       // rad²

  Mat66 cov_vehicle = Mat66::Zero();
  cov_vehicle(2, 2) = kUnmeasuredTranslationVar;  // height
  cov_vehicle(3, 3) = kUnmeasuredRotationVar;     // roll
  cov_vehicle(4, 4) = kUnmeasuredRotationVar;     // pitch

  // A runner-up the frame cannot rule out widens the measurement along the
  // direction the two candidates disagree on. Same cost noise scale as the
  // basin covariance above, so one estimate of the residual drives both.
  //
  // Modes are interior cells, so an argmin on the border is not among them.
  // Pairing it with an unrelated interior runner-up would describe a
  // disagreement the measurement does not rest on; that case is already carried
  // by the softmax fallback in SamplingCovariance, which is wide.
  const bool argmin_leads_the_modes =
      !modes.empty() && modes[0].ix == argmin.ix && modes[0].iy == argmin.iy &&
      modes[0].iw == argmin.iw;
  const Eigen::Matrix3d ambiguity =
      argmin_leads_the_modes
          ? ModeAmbiguityCovariance(modes, std::max<double>(argmin.cost, 1e-3))
          : Eigen::Matrix3d::Zero();

  // Fallback variances for a frame whose basin shape could not be measured.
  const Eigen::Matrix3d unshaped_basin(Vec3(0.25, 0.25, 0.01).asDiagonal());
  const Eigen::Matrix3d basin =
      reuse * (conf.valid ? conf.covariance : unshaped_basin) + ambiguity;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      cov_vehicle(kMeasured[r], kMeasured[c]) = basin(r, c);
    }
  }

  // Rotate the whole 6x6 into the world frame the error state uses; both
  // halves take the same rotation, which is what carries the cross terms
  // across with them.
  const Eigen::Matrix3d R_world_vehicle =
      meas.rotation * Frames::RotCam0Vehicle();
  Mat66 basis = Mat66::Zero();
  basis.block<3, 3>(0, 0) = R_world_vehicle;
  basis.block<3, 3>(3, 3) = R_world_vehicle;
  const Mat66 meas_cov = basis * cov_vehicle * basis.transpose();

  kf_.Update(meas, meas_cov);
  return Status::kOk;
}

void LocalizationEngine::ApplyGlobalMeasurement(const kitti::Egomotion& ego) {
  if (!params_.use_global_ego_measurement) return;

  SE3State meas;
  meas.translation = ego.global.T_world_cam0.block<3, 1>(0, 3);
  meas.rotation = ego.global.T_world_cam0.block<3, 3>(0, 0);
  kf_.Update(meas, ego.cov_global);
}

void LocalizationEngine::WriteResult(const kitti::Egomotion& ego) {
  result_.frame = ego.global.frame;
  result_.timestamp_ns = ego.global.timestamp_ns;
  result_.valid = kf_.initialized();

  const SE3State s = kf_.state();
  result_.T_world_rig = Mat44::Identity();
  result_.T_world_rig.block<3, 3>(0, 0) = s.rotation;
  result_.T_world_rig.block<3, 1>(0, 3) = s.translation;
  // The filter estimates the pose *in the map's frame*. The map's frame is
  // displaced from the world's by the survey error, and the two errors are
  // independent, so the world-frame uncertainty is their sum.
  //
  // This is added on publication rather than folded into the measurement
  // covariance, and the distinction is the whole point. A survey error is the
  // same error in every frame that sees the same stretch of road. Put it in the
  // measurement and the filter averages it across frames like noise, driving
  // its own covariance down while the error it describes does not move -- the
  // same mistake the aggregation window makes with a reused frame, one level
  // up. A bias does not average away, so it is never offered to the update.
  result_.covariance = kf_.covariance();
  AddMapUncertaintyTo(
      map_loader_ ? map_loader_->uncertainty() : map::MapUncertainty{},
      result_.T_world_rig, result_.covariance);
}

Status LocalizationEngine::ProcessFrame(
    const kitti::Egomotion& ego, const kitti::FramePerception& perception) {
  // --- Initialization (frame 0) ---
  if (!kf_.initialized()) {
    SE3State init;
    init.translation = ego.global.T_world_cam0.block<3, 1>(0, 3);
    init.rotation = ego.global.T_world_cam0.block<3, 3>(0, 0);
    kf_.Initialize(init, ego.cov_global);
    // Nothing has corrected the filter yet, so its covariance is the prior's
    // and says nothing about where the map match will land.
    full_extent_frames_ = params_.adaptive_extent.reexpand_frames;
  } else if (ego.global.frame > 0) {
    // --- Prediction: propagate with relative odometry T_curr_prev ---
    kf_.Predict(ego.T_curr_prev,
                kf_.MotionProcessCov(ego.T_curr_prev, params_.odometry));
    total_travel_m_ += ego.T_curr_prev.block<3, 1>(0, 3).head<2>().norm();
  }

  kitti::MapChunk local_map;
  kitti::FramePerception active_perception;
  const Status match_st =
      RunMapMatching(ego, perception, local_map, active_perception);

  // --- Updates (observations) ---
  if (params_.use_gt_global_prior) {
    // Debug: near-perfect GT observation every frame.
    SE3State gt;
    gt.translation = ego.global.T_world_cam0.block<3, 1>(0, 3);
    gt.rotation = ego.global.T_world_cam0.block<3, 3>(0, 0);
    kf_.Update(gt, ego.cov_global * 0.01);
  } else if (match_st != Status::kOk) {
    // Map matching failed: optional fallback to global odometry measurement.
    ApplyGlobalMeasurement(ego);
  }
  // Note: successful map matching already called kf_.Update inside
  // ApplySamplingMeasurement.

  WriteResult(ego);
  return Status::kOk;
}

}  // namespace cam_loc::core
