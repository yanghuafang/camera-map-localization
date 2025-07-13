/// Per-frame localization orchestrator: EKF predict → map matching → optional
/// global update.
#include "cam_loc/core/localization_engine.h"

#include "cam_loc/core/frames.h"
#include "cam_loc/core/sampling_covariance.h"
#include "cam_loc/map/map_loader.h"

#ifdef CAMLOC_CUDA_ENABLED
#include "cam_loc/cuda/distance_transform.h"
#endif

#include <cmath>
#include <limits>

namespace cam_loc::core {

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

bool LocalizationEngine::IsCostMapFlat(
    const CostGrid& grid, const CostGrid::ArgMinResult& argmin) const {
  float max_c = argmin.cost;
  float min_c = argmin.cost;
  for (float c : grid.data()) {
    max_c = std::max(max_c, c);
    min_c = std::min(min_c, c);
  }
  return (max_c - min_c) < params_.cost_flat_threshold;
}

Status LocalizationEngine::RunMapMatching(
    const kitti::Egomotion& ego, const kitti::FramePerception& perception,
    kitti::MapChunk& local_map, kitti::FramePerception& active_perception) {
  // --- Observation generation (map matching) ---
  // Builds a pose measurement for the EKF from perception + local map
  // alignment. Does not call kf_.Update when the cost surface is too flat
  // (ambiguous match).
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

  if (params_.enable_image) {
    CostGrid image_costs(params_.grid);
    const Status st = pose_sampler_.ComputeImageCosts(local_map, T_plane,
                                                      image_dt, image_costs);
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
      if (pose_sampler_.ComputeBevCosts(local_map, T_plane, bev_dt,
                                        bev_costs) == Status::kOk) {
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
    debug_.raw_costs = raw_costs;
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
  result_.cost_map_flat = IsCostMapFlat(aggregated, argmin);
  result_.match_cost_too_high = argmin.cost > params_.max_match_cost;
  result_.sampling_measurement_applied = false;

  // Two ways a frame has nothing to say. A flat surface cannot tell hypotheses
  // apart; a uniformly high one says none of them fit. Either way the argmin is
  // not evidence, and feeding it to the filter is worse than coasting on the
  // motion model.
  if (result_.cost_map_flat || result_.match_cost_too_high) {
    return Status::kOk;
  }

  result_.sampling_measurement_applied = true;
  return ApplySamplingMeasurement(T_plane, aggregated, argmin);
}

Status LocalizationEngine::ApplySamplingMeasurement(
    const Mat44& T_world_plane, const CostGrid& aggregated,
    const CostGrid::ArgMinResult& argmin) {
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

  // The grid measures three degrees of freedom in the *vehicle* frame: forward,
  // left, and heading. The filter's error state is world-frame, so build the
  // covariance on vehicle axes and rotate it, rather than dropping the three
  // variances into world slots that only line up when the vehicle happens to
  // face along a world axis.
  //
  // The unmeasured three -- height, roll and pitch -- get large fixed values so
  // the update leaves them to the motion model.
  constexpr double kUnmeasuredTranslationVar = 100.0;  // m²
  constexpr double kUnmeasuredRotationVar = 1.0;       // rad²

  Vec3 translation_var(kUnmeasuredTranslationVar, kUnmeasuredTranslationVar,
                       kUnmeasuredTranslationVar);
  Vec3 rotation_var(kUnmeasuredRotationVar, kUnmeasuredRotationVar,
                    kUnmeasuredRotationVar);
  if (conf.valid) {
    // Floors keep a razor-sharp cost surface from producing a near-zero
    // variance, which would make the filter ignore its own motion model.
    translation_var.x() = std::max(conf.covariance(0, 0), 1e-4);  // forward
    translation_var.y() = std::max(conf.covariance(1, 1), 1e-4);  // left
    rotation_var.z() = std::max(conf.covariance(2, 2), 1e-6);     // heading
  } else {
    translation_var.x() = 0.25;
    translation_var.y() = 0.25;
    rotation_var.z() = 0.01;
  }

  const Eigen::Matrix3d R_world_vehicle =
      meas.rotation * Frames::RotCam0Vehicle();
  Mat66 meas_cov = Mat66::Zero();
  meas_cov.block<3, 3>(0, 0) = R_world_vehicle * translation_var.asDiagonal() *
                               R_world_vehicle.transpose();
  meas_cov.block<3, 3>(3, 3) =
      R_world_vehicle * rotation_var.asDiagonal() * R_world_vehicle.transpose();

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
  result_.covariance = kf_.covariance();
}

Status LocalizationEngine::ProcessFrame(
    const kitti::Egomotion& ego, const kitti::FramePerception& perception) {
  // --- Initialization (frame 0) ---
  if (!kf_.initialized()) {
    SE3State init;
    init.translation = ego.global.T_world_cam0.block<3, 1>(0, 3);
    init.rotation = ego.global.T_world_cam0.block<3, 3>(0, 0);
    kf_.Initialize(init, ego.cov_global);
  } else if (ego.global.frame > 0) {
    // --- Prediction: propagate with relative odometry T_curr_prev ---
    kf_.Predict(ego.T_curr_prev, ego.cov_relative);
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
