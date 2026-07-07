/// Per-frame localization orchestrator: EKF predict → map matching → optional global update.
#include <cam_loc/core/localization_engine.hpp>

#include <cam_loc/core/sampling_covariance.hpp>
#include <cam_loc/map/map_loader.hpp>
#include <cam_loc/perception/synthesize.hpp>

#ifdef CAMLOC_CUDA_ENABLED
#include <cam_loc/cuda/distance_transform.hpp>
#endif

#include <cmath>
#include <limits>

namespace cam_loc::core {

LocalizationEngine::LocalizationEngine(LocalizationParams params)
    : params_(std::move(params)),
      pose_sampler_(params_),
      aggregator_(params_.aggregation) {}

void LocalizationEngine::setMapLoader(std::shared_ptr<map::IMapLoader> loader) {
  map_loader_ = std::move(loader);
}

void LocalizationEngine::setCalibration(const kitti::Calibration& calib) {
  projection_.emplace(calib);
  pose_sampler_.setProjection(*projection_);
}

Mat44 LocalizationEngine::samplingPlanePose(const kitti::Egomotion& ego) const {
  // Anchor for the (x, y, yaw) cost grid: GT pose (oracle) or current KF estimate.
  if (params_.use_gt_sampling_plane || !kf_.isInitialized()) {
    return ego.global.T_world_cam0;
  }
  const SE3State s = kf_.state();
  Mat44 T = Mat44::Identity();
  T.block<3, 3>(0, 0) = s.rotation;
  T.block<3, 1>(0, 3) = s.translation;
  return T;
}

bool LocalizationEngine::isCostMapFlat(const CostGrid& grid,
                                       const CostGrid::ArgMinResult& argmin) const {
  float max_c = argmin.cost;
  float min_c = argmin.cost;
  for (float c : grid.data()) {
    max_c = std::max(max_c, c);
    min_c = std::min(min_c, c);
  }
  return (max_c - min_c) < params_.cost_flat_threshold;
}

Status LocalizationEngine::runMapMatching(const kitti::Egomotion& ego,
                                          const kitti::FramePerception& perception,
                                          kitti::MapChunk& local_map,
                                          kitti::FramePerception& active_perception) {
  // --- Observation generation (map matching) ---
  // Builds a pose measurement for the EKF from perception + local map alignment.
  // Does not call kf_.update when the cost surface is too flat (ambiguous match).
  if (!projection_) {
    return Status::kInvalidArgument;
  }
  if (!map_loader_) {
    return Status::kInvalidArgument;
  }

  const Mat44 T_plane = samplingPlanePose(ego);
  if (map_loader_->queryLocalMap(T_plane, params_.map_query_radius_m, local_map) != Status::kOk) {
    return Status::kInvalidArgument;
  }
  if (local_map.polylines.empty()) {
    return Status::kInvalidArgument;
  }

  // Perception: use caller input or synthesize by projecting map into the image.
  active_perception = perception;
  const bool has_perception = !active_perception.lane_lines.empty() ||
                              !active_perception.road_boundaries.empty();
  result_.perception_synthesized = false;
  if (!has_perception) {
    active_perception =
        perception::synthesizeFromMap(local_map, *projection_, T_plane, ego.global.frame);
    result_.perception_synthesized = true;
  }
  if (active_perception.lane_lines.empty() && active_perception.road_boundaries.empty()) {
    return Status::kInvalidArgument;
  }

  LabelledDistanceTransform image_dt;
  if (pose_sampler_.buildImageDt(active_perception, image_dt) != Status::kOk) {
    return Status::kInvalidArgument;
  }

  // Score each (x, y, yaw) hypothesis: project map points, sample perception DT.
  CostGrid raw_costs(params_.grid);
  Status st = pose_sampler_.computeImageCosts(local_map, T_plane, image_dt, raw_costs);
  if (st != Status::kOk) return st;

  LabelledDistanceTransform bev_dt;
  bool has_bev_dt = false;
  if (params_.enable_bev) {
    if (pose_sampler_.buildBevDtFromImagePerception(active_perception, bev_dt) == Status::kOk) {
      has_bev_dt = true;
      CostGrid bev_costs(params_.grid);
      if (pose_sampler_.computeBevCosts(local_map, T_plane, bev_dt, bev_costs) == Status::kOk) {
        // Equal-weight fusion of image and BEV pose costs (see docs/ARCHITECTURE.md).
        for (size_t i = 0; i < raw_costs.data().size(); ++i) {
          raw_costs.data()[i] = 0.5f * raw_costs.data()[i] + 0.5f * bev_costs.data()[i];
        }
      }
    }
  }

  CostGrid aggregated = raw_costs;
  bool use_gpu = params_.use_cuda;
#ifdef CAMLOC_CUDA_ENABLED
  use_gpu = use_gpu && cuda::isAvailable();
#endif
  // Temporal fusion: warp past cost volumes into the current sampling plane.
  aggregator_.aggregate(aggregated, T_plane, total_travel_m_, use_gpu);
  aggregator_.pushHistory(raw_costs, T_plane, ego.global.frame);

  const auto argmin = aggregated.argmin(use_gpu);
  result_.aggregate_min_cost = argmin.cost;
  result_.best_sample_xyyaw = aggregated.indexToOffset(argmin.ix, argmin.iy, argmin.iw);
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
  result_.cost_map_flat = isCostMapFlat(aggregated, argmin);
  result_.sampling_measurement_applied = false;

  // Flat cost map → skip observation (no informative map-matching update).
  if (result_.cost_map_flat) {
    return Status::kOk;
  }

  result_.sampling_measurement_applied = true;
  return applySamplingMeasurement(ego, T_plane, aggregated, argmin);
}

Status LocalizationEngine::applySamplingMeasurement(const kitti::Egomotion& ego,
                                                    const Mat44& T_world_plane,
                                                    const CostGrid& aggregated,
                                                    const CostGrid::ArgMinResult& argmin) {
  // Map-matching observation: best grid cell → full SE(3) pose in world frame.
  const Vec3 offset = result_.best_sample_xyyaw;
  const Mat44 T_sample =
      T_world_plane * Projection::offsetToTransform(offset.x(), offset.y(), offset.z());

  SE3State meas;
  meas.translation = T_sample.block<3, 1>(0, 3);
  meas.rotation = T_sample.block<3, 3>(0, 0);

  const auto conf =
      SamplingCovariance::compute(aggregated, argmin, params_.cost_softmax_scale);

  // Measurement noise: tight on (x, y, yaw) from cost-surface spread; loose on z, roll, pitch.
  // Error-state order is [x, y, z, roll, pitch, yaw], so the 3-DOF cost covariance (x, y, yaw)
  // maps to indices 0, 1, 5 while z/roll/pitch (2, 3, 4) get fixed large values.
  Mat66 meas_cov = Mat66::Identity();
  if (conf.valid) {
    meas_cov(0, 0) = std::max(conf.covariance(0, 0), 1e-4);
    meas_cov(1, 1) = std::max(conf.covariance(1, 1), 1e-4);
    meas_cov(5, 5) = std::max(conf.covariance(2, 2), 1e-6);
  } else {
    meas_cov.block<2, 2>(0, 0) *= 0.25;
    meas_cov(5, 5) = 0.01;
  }
  meas_cov(2, 2) = 100.0;
  meas_cov(3, 3) = 1.0;
  meas_cov(4, 4) = 1.0;

  kf_.update(meas, meas_cov);

  (void)ego;
  return Status::kOk;
}

void LocalizationEngine::applyGlobalMeasurement(const kitti::Egomotion& ego) {
  if (!params_.use_global_ego_measurement) return;

  SE3State meas;
  meas.translation = ego.global.T_world_cam0.block<3, 1>(0, 3);
  meas.rotation = ego.global.T_world_cam0.block<3, 3>(0, 0);
  kf_.update(meas, ego.cov_global);
}

void LocalizationEngine::writeResult(const kitti::Egomotion& ego) {
  result_.frame = ego.global.frame;
  result_.timestamp_ns = ego.global.timestamp_ns;
  result_.valid = kf_.isInitialized();

  const SE3State s = kf_.state();
  result_.T_world_rig = Mat44::Identity();
  result_.T_world_rig.block<3, 3>(0, 0) = s.rotation;
  result_.T_world_rig.block<3, 1>(0, 3) = s.translation;
  result_.covariance = kf_.covariance();
}

Status LocalizationEngine::processFrame(const kitti::Egomotion& ego,
                                        const kitti::FramePerception& perception) {
  // --- Initialization (frame 0) ---
  if (!kf_.isInitialized()) {
    SE3State init;
    init.translation = ego.global.T_world_cam0.block<3, 1>(0, 3);
    init.rotation = ego.global.T_world_cam0.block<3, 3>(0, 0);
    kf_.initialize(init, ego.cov_global);
  } else if (ego.global.frame > 0) {
    // --- Prediction: propagate with relative odometry T_curr_prev ---
    kf_.predict(ego.T_curr_prev, LocalizationKF::defaultProcessCov());
    total_travel_m_ += ego.T_curr_prev.block<3, 1>(0, 3).head<2>().norm();
  }

  kitti::MapChunk local_map;
  kitti::FramePerception active_perception;
  const Status match_st = runMapMatching(ego, perception, local_map, active_perception);

  // --- Updates (observations) ---
  if (params_.use_gt_global_prior) {
    // Debug: near-perfect GT observation every frame.
    SE3State gt;
    gt.translation = ego.global.T_world_cam0.block<3, 1>(0, 3);
    gt.rotation = ego.global.T_world_cam0.block<3, 3>(0, 0);
    kf_.update(gt, ego.cov_global * 0.01);
  } else if (match_st != Status::kOk) {
    // Map matching failed: optional fallback to global odometry measurement.
    applyGlobalMeasurement(ego);
  }
  // Note: successful map matching already called kf_.update inside applySamplingMeasurement.

  writeResult(ego);
  return Status::kOk;
}

}  // namespace cam_loc::core
