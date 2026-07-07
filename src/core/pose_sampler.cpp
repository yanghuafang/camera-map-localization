// Pose sampling: perception rasterization → labelled DT → per-hypothesis map-matching costs.

#include <cam_loc/core/pose_sampler.hpp>

#include <cam_loc/core/distance_transform_cpu.hpp>

#ifdef CAMLOC_CUDA_ENABLED
#include <cam_loc/cuda/distance_transform.hpp>
#endif

#include <cmath>

namespace cam_loc::core {

namespace {

uint8_t typeToLabel(kitti::PolylineType type) {
  // DT label channel: 1=solid lane, 2=dashed, 3=road edge, 0=unlabelled
  switch (type) {
    case kitti::PolylineType::kLaneSolid:
      return 1;
    case kitti::PolylineType::kLaneDashed:
      return 2;
    case kitti::PolylineType::kRoadEdge:
      return 3;
    default:
      return 0;
  }
}

void rasterizePolylineList(const std::vector<kitti::Polyline2D>& polylines, int width, int height,
                           float stroke, std::vector<uint8_t>& binary,
                           std::vector<uint8_t>& labels) {
  binary.assign(static_cast<size_t>(width * height), 255);
  labels.assign(static_cast<size_t>(width * height), 0);

  for (const auto& pl : polylines) {
    if (pl.points.size() < 2) continue;
    std::vector<uint8_t> tmp;
    DistanceTransformCpu::rasterizePolylines(pl.points, width, height, stroke, tmp);
    const uint8_t label = typeToLabel(pl.type);
    for (size_t i = 0; i < tmp.size(); ++i) {
      if (tmp[i] == 0) {
        binary[i] = 0;
        labels[i] = label;
      }
    }
  }
}

float bilinearSample(const std::vector<float>& img, int width, int height, double u, double v) {
  if (u < 0 || v < 0 || u >= width - 1 || v >= height - 1) {
    return std::numeric_limits<float>::max();
  }
  const int x0 = static_cast<int>(std::floor(u));
  const int y0 = static_cast<int>(std::floor(v));
  const int x1 = x0 + 1;
  const int y1 = y0 + 1;
  const double tx = u - x0;
  const double ty = v - y0;
  const auto at = [&](int x, int y) { return img[static_cast<size_t>(y * width + x)]; };
  const double v00 = at(x0, y0);
  const double v10 = at(x1, y0);
  const double v01 = at(x0, y1);
  const double v11 = at(x1, y1);
  const double v0 = v00 * (1 - tx) + v10 * tx;
  const double v1 = v01 * (1 - tx) + v11 * tx;
  return static_cast<float>(v0 * (1 - ty) + v1 * ty);
}

uint8_t nearestLabel(const std::vector<uint8_t>& labels, int width, int height, double u,
                     double v) {
  const int x = static_cast<int>(std::lround(u));
  const int y = static_cast<int>(std::lround(v));
  if (x < 0 || y < 0 || x >= width || y >= height) return 0;
  return labels[static_cast<size_t>(y * width + x)];
}

Vec3 worldToRig(const Mat44& T_world_rig, const Vec3& p_world) {
  const Eigen::Matrix3d R = T_world_rig.block<3, 3>(0, 0);
  const Vec3 t = T_world_rig.block<3, 1>(0, 3);
  return R.transpose() * (p_world - t);
}

Mat44 hypothesisPose(const Mat44& T_world_plane, double x_m, double y_m, double yaw_rad) {
  // Compose plane pose with grid offset: T_world_hyp = T_world_plane · T_offset
  return T_world_plane * Projection::offsetToTransform(x_m, y_m, yaw_rad);
}

void packMapPoints(const kitti::MapChunk& map, std::vector<float>& xyz,
                   std::vector<uint8_t>& labels) {
  for (const auto& pl : map.polylines) {
    for (const auto& p : pl.points) {
      xyz.push_back(static_cast<float>(p.x()));
      xyz.push_back(static_cast<float>(p.y()));
      xyz.push_back(static_cast<float>(p.z()));
      labels.push_back(typeToLabel(pl.type));
    }
  }
}

}  // namespace

PoseSampler::PoseSampler(const LocalizationParams& params) : params_(params) {}

void PoseSampler::setProjection(const Projection& projection) { projection_ = &projection; }

Status PoseSampler::buildImageDt(const kitti::FramePerception& perception,
                                 LabelledDistanceTransform& out) {
  if (!projection_) return Status::kInvalidArgument;

  constexpr int kWidth = 1241;
  constexpr int kHeight = 376;
  out.width = kWidth;
  out.height = kHeight;
  out.max_cost = 5.f;

  // Rasterize image-space polylines (stroke 2 px) then Felzenszwalb EDT
  std::vector<kitti::Polyline2D> all = perception.lane_lines;
  all.insert(all.end(), perception.road_boundaries.begin(), perception.road_boundaries.end());

  std::vector<uint8_t> binary;
  rasterizePolylineList(all, kWidth, kHeight, 2.f, binary, out.labels);
#ifdef CAMLOC_CUDA_ENABLED
  if (params_.use_cuda && cuda::isAvailable()) {
    return cuda::computeDistanceTransformGpu(binary, kWidth, kHeight, out.distance);
  }
#endif
  return DistanceTransformCpu::compute(binary, kWidth, kHeight, out.distance);
}

Status PoseSampler::buildBevDtFromImagePerception(const kitti::FramePerception& perception,
                                                  LabelledDistanceTransform& out) {
  if (!projection_) return Status::kInvalidArgument;

  out.width = BevConfig::kImageWidth;
  out.height = BevConfig::kImageHeight;
  out.max_cost = BevConfig::kDistanceMax;

  // Image UV → ground rig (Z=0) → BEV meters → rasterize at BEV resolution
  std::vector<kitti::Polyline2D> bev_polylines;
  auto convert = [&](const std::vector<kitti::Polyline2D>& src) {
    for (const auto& pl : src) {
      kitti::Polyline2D bev_pl;
      bev_pl.type = pl.type;
      for (const auto& uv : pl.points) {
        Vec3 p_rig;
        if (projection_->imageToGroundRig(uv, 0.0, p_rig) != Status::kOk) continue;
        int col = 0, row = 0;
        if (Projection::rigToBevPixel(p_rig, col, row) != Status::kOk) continue;
        const double x = BevConfig::kXMin + (col + 0.5) * BevConfig::metersPerPixelX();
        const double y = BevConfig::kYMin + (row + 0.5) * BevConfig::metersPerPixelY();
        bev_pl.points.emplace_back(x, y);
      }
      if (bev_pl.points.size() >= 2) bev_polylines.push_back(std::move(bev_pl));
    }
  };
  convert(perception.lane_lines);
  convert(perception.road_boundaries);

  std::vector<uint8_t> binary;
  rasterizePolylineList(bev_polylines, out.width, out.height, 1.5f, binary, out.labels);
#ifdef CAMLOC_CUDA_ENABLED
  if (params_.use_cuda && cuda::isAvailable()) {
    return cuda::computeDistanceTransformGpu(binary, out.width, out.height, out.distance);
  }
#endif
  return DistanceTransformCpu::compute(binary, out.width, out.height, out.distance);
}

float PoseSampler::sampleImageCost(const LabelledDistanceTransform& dt, const Vec2& uv,
                                   kitti::PolylineType type) const {
  const uint8_t label = nearestLabel(dt.labels, dt.width, dt.height, uv.x(), uv.y());
  if (label != 0 && label != typeToLabel(type)) {
    return dt.max_cost;
  }
  const float d = bilinearSample(dt.distance, dt.width, dt.height, uv.x(), uv.y());
  return std::min(d, dt.max_cost);
}

float PoseSampler::sampleBevCost(const LabelledDistanceTransform& dt, const Vec3& p_rig,
                                 kitti::PolylineType type) const {
  int col = 0, row = 0;
  if (Projection::rigToBevPixel(p_rig, col, row) != Status::kOk) {
    return dt.max_cost;
  }
  const double u = col + 0.5;
  const double v = row + 0.5;
  return sampleImageCost(dt, Vec2(u, v), type);
}

Status PoseSampler::computeImageCosts(const kitti::MapChunk& map, const Mat44& T_world_plane,
                                    const LabelledDistanceTransform& dt,
                                    CostGrid& costs) const {
  if (!projection_) return Status::kInvalidArgument;

#ifdef CAMLOC_CUDA_ENABLED
  if (params_.use_cuda && cuda::isAvailable()) {
    std::vector<float> map_xyz;
    std::vector<uint8_t> map_labels;
    packMapPoints(map, map_xyz, map_labels);
    if (!map_xyz.empty()) {
      float T[16];
      for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
          T[r * 4 + c] = static_cast<float>(T_world_plane(r, c));
        }
      }
      cuda::PoseCostGpuParams gp;
      gp.num_x = params_.grid.num_x;
      gp.num_y = params_.grid.num_y;
      gp.num_yaw = params_.grid.num_yaw;
      gp.step_x_m = params_.grid.step_x_m;
      gp.step_y_m = params_.grid.step_y_m;
      gp.step_yaw_deg = params_.grid.step_yaw_deg;
      gp.fx = projection_->fx();
      gp.fy = projection_->fy();
      gp.cx = projection_->cx();
      gp.cy = projection_->cy();
      gp.dt_max_cost = dt.max_cost;
      gp.dt_width = dt.width;
      gp.dt_height = dt.height;
      std::vector<float> gpu_costs;
      const int npts = static_cast<int>(map_labels.size());
      if (cuda::computeImagePoseCostsGpu(T, map_xyz.data(), npts, map_labels.data(),
                                         dt.distance.data(), dt.labels.data(), gp,
                                         gpu_costs) == Status::kOk) {
        costs.data() = gpu_costs;
        return Status::kOk;
      }
    }
  }
#endif

  costs.fill(dt.max_cost * 10.f);
  // Brute-force CPU fallback: one thread-equivalent loop over all hypotheses
  for (int iw = 0; iw < costs.dimW(); ++iw) {
    for (int iy = 0; iy < costs.dimY(); ++iy) {
      for (int ix = 0; ix < costs.dimX(); ++ix) {
        const Vec3 offset = costs.indexToOffset(ix, iy, iw);
        const Mat44 T_world_hyp =
            hypothesisPose(T_world_plane, offset.x(), offset.y(), offset.z());

        float total = 0.f;
        int count = 0;
        for (const auto& pl : map.polylines) {
          for (const auto& p_world : pl.points) {
            const Vec3 p_rig = worldToRig(T_world_hyp, p_world);
            if (p_rig.z() <= 0.5) continue;
            Vec2 uv;
            if (projection_->projectRigToImage(p_rig, uv) != Status::kOk) continue;
            total += sampleImageCost(dt, uv, pl.type);
            ++count;
          }
        }
        costs.at(ix, iy, iw) = count > 0 ? total / static_cast<float>(count) : dt.max_cost * 10.f;
      }
    }
  }
  return Status::kOk;
}

Status PoseSampler::computeBevCosts(const kitti::MapChunk& map, const Mat44& T_world_plane,
                                    const LabelledDistanceTransform& dt,
                                    CostGrid& costs) const {
  if (!projection_) return Status::kInvalidArgument;

#ifdef CAMLOC_CUDA_ENABLED
  if (params_.use_cuda && cuda::isAvailable()) {
    std::vector<float> map_xyz;
    std::vector<uint8_t> map_labels;
    packMapPoints(map, map_xyz, map_labels);
    if (!map_xyz.empty()) {
      float T[16];
      for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
          T[r * 4 + c] = static_cast<float>(T_world_plane(r, c));
        }
      }
      cuda::PoseCostGpuParams gp;
      gp.num_x = params_.grid.num_x;
      gp.num_y = params_.grid.num_y;
      gp.num_yaw = params_.grid.num_yaw;
      gp.step_x_m = params_.grid.step_x_m;
      gp.step_y_m = params_.grid.step_y_m;
      gp.step_yaw_deg = params_.grid.step_yaw_deg;
      gp.dt_max_cost = dt.max_cost;
      gp.dt_width = dt.width;
      gp.dt_height = dt.height;
      gp.bev_x_min = static_cast<float>(BevConfig::kXMin);
      gp.bev_x_max = static_cast<float>(BevConfig::kXMax);
      gp.bev_y_min = static_cast<float>(BevConfig::kYMin);
      gp.bev_y_max = static_cast<float>(BevConfig::kYMax);
      gp.bev_mpp_x = static_cast<float>(BevConfig::metersPerPixelX());
      gp.bev_mpp_y = static_cast<float>(BevConfig::metersPerPixelY());
      std::vector<float> gpu_costs;
      const int npts = static_cast<int>(map_labels.size());
      if (cuda::computeBevPoseCostsGpu(T, map_xyz.data(), npts, map_labels.data(),
                                       dt.distance.data(), dt.labels.data(), gp,
                                       gpu_costs) == Status::kOk) {
        costs.data() = gpu_costs;
        return Status::kOk;
      }
    }
  }
#endif

  costs.fill(dt.max_cost * 10.f);

  // CPU fallback: score hypotheses in rig/BEV frame (no pinhole projection)
  for (int iw = 0; iw < costs.dimW(); ++iw) {
    for (int iy = 0; iy < costs.dimY(); ++iy) {
      for (int ix = 0; ix < costs.dimX(); ++ix) {
        const Vec3 offset = costs.indexToOffset(ix, iy, iw);
        const Mat44 T_world_hyp =
            hypothesisPose(T_world_plane, offset.x(), offset.y(), offset.z());

        float total = 0.f;
        int count = 0;
        for (const auto& pl : map.polylines) {
          for (const auto& p_world : pl.points) {
            const Vec3 p_rig = worldToRig(T_world_hyp, p_world);
            if (p_rig.x() < BevConfig::kXMin || p_rig.x() > BevConfig::kXMax ||
                p_rig.y() < BevConfig::kYMin || p_rig.y() > BevConfig::kYMax) {
              continue;
            }
            total += sampleBevCost(dt, p_rig, pl.type);
            ++count;
          }
        }
        costs.at(ix, iy, iw) = count > 0 ? total / static_cast<float>(count) : dt.max_cost * 10.f;
      }
    }
  }
  return Status::kOk;
}

}  // namespace cam_loc::core
