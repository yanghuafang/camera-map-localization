// Pose sampling: perception rasterization → labelled DT → per-hypothesis
// map-matching costs.

#include "cam_loc/core/pose_sampler.h"

#include "cam_loc/core/distance_transform_cpu.h"
#include "cam_loc/core/frames.h"

#ifdef CAMLOC_CUDA_ENABLED
#include "cam_loc/cuda/distance_transform.h"
#endif

#include <array>
#include <cmath>

namespace cam_loc::core {

namespace {

uint8_t TypeToLabel(kitti::PolylineType type) {
  // DT label channel. A map point may only match perception of its own class,
  // so every class that is detected has to have a channel of its own -- poles
  // and signs shared the unlabelled 0 with kUnknown until they were extracted,
  // which silently disabled the class gate for exactly the two classes that
  // constrain along-track position.
  switch (type) {
    case kitti::PolylineType::kLaneSolid:
      return 1;
    case kitti::PolylineType::kLaneDashed:
      return 2;
    case kitti::PolylineType::kRoadEdge:
      return 3;
    case kitti::PolylineType::kPole:
      return 4;
    case kitti::PolylineType::kSign:
      return 5;
    default:
      return 0;
  }
}

// One accumulator per DT label channel, so each class can be scored on its own.
constexpr int kNumTypeChannels = 6;
using ClassCostSums = std::array<float, kNumTypeChannels>;
using ClassCounts = std::array<int, kNumTypeChannels>;

// Score for one class, given the summed DT cost over the map points of that
// class the hypothesis managed to project, and how many that was.
//
// A plain mean lets a hypothesis that projects two well-aligned points beat one
// that aligns two hundred, which is how a pose looking away from the map could
// win. Treating the missing support as maximally wrong removes that, and it
// keeps every cell inside [0, max_cost] -- so the spread across the grid
// measures real variation, instead of being dominated by an out-of-band
// sentinel for cells that projected nothing.
float SupportWeightedCost(float total, int count, int min_support,
                          float max_cost) {
  const int effective = count > min_support ? count : min_support;
  const auto missing = static_cast<float>(effective - count);
  return (total + missing * max_cost) / static_cast<float>(effective);
}

// Combine per-class scores into the cost for one hypothesis.
//
// The average is over *classes*, not over points, and that is the whole point.
// A lane boundary is sampled every couple of metres along the road, so it
// contributes an order of magnitude more points than the handful belonging to a
// pole -- and lane geometry runs parallel to travel, so its cost barely changes
// as the hypothesis slides forward. Averaging per point lets that flat majority
// outvote the sparse landmarks that actually pin along-track position, and the
// argmin wanders off down the road. Per class, a pole counts as much as a lane.
//
// Classes the local map does not contain are skipped rather than scored as
// missing: their absence says nothing about the pose.
float CombineClassCosts(const ClassCostSums& sums, const ClassCounts& counts,
                        const std::array<bool, kNumTypeChannels>& present,
                        int min_support, float max_cost) {
  float acc = 0.f;
  int classes = 0;
  for (int ch = 0; ch < kNumTypeChannels; ++ch) {
    if (!present[ch]) continue;
    acc += SupportWeightedCost(sums[ch], counts[ch], min_support, max_cost);
    ++classes;
  }
  return classes > 0 ? acc / static_cast<float>(classes) : max_cost;
}

/// Which DT label channels the local map can offer at all.
std::array<bool, kNumTypeChannels> PresentChannels(const kitti::MapChunk& map,
                                                   bool ground_plane_only) {
  std::array<bool, kNumTypeChannels> present{};
  for (const auto& pl : map.polylines) {
    if (ground_plane_only && !kitti::IsGroundPlaneType(pl.type)) continue;
    present[TypeToLabel(pl.type)] = true;
  }
  return present;
}

void RasterizePolylineList(const std::vector<kitti::Polyline2D>& polylines,
                           int width, int height, float stroke,
                           std::vector<uint8_t>& binary,
                           std::vector<uint8_t>& labels) {
  binary.assign(static_cast<size_t>(width * height), 255);
  labels.assign(static_cast<size_t>(width * height), 0);

  for (const auto& pl : polylines) {
    if (pl.points.size() < 2) continue;
    std::vector<uint8_t> tmp;
    DistanceTransformCpu::RasterizePolylines(pl.points, width, height, stroke,
                                             tmp);
    const uint8_t label = TypeToLabel(pl.type);
    for (size_t i = 0; i < tmp.size(); ++i) {
      if (tmp[i] == 0) {
        binary[i] = 0;
        labels[i] = label;
      }
    }
  }
}

float BilinearSample(const std::vector<float>& img, int width, int height,
                     double u, double v) {
  if (u < 0 || v < 0 || u >= width - 1 || v >= height - 1) {
    return std::numeric_limits<float>::max();
  }
  const int x0 = static_cast<int>(std::floor(u));
  const int y0 = static_cast<int>(std::floor(v));
  const int x1 = x0 + 1;
  const int y1 = y0 + 1;
  const double tx = u - x0;
  const double ty = v - y0;
  const auto at = [&](int x, int y) {
    return img[static_cast<size_t>(y * width + x)];
  };
  const double v00 = at(x0, y0);
  const double v10 = at(x1, y0);
  const double v01 = at(x0, y1);
  const double v11 = at(x1, y1);
  const double v0 = v00 * (1 - tx) + v10 * tx;
  const double v1 = v01 * (1 - tx) + v11 * tx;
  return static_cast<float>(v0 * (1 - ty) + v1 * ty);
}

uint8_t NearestLabel(const std::vector<uint8_t>& labels, int width, int height,
                     double u, double v) {
  const int x = static_cast<int>(std::lround(u));
  const int y = static_cast<int>(std::lround(v));
  if (x < 0 || y < 0 || x >= width || y >= height) return 0;
  return labels[static_cast<size_t>(y * width + x)];
}

Vec3 WorldToRig(const Mat44& T_world_rig, const Vec3& p_world) {
  const Eigen::Matrix3d R = T_world_rig.block<3, 3>(0, 0);
  const Vec3 t = T_world_rig.block<3, 1>(0, 3);
  return R.transpose() * (p_world - t);
}

Mat44 HypothesisPose(const Mat44& T_world_plane, double x_m, double y_m,
                     double yaw_rad) {
  // T_world_hyp = T_world_plane · T_offset, with the offset expressed as a
  // vehicle-frame SE(2) move (forward, left, heading) rewritten in cam0.
  return T_world_plane * Frames::OffsetToCam0Transform(x_m, y_m, yaw_rad);
}

#ifdef CAMLOC_CUDA_ENABLED
// Flattens the map's polyline points into the flat xyz + label arrays the
// kernels take. Only the GPU path needs it, and it is compiled only there: left
// visible in a CPU-only build it is an unused function, which -Wall reports.
void PackMapPoints(const kitti::MapChunk& map, std::vector<float>& xyz,
                   std::vector<uint8_t>& labels) {
  for (const auto& pl : map.polylines) {
    for (const auto& p : pl.points) {
      xyz.push_back(static_cast<float>(p.x()));
      xyz.push_back(static_cast<float>(p.y()));
      xyz.push_back(static_cast<float>(p.z()));
      labels.push_back(TypeToLabel(pl.type));
    }
  }
}
#endif

}  // namespace

PoseSampler::PoseSampler(const LocalizationParams& params) : params_(params) {}

void PoseSampler::set_projection(const Projection& projection) {
  projection_ = &projection;
}

Status PoseSampler::BuildImageDt(const kitti::FramePerception& perception,
                                 LabelledDistanceTransform& out) {
  if (projection_ == nullptr) return Status::kInvalidArgument;

  out.width = params_.image_width;
  out.height = params_.image_height;
  out.max_cost = 5.f;

  // Rasterize image-space polylines (stroke 2 px) then Felzenszwalb EDT
  std::vector<uint8_t> binary;
  RasterizePolylineList(perception.features, out.width, out.height, 2.f, binary,
                        out.labels);
#ifdef CAMLOC_CUDA_ENABLED
  if (params_.use_cuda && cuda::IsAvailable()) {
    return cuda::ComputeDistanceTransformGpu(binary, out.width, out.height,
                                             out.distance);
  }
#endif
  return DistanceTransformCpu::Compute(binary, out.width, out.height,
                                       out.distance);
}

Status PoseSampler::BuildBevDtFromImagePerception(
    const kitti::FramePerception& perception, LabelledDistanceTransform& out) {
  if (projection_ == nullptr) return Status::kInvalidArgument;

  out.width = BevConfig::kImageWidth;
  out.height = BevConfig::kImageHeight;
  out.max_cost = BevConfig::kDistanceMax;

  // Image pixel → road plane → BEV pixel. Only ground-plane classes go through
  // here: inverse perspective assumes the point lies on the road, so an
  // elevated pole or sign would be placed at whatever range that assumption
  // implies, which is not where it is. Those classes are scored in the image
  // branch alone.
  std::vector<kitti::Polyline2D> bev_polylines;
  for (const auto& pl : perception.features) {
    if (!kitti::IsGroundPlaneType(pl.type)) continue;
    kitti::Polyline2D bev_pl;
    bev_pl.type = pl.type;
    for (const auto& uv : pl.points) {
      Vec3 p_rig;
      if (projection_->ImageToGroundRig(uv, p_rig) != Status::kOk) continue;
      int col = 0, row = 0;
      if (Projection::RigToBevPixel(p_rig, col, row) != Status::kOk) continue;
      // Pixel centres, not metres: RasterizePolylines works in raster
      // coordinates, and handing it metres drew every polyline into one corner
      // of the canvas.
      bev_pl.points.emplace_back(col + 0.5, row + 0.5);
    }
    if (bev_pl.points.size() >= 2) bev_polylines.push_back(std::move(bev_pl));
  }

  std::vector<uint8_t> binary;
  RasterizePolylineList(bev_polylines, out.width, out.height, 1.5f, binary,
                        out.labels);
#ifdef CAMLOC_CUDA_ENABLED
  if (params_.use_cuda && cuda::IsAvailable()) {
    return cuda::ComputeDistanceTransformGpu(binary, out.width, out.height,
                                             out.distance);
  }
#endif
  return DistanceTransformCpu::Compute(binary, out.width, out.height,
                                       out.distance);
}

float PoseSampler::SampleImageCost(const LabelledDistanceTransform& dt,
                                   const Vec2& uv, kitti::PolylineType type) {
  const uint8_t label =
      NearestLabel(dt.labels, dt.width, dt.height, uv.x(), uv.y());
  if (label != 0 && label != TypeToLabel(type)) {
    return dt.max_cost;
  }
  const float d =
      BilinearSample(dt.distance, dt.width, dt.height, uv.x(), uv.y());
  return std::min(d, dt.max_cost);
}

float PoseSampler::SampleBevCost(const LabelledDistanceTransform& dt,
                                 const Vec3& p_rig, kitti::PolylineType type) {
  int col = 0, row = 0;
  if (Projection::RigToBevPixel(p_rig, col, row) != Status::kOk) {
    return dt.max_cost;
  }
  const double u = col + 0.5;
  const double v = row + 0.5;
  return SampleImageCost(dt, Vec2(u, v), type);
}

Status PoseSampler::ComputeImageCosts(const kitti::MapChunk& map,
                                      const Mat44& T_world_plane,
                                      const LabelledDistanceTransform& dt,
                                      CostGrid& costs) const {
  if (projection_ == nullptr) return Status::kInvalidArgument;

#ifdef CAMLOC_CUDA_ENABLED
  if (params_.use_cuda && cuda::IsAvailable()) {
    std::vector<float> map_xyz;
    std::vector<uint8_t> map_labels;
    PackMapPoints(map, map_xyz, map_labels);
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
      if (cuda::ComputeImagePoseCostsGpu(
              T, map_xyz.data(), npts, map_labels.data(), dt.distance.data(),
              dt.labels.data(), gp, gpu_costs) == Status::kOk) {
        costs.data() = gpu_costs;
        return Status::kOk;
      }
    }
  }
#endif

  costs.Fill(dt.max_cost);
  const auto present = PresentChannels(map, /*ground_plane_only=*/false);

  // Brute-force CPU fallback: one thread-equivalent loop over all hypotheses
  for (int iw = 0; iw < costs.DimW(); ++iw) {
    for (int iy = 0; iy < costs.DimY(); ++iy) {
      for (int ix = 0; ix < costs.DimX(); ++ix) {
        const Vec3 offset = costs.IndexToOffset(ix, iy, iw);
        const Mat44 T_world_hyp =
            HypothesisPose(T_world_plane, offset.x(), offset.y(), offset.z());

        ClassCostSums sums{};
        ClassCounts counts{};
        for (const auto& pl : map.polylines) {
          const int ch = TypeToLabel(pl.type);
          for (const auto& p_world : pl.points) {
            const Vec3 p_rig = WorldToRig(T_world_hyp, p_world);
            if (p_rig.z() <= 0.5) continue;
            Vec2 uv;
            if (projection_->ProjectRigToImage(p_rig, uv) != Status::kOk)
              continue;
            sums[ch] += SampleImageCost(dt, uv, pl.type);
            ++counts[ch];
          }
        }
        costs.At(ix, iy, iw) = CombineClassCosts(
            sums, counts, present, params_.min_support_points, dt.max_cost);
      }
    }
  }
  return Status::kOk;
}

Status PoseSampler::ComputeBevCosts(const kitti::MapChunk& map,
                                    const Mat44& T_world_plane,
                                    const LabelledDistanceTransform& dt,
                                    CostGrid& costs) const {
  if (projection_ == nullptr) return Status::kInvalidArgument;

#ifdef CAMLOC_CUDA_ENABLED
  if (params_.use_cuda && cuda::IsAvailable()) {
    std::vector<float> map_xyz;
    std::vector<uint8_t> map_labels;
    PackMapPoints(map, map_xyz, map_labels);
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
      gp.bev_x_min = static_cast<float>(BevConfig::kForwardMinM);
      gp.bev_x_max = static_cast<float>(BevConfig::kForwardMaxM);
      gp.bev_y_min = static_cast<float>(BevConfig::kLeftMinM);
      gp.bev_y_max = static_cast<float>(BevConfig::kLeftMaxM);
      gp.bev_mpp_x = static_cast<float>(BevConfig::MetersPerPixelX());
      gp.bev_mpp_y = static_cast<float>(BevConfig::MetersPerPixelY());
      std::vector<float> gpu_costs;
      const int npts = static_cast<int>(map_labels.size());
      if (cuda::ComputeBevPoseCostsGpu(
              T, map_xyz.data(), npts, map_labels.data(), dt.distance.data(),
              dt.labels.data(), gp, gpu_costs) == Status::kOk) {
        costs.data() = gpu_costs;
        return Status::kOk;
      }
    }
  }
#endif

  costs.Fill(dt.max_cost);
  const auto present = PresentChannels(map, /*ground_plane_only=*/true);

  // CPU fallback: score hypotheses in the BEV raster (no pinhole projection)
  for (int iw = 0; iw < costs.DimW(); ++iw) {
    for (int iy = 0; iy < costs.DimY(); ++iy) {
      for (int ix = 0; ix < costs.DimX(); ++ix) {
        const Vec3 offset = costs.IndexToOffset(ix, iy, iw);
        const Mat44 T_world_hyp =
            HypothesisPose(T_world_plane, offset.x(), offset.y(), offset.z());

        ClassCostSums sums{};
        ClassCounts counts{};
        for (const auto& pl : map.polylines) {
          // Elevated classes have no place in a ground-plane raster; see
          // kitti::IsGroundPlaneType.
          if (!kitti::IsGroundPlaneType(pl.type)) continue;
          const int ch = TypeToLabel(pl.type);
          for (const auto& p_world : pl.points) {
            const Vec3 p_rig = WorldToRig(T_world_hyp, p_world);
            const Vec3 p_veh = Frames::ToVehicle(p_rig);
            if (p_veh.x() < BevConfig::kForwardMinM ||
                p_veh.x() > BevConfig::kForwardMaxM ||
                p_veh.y() < BevConfig::kLeftMinM ||
                p_veh.y() > BevConfig::kLeftMaxM) {
              continue;
            }
            sums[ch] += SampleBevCost(dt, p_rig, pl.type);
            ++counts[ch];
          }
        }
        costs.At(ix, iy, iw) = CombineClassCosts(
            sums, counts, present, params_.min_support_points, dt.max_cost);
      }
    }
  }
  return Status::kOk;
}

}  // namespace cam_loc::core
