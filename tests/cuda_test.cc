// Unit tests for CUDA DT, aggregation, and pose-grid kernels vs CPU.
#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cam_loc/core/cost_aggregator.h"
#include "cam_loc/core/cost_grid.h"
#include "cam_loc/core/distance_transform_cpu.h"
#include "cam_loc/cuda/distance_transform.h"

namespace {

void FillDt(int w, int h, std::vector<float>& dist,
            std::vector<uint8_t>& labels) {
  dist.assign(static_cast<size_t>(w * h), 10.f);
  labels.assign(static_cast<size_t>(w * h), 0);
  const int cx = w / 2;
  const int cy = h / 2;
  dist[static_cast<size_t>(cy * w + cx)] = 0.f;
  labels[static_cast<size_t>(cy * w + cx)] = 1;
}

}  // namespace

TEST(CudaTest, ReportsTheDeviceItRunsOn) {
  if (!cam_loc::cuda::IsAvailable()) {
    GTEST_SKIP() << "No CUDA device";
  }
  // Which card is device 0 is a property of the host, so there is nothing to
  // assert it against. What matters is that the parity results below name the
  // hardware that produced them: "the CUDA tests pass" says little on its own,
  // and the device index would not say it either -- 0 is a different card on
  // every machine.
  const std::string name = cam_loc::cuda::DeviceName();
  EXPECT_FALSE(name.empty());
  GTEST_LOG_(INFO) << "CUDA parity tests running on: " << name;
}

TEST(CudaTest, ArgminMatchesCpu) {
  if (!cam_loc::cuda::IsAvailable()) {
    GTEST_SKIP() << "No CUDA device";
  }
  std::vector<float> costs = {4.f, 1.f, 9.f, 2.f, 7.f};
  int idx = -1;
  float vmin = 0.f;
  ASSERT_EQ(cam_loc::cuda::ArgminGpu(costs, idx, vmin), cam_loc::Status::kOk);
  EXPECT_EQ(idx, 1);
  EXPECT_FLOAT_EQ(vmin, 1.f);
}

TEST(CudaTest, ImagePoseCostsCenterHypothesis) {
  if (!cam_loc::cuda::IsAvailable()) {
    GTEST_SKIP() << "No CUDA device";
  }
  constexpr int kW = 64;
  constexpr int kH = 48;
  std::vector<float> dist;
  std::vector<uint8_t> labels;
  FillDt(kW, kH, dist, labels);

  // Identity plane pose; one map point straight ahead in rig +Z.
  float T[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 10, 1};
  float map_xyz[3] = {0.f, 0.f, 20.f};
  uint8_t map_labels[1] = {1};

  cam_loc::cuda::PoseCostGpuParams gp;
  gp.num_x = 3;
  gp.num_y = 3;
  gp.num_yaw = 3;
  gp.step_x_m = 1.0;
  gp.step_y_m = 1.0;
  gp.step_yaw_deg = 5.0;
  gp.fx = 500.f;
  gp.fy = 500.f;
  gp.cx = kW / 2.f;
  gp.cy = kH / 2.f;
  gp.dt_max_cost = 10.f;
  gp.dt_width = kW;
  gp.dt_height = kH;

  std::vector<float> costs;
  ASSERT_EQ(
      cam_loc::cuda::ComputeImagePoseCostsGpu(
          T, map_xyz, 1, map_labels, dist.data(), labels.data(), gp, costs),
      cam_loc::Status::kOk);
  ASSERT_EQ(costs.size(), 27u);

  int center = 1 + 3 + 3 * 3 * 1;  // ix=1, iy=1, iw=1 in row-major x,y,yaw
  center = 1 * (gp.num_y * gp.num_yaw) + 1 * gp.num_yaw + 1;
  EXPECT_LT(costs[static_cast<size_t>(center)], costs[0]);
}

TEST(CudaTest, DistanceTransformMatchesCpu) {
  if (!cam_loc::cuda::IsAvailable()) {
    GTEST_SKIP() << "No CUDA device";
  }
  constexpr int kW = 32;
  constexpr int kH = 24;
  std::vector<uint8_t> binary(kW * kH, 255);
  binary[12 * kW + 16] = 0;
  binary[12 * kW + 17] = 0;

  std::vector<float> cpu_dt;
  ASSERT_EQ(
      cam_loc::core::DistanceTransformCpu::Compute(binary, kW, kH, cpu_dt),
      cam_loc::Status::kOk);

  std::vector<float> gpu_dt;
  ASSERT_EQ(cam_loc::cuda::ComputeDistanceTransformGpu(binary, kW, kH, gpu_dt),
            cam_loc::Status::kOk);
  ASSERT_EQ(cpu_dt.size(), gpu_dt.size());
  for (size_t i = 0; i < cpu_dt.size(); ++i) {
    EXPECT_NEAR(cpu_dt[i], gpu_dt[i], 0.15f) << "index " << i;
  }
}

TEST(CudaTest, AggregateMatchesCpu) {
  if (!cam_loc::cuda::IsAvailable()) {
    GTEST_SKIP() << "No CUDA device";
  }

  cam_loc::SamplingGridParams gp;
  gp.num_x = 5;
  gp.num_y = 5;
  gp.num_yaw = 3;

  cam_loc::core::CostGrid frame0(gp);
  for (int iw = 0; iw < frame0.DimW(); ++iw) {
    for (int iy = 0; iy < frame0.DimY(); ++iy) {
      for (int ix = 0; ix < frame0.DimX(); ++ix) {
        const cam_loc::Vec3 off = frame0.IndexToOffset(ix, iy, iw);
        frame0.At(ix, iy, iw) =
            static_cast<float>(off.x() * off.x() + off.y() * off.y());
      }
    }
  }

  cam_loc::Mat44 T0 = cam_loc::Mat44::Identity();
  cam_loc::Mat44 T1 = cam_loc::Mat44::Identity();
  T1(0, 3) = 0.3;

  cam_loc::AggregationParams ap;
  ap.window_size = 4;
  ap.distance_decay = 0.01f;

  cam_loc::core::CostAggregator agg_cpu(ap);
  agg_cpu.PushHistory(frame0, T0, 0, 0.0);

  cam_loc::core::CostGrid cpu_current(gp);
  cpu_current.Fill(2.f);
  cam_loc::core::CostGrid gpu_current(gp);
  gpu_current.data() = cpu_current.data();

  ASSERT_EQ(agg_cpu.Aggregate(cpu_current, T1, 0.1, false),
            cam_loc::Status::kOk);

  cam_loc::core::CostAggregator agg_gpu(ap);
  agg_gpu.PushHistory(frame0, T0, 0, 0.0);
  ASSERT_EQ(agg_gpu.Aggregate(gpu_current, T1, 0.1, true),
            cam_loc::Status::kOk);

  ASSERT_EQ(cpu_current.data().size(), gpu_current.data().size());
  for (size_t i = 0; i < cpu_current.data().size(); ++i) {
    EXPECT_NEAR(cpu_current.data()[i], gpu_current.data()[i], 1e-3f)
        << "index " << i;
  }
}
