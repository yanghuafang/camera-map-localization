// Unit tests for CUDA DT, aggregation, and pose-grid kernels vs CPU.
#include <cam_loc/cuda/distance_transform.hpp>
#include <cam_loc/core/cost_aggregator.hpp>
#include <cam_loc/core/cost_grid.hpp>
#include <cam_loc/core/distance_transform_cpu.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace {

void fillDt(int w, int h, std::vector<float>& dist, std::vector<uint8_t>& labels) {
  dist.assign(static_cast<size_t>(w * h), 10.f);
  labels.assign(static_cast<size_t>(w * h), 0);
  const int cx = w / 2;
  const int cy = h / 2;
  dist[static_cast<size_t>(cy * w + cx)] = 0.f;
  labels[static_cast<size_t>(cy * w + cx)] = 1;
}

}  // namespace

TEST(CudaTest, ArgminMatchesCpu) {
  if (!cam_loc::cuda::isAvailable()) {
    GTEST_SKIP() << "No CUDA device";
  }
  std::vector<float> costs = {4.f, 1.f, 9.f, 2.f, 7.f};
  int idx = -1;
  float vmin = 0.f;
  ASSERT_EQ(cam_loc::cuda::argminGpu(costs, idx, vmin), cam_loc::Status::kOk);
  EXPECT_EQ(idx, 1);
  EXPECT_FLOAT_EQ(vmin, 1.f);
}

TEST(CudaTest, ImagePoseCostsCenterHypothesis) {
  if (!cam_loc::cuda::isAvailable()) {
    GTEST_SKIP() << "No CUDA device";
  }
  constexpr int w = 64;
  constexpr int h = 48;
  std::vector<float> dist;
  std::vector<uint8_t> labels;
  fillDt(w, h, dist, labels);

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
  gp.cx = w / 2.f;
  gp.cy = h / 2.f;
  gp.dt_max_cost = 10.f;
  gp.dt_width = w;
  gp.dt_height = h;

  std::vector<float> costs;
  ASSERT_EQ(cam_loc::cuda::computeImagePoseCostsGpu(T, map_xyz, 1, map_labels, dist.data(),
                                                    labels.data(), gp, costs),
            cam_loc::Status::kOk);
  ASSERT_EQ(costs.size(), 27u);

  int center = 1 + 3 + 3 * 3 * 1;  // ix=1, iy=1, iw=1 in row-major x,y,yaw
  center = 1 * (gp.num_y * gp.num_yaw) + 1 * gp.num_yaw + 1;
  EXPECT_LT(costs[static_cast<size_t>(center)], costs[0]);
}

TEST(CudaTest, DistanceTransformMatchesCpu) {
  if (!cam_loc::cuda::isAvailable()) {
    GTEST_SKIP() << "No CUDA device";
  }
  constexpr int w = 32;
  constexpr int h = 24;
  std::vector<uint8_t> binary(w * h, 255);
  binary[12 * w + 16] = 0;
  binary[12 * w + 17] = 0;

  std::vector<float> cpu_dt;
  ASSERT_EQ(cam_loc::core::DistanceTransformCpu::compute(binary, w, h, cpu_dt),
            cam_loc::Status::kOk);

  std::vector<float> gpu_dt;
  ASSERT_EQ(cam_loc::cuda::computeDistanceTransformGpu(binary, w, h, gpu_dt), cam_loc::Status::kOk);
  ASSERT_EQ(cpu_dt.size(), gpu_dt.size());
  for (size_t i = 0; i < cpu_dt.size(); ++i) {
    EXPECT_NEAR(cpu_dt[i], gpu_dt[i], 0.15f) << "index " << i;
  }
}

TEST(CudaTest, AggregateMatchesCpu) {
  if (!cam_loc::cuda::isAvailable()) {
    GTEST_SKIP() << "No CUDA device";
  }

  cam_loc::SamplingGridParams gp;
  gp.num_x = 5;
  gp.num_y = 5;
  gp.num_yaw = 3;

  cam_loc::core::CostGrid frame0(gp);
  for (int iw = 0; iw < frame0.dimW(); ++iw) {
    for (int iy = 0; iy < frame0.dimY(); ++iy) {
      for (int ix = 0; ix < frame0.dimX(); ++ix) {
        const cam_loc::Vec3 off = frame0.indexToOffset(ix, iy, iw);
        frame0.at(ix, iy, iw) = static_cast<float>(off.x() * off.x() + off.y() * off.y());
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
  agg_cpu.pushHistory(frame0, T0, 0);

  cam_loc::core::CostGrid cpu_current(gp);
  cpu_current.fill(2.f);
  cam_loc::core::CostGrid gpu_current(gp);
  gpu_current.data() = cpu_current.data();

  ASSERT_EQ(agg_cpu.aggregate(cpu_current, T1, 0.1, false), cam_loc::Status::kOk);

  cam_loc::core::CostAggregator agg_gpu(ap);
  agg_gpu.pushHistory(frame0, T0, 0);
  ASSERT_EQ(agg_gpu.aggregate(gpu_current, T1, 0.1, true), cam_loc::Status::kOk);

  ASSERT_EQ(cpu_current.data().size(), gpu_current.data().size());
  for (size_t i = 0; i < cpu_current.data().size(); ++i) {
    EXPECT_NEAR(cpu_current.data()[i], gpu_current.data()[i], 1e-3f) << "index " << i;
  }
}
