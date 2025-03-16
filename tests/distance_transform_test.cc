// Unit tests for CPU Felzenszwalb distance transform.
#include <gtest/gtest.h>

#include "cam_loc/core/distance_transform_cpu.h"

TEST(DistanceTransformTest, PointFeature) {
  constexpr int kW = 8;
  constexpr int kH = 8;
  std::vector<uint8_t> binary(kW * kH, 255);
  binary[4 * kW + 4] = 0;

  std::vector<float> dt;
  ASSERT_EQ(cam_loc::core::DistanceTransformCpu::Compute(binary, kW, kH, dt),
            cam_loc::Status::kOk);
  EXPECT_NEAR(dt[4 * kW + 4], 0.f, 1e-3f);
  EXPECT_GT(dt[0], 0.f);
  EXPECT_NEAR(dt[4 * kW + 5], 1.f, 0.5f);
}

TEST(DistanceTransformTest, RasterizeLine) {
  std::vector<cam_loc::Vec2> pts{{10, 10}, {20, 10}};
  std::vector<uint8_t> binary;
  ASSERT_EQ(cam_loc::core::DistanceTransformCpu::RasterizePolylines(
                pts, 32, 32, 1.f, binary),
            cam_loc::Status::kOk);
  EXPECT_EQ(binary[10 * 32 + 15], 0);
}
