// Unit tests for CPU Felzenszwalb distance transform.
#include <cam_loc/core/distance_transform_cpu.hpp>

#include <gtest/gtest.h>

TEST(DistanceTransformTest, PointFeature) {
  constexpr int w = 8;
  constexpr int h = 8;
  std::vector<uint8_t> binary(w * h, 255);
  binary[4 * w + 4] = 0;

  std::vector<float> dt;
  ASSERT_EQ(cam_loc::core::DistanceTransformCpu::compute(binary, w, h, dt),
            cam_loc::Status::kOk);
  EXPECT_NEAR(dt[4 * w + 4], 0.f, 1e-3f);
  EXPECT_GT(dt[0], 0.f);
  EXPECT_NEAR(dt[4 * w + 5], 1.f, 0.5f);
}

TEST(DistanceTransformTest, RasterizeLine) {
  std::vector<cam_loc::Vec2> pts{{10, 10}, {20, 10}};
  std::vector<uint8_t> binary;
  ASSERT_EQ(cam_loc::core::DistanceTransformCpu::rasterizePolylines(pts, 32, 32, 1.f, binary),
            cam_loc::Status::kOk);
  EXPECT_EQ(binary[10 * 32 + 15], 0);
}
