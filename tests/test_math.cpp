// Unit tests for SE(3) math helpers and sequence ID formatting.
#include <cam_loc/types/status.hpp>

#include <gtest/gtest.h>

TEST(MathTest, RelativeTransformIdentity) {
  cam_loc::Mat44 T = cam_loc::Mat44::Identity();
  T(0, 3) = 1.0;
  const cam_loc::Mat44 rel = cam_loc::relativeTransform(T, T);
  EXPECT_NEAR((rel - cam_loc::Mat44::Identity()).norm(), 0.0, 1e-9);
}

TEST(MathTest, FormatSequenceId) {
  EXPECT_EQ(cam_loc::formatSequenceId(0), "00");
  EXPECT_EQ(cam_loc::formatSequenceId(9), "09");
  EXPECT_EQ(cam_loc::formatSequenceId(10), "10");
}
