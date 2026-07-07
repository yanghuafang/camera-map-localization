// Unit tests for Semantic KITTI label → perception conversion.
#include <cam_loc/semantic_kitti/preprocess.hpp>

#include <gtest/gtest.h>

TEST(SemanticKittiTest, LabelsToPerceptionRows) {
  constexpr int w = 20;
  constexpr int h = 10;
  std::vector<uint16_t> labels(static_cast<size_t>(w * h), 0);
  for (int x = 5; x < 15; ++x) {
    labels[static_cast<size_t>(3 * w + x)] = cam_loc::semantic_kitti::kLaneMarking;
  }

  cam_loc::semantic_kitti::PreprocessOptions opts;
  opts.frame = 7;
  opts.image_width = w;
  opts.image_height = h;
  opts.row_stride = 1;
  opts.min_run_length = 4;

  cam_loc::kitti::FramePerception perception;
  ASSERT_EQ(cam_loc::semantic_kitti::labelsToPerception(labels, w, h, opts, perception),
            cam_loc::Status::kOk);
  EXPECT_EQ(perception.frame, 7);
  EXPECT_GE(perception.lane_lines.size(), 1u);
}
