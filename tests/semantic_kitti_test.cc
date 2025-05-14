// Label raster → landmarks: one scan orientation per class shape.
#include <gtest/gtest.h>

#include "cam_loc/semantic_kitti/preprocess.h"

namespace {

namespace sk = cam_loc::semantic_kitti;
using cam_loc::kitti::PolylineType;

constexpr int kW = 64;
constexpr int kH = 48;

/// A raster with a road band, a lane marking along it, a pole and a sign.
std::vector<uint16_t> SyntheticScene() {
  std::vector<uint16_t> labels(static_cast<size_t>(kW * kH), 0);
  auto set = [&](int x, int y, uint16_t v) {
    labels[static_cast<size_t>(y) * kW + x] = v;
  };

  // Road occupies columns 10..49 over the lower half of the image.
  for (int y = 24; y < kH; ++y) {
    for (int x = 10; x < 50; ++x) set(x, y, sk::kRoad);
    // A lane marking down the middle of it.
    for (int x = 28; x < 32; ++x) set(x, y, sk::kLaneMarking);
  }
  // A pole: two columns, tall.
  for (int y = 4; y < 30; ++y) {
    set(52, y, sk::kPole);
    set(53, y, sk::kPole);
  }
  // A sign: a small upright patch.
  for (int y = 6; y < 20; ++y) {
    for (int x = 4; x < 8; ++x) set(x, y, sk::kTrafficSign);
  }
  return labels;
}

sk::PreprocessOptions Options() {
  sk::PreprocessOptions opts;
  opts.frame = 7;
  opts.image_width = kW;
  opts.image_height = kH;
  opts.scan_stride = 1;
  opts.min_run_length = 4;
  return opts;
}

}  // namespace

TEST(SemanticKittiTest, LabelsToPerceptionRows) {
  cam_loc::kitti::FramePerception p;
  ASSERT_EQ(sk::LabelsToPerception(SyntheticScene(), kW, kH, Options(), p),
            cam_loc::Status::kOk);
  EXPECT_EQ(p.frame, 7);
  EXPECT_GE(p.CountOf(PolylineType::kLaneSolid), 1);
}

TEST(SemanticKittiTest, RejectsMismatchedRasterSize) {
  cam_loc::kitti::FramePerception p;
  EXPECT_EQ(sk::LabelsToPerception(SyntheticScene(), kW, kH + 1, Options(), p),
            cam_loc::Status::kInvalidArgument);
}

// Poles and traffic signs are the classes that pin along-track position, and
// they are upright: a horizontal scan meets a pole one or two pixels at a time
// and discards it as too short. They need the column scan.
TEST(SemanticKittiTest, ExtractsUprightLandmarks) {
  cam_loc::kitti::FramePerception p;
  ASSERT_EQ(sk::LabelsToPerception(SyntheticScene(), kW, kH, Options(), p),
            cam_loc::Status::kOk);

  EXPECT_GE(p.CountOf(PolylineType::kPole), 1) << "pole class not extracted";
  EXPECT_GE(p.CountOf(PolylineType::kSign), 1) << "sign class not extracted";

  for (const auto& pl : p.features) {
    if (pl.type != PolylineType::kPole) continue;
    // A column run: constant x, and taller than it is wide.
    EXPECT_DOUBLE_EQ(pl.points.front().x(), pl.points.back().x());
    EXPECT_GT(std::abs(pl.points.back().y() - pl.points.front().y()), 4.0);
  }
}

// The boundary is where the drivable surface ends. Tracing runs of sidewalk or
// terrain instead followed the far side of the kerb, and typed them from
// whatever pixel happened to terminate the run.
TEST(SemanticKittiTest, RoadBoundariesFollowTheEdgeOfTheRoad) {
  cam_loc::kitti::FramePerception p;
  ASSERT_EQ(sk::LabelsToPerception(SyntheticScene(), kW, kH, Options(), p),
            cam_loc::Status::kOk);

  ASSERT_EQ(p.CountOf(PolylineType::kRoadEdge), 2) << "expected two edges";
  for (const auto& pl : p.features) {
    if (pl.type != PolylineType::kRoadEdge) continue;
    const double x = pl.points.front().x();
    EXPECT_TRUE(x == 10.0 || x == 49.0)
        << "edge at x=" << x << ", not at a road boundary";
  }
}
