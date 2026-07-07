#pragma once

#include <cam_loc/types/params.hpp>
#include <cam_loc/types/status.hpp>

#include <vector>

namespace cam_loc::core {

/// 3-DOF cost volume over pose offsets in the sampling plane.
///
/// Each cell stores the mean DT cost of projecting map points under a hypothesis
/// T_world_hyp = T_world_plane · offsetToTransform(x, y, yaw).
///
/// Indexing (row-major in yaw):
///   linearIndex(ix, iy, iw) = iw·dimX·dimY + iy·dimX + ix
/// Center cell (ix, iy, iw) = (nx, ny, nw) ↔ zero offset; negative indices are
/// left/back, positive are right/forward relative to the plane pose.
class CostGrid {
 public:
  explicit CostGrid(const SamplingGridParams& params);

  int nx() const { return nx_; }
  int ny() const { return ny_; }
  int nw() const { return nw_; }
  int dimX() const { return 2 * nx_ + 1; }
  int dimY() const { return 2 * ny_ + 1; }
  int dimW() const { return 2 * nw_ + 1; }
  int size() const { return static_cast<int>(costs_.size()); }

  double stepX() const { return step_x_; }
  double stepY() const { return step_y_; }
  double stepYaw() const { return step_yaw_; }

  float& at(int ix, int iy, int iw);
  float at(int ix, int iy, int iw) const;
  int linearIndex(int ix, int iy, int iw) const;

  /// Discrete grid index → continuous offset (m, m, rad) relative to plane pose.
  Vec3 indexToOffset(int ix, int iy, int iw) const;
  void offsetToNearestIndex(const Vec3& offset, int& ix, int& iy, int& iw) const;

  /// Trilinear-ish sample at continuous offset (x_m, y_m, yaw_rad) in plane frame.
  float sampleContinuous(double x_m, double y_m, double yaw_rad) const;

  const std::vector<float>& data() const { return costs_; }
  std::vector<float>& data() { return costs_; }

  void fill(float value);

  struct ArgMinResult {
    int ix = 0;
    int iy = 0;
    int iw = 0;
    float cost = 0.f;
  };
  ArgMinResult argmin(bool use_gpu = false) const;

 private:
  int nx_{10};
  int ny_{15};
  int nw_{6};
  double step_x_{0.5};
  double step_y_{0.5};
  double step_yaw_{0.00872665};
  std::vector<float> costs_;
};

}  // namespace cam_loc::core
