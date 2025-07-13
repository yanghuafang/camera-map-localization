#ifndef CAM_LOC_CORE_COST_GRID_H_
#define CAM_LOC_CORE_COST_GRID_H_

#include <vector>

#include "cam_loc/types/params.h"
#include "cam_loc/types/status.h"

namespace cam_loc::core {

/// 3-DOF cost volume over pose offsets in the sampling plane.
///
/// Each cell stores the mean DT cost of projecting map points under a
/// hypothesis T_world_hyp = T_world_plane · OffsetToTransform(x, y, yaw).
///
/// Indexing (row-major in yaw):
///   LinearIndex(ix, iy, iw) = iw·DimX·DimY + iy·DimX + ix
/// Center cell (ix, iy, iw) = (nx, ny, nw) ↔ zero offset; negative indices are
/// left/back, positive are right/forward relative to the plane pose.
class CostGrid {
 public:
  explicit CostGrid(const SamplingGridParams& params);

  int nx() const { return nx_; }
  int ny() const { return ny_; }
  int nw() const { return nw_; }
  int DimX() const { return 2 * nx_ + 1; }
  int DimY() const { return 2 * ny_ + 1; }
  int DimW() const { return 2 * nw_ + 1; }
  int Size() const { return static_cast<int>(costs_.size()); }

  double step_x() const { return step_x_; }
  double step_y() const { return step_y_; }
  double step_yaw() const { return step_yaw_; }

  /// @note Indices are not bounds-checked; they are expected to come from the
  ///       Dim*() loops or from OffsetToNearestIndex, which clamps.
  float& At(int ix, int iy, int iw);
  float At(int ix, int iy, int iw) const;
  int LinearIndex(int ix, int iy, int iw) const;

  /// Discrete grid index → continuous offset relative to the plane pose.
  ///
  /// @return `(x_m, y_m, yaw_rad)` — a packed offset, not a point.
  Vec3 IndexToOffset(int ix, int iy, int iw) const;

  /// Inverse of IndexToOffset, rounded to the nearest cell.
  ///
  /// @param offset `(x_m, y_m, yaw_rad)`; values outside the grid are clamped
  ///        to the border rather than rejected.
  void OffsetToNearestIndex(const Vec3& offset, int& ix, int& iy,
                            int& iw) const;

  /// Trilinear sample at a fractional offset in the plane frame.
  ///
  /// @return Interpolated cost. Offsets outside the grid read the border value
  ///         instead of extrapolating, which is what lets temporal aggregation
  ///         sample a history cell whose warped offset has left the grid.
  float SampleContinuous(double x_m, double y_m, double yaw_rad) const;

  const std::vector<float>& data() const { return costs_; }
  std::vector<float>& data() { return costs_; }

  void Fill(float value);

  /// Lowest-cost cell and its cost.
  struct ArgMinResult {
    int ix = 0;
    int iy = 0;
    int iw = 0;
    float cost = 0.f;
  };

  /// @param use_gpu Try the CUDA reduction first; falls back to the CPU scan
  ///        when CUDA is unavailable or the kernel fails.
  ArgMinResult Argmin(bool use_gpu = false) const;

  /// Offset of the minimum, refined below the cell pitch.
  ///
  /// The grid step bounds how precisely the argmin alone can place the pose --
  /// at the default half-metre pitch the answer is quantized to half a metre,
  /// which shows up as a sawtooth in the trajectory error. Fitting a parabola
  /// through the minimum and its two neighbours along each axis recovers the
  /// sub-cell position, which is what the cost surface was already telling us.
  ///
  /// Falls back to the cell centre per axis when the minimum sits on a border,
  /// or when the three samples are collinear and the fit has no vertex.
  Vec3 RefinedOffset(const ArgMinResult& argmin) const;

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

#endif  // CAM_LOC_CORE_COST_GRID_H_
