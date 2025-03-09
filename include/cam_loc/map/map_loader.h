#ifndef CAM_LOC_MAP_MAP_LOADER_H_
#define CAM_LOC_MAP_MAP_LOADER_H_

/// Map data access for localization: load polylines and query a local
/// neighborhood per frame.

#include <string>
#include <vector>

#include "cam_loc/kitti/types.h"
#include "cam_loc/types/status.h"

namespace cam_loc::map {

/// How far the stored geometry is expected to be from the world, vehicle axes.
///
/// A map is a survey, and a survey has an error bar. A filter handed only the
/// sharpness of its cost surface is precise about a landmark whose stored
/// position is wrong, which does not show up as a worse pose -- it shows up as
/// a covariance that claims more than the estimate can support.
struct MapUncertainty {
  double lateral_sigma_m = 0.0;
  double longitudinal_sigma_m = 0.0;
  /// Heading, from the survey error's gradient along the route rather than from
  /// the offset itself: a map displaced bodily is not rotated, one displaced by
  /// a differing amount at each end is.
  double yaw_sigma_rad = 0.0;

  bool Enabled() const {
    return lateral_sigma_m > 0.0 || longitudinal_sigma_m > 0.0 ||
           yaw_sigma_rad > 0.0;
  }
};

/// Abstract map source: load polylines once, query a local neighborhood per
/// frame.
class IMapLoader {
 public:
  virtual ~IMapLoader() = default;

  /// Populate the full map from a JSON file.
  ///
  /// Points are world-frame `[x, y, z]` by default, or WGS84
  /// `[lat, lon, alt]` when the polyline sets `"coord_frame": "wgs84"` and the
  /// file carries a top-level `"georef"` block.
  ///
  /// @return `kIoError` if the file cannot be opened, `kInvalidArgument` if it
  ///         is malformed or yields no polyline with at least two points.
  virtual Status LoadFromJsonFile(const std::string& path) = 0;

  /// Return the map near a pose, trimmed point by point.
  ///
  /// @param T_world_rig Query pose; only its translation is used.
  /// @param radius_m    3-D radius about that translation.
  /// @param out         Cleared, then filled with the polylines that keep at
  ///                    least two points inside the radius. A polyline that
  ///                    enters and leaves the radius is returned as a single
  ///                    polyline with the gap closed, not split in two.
  virtual Status QueryLocalMap(const Mat44& T_world_rig, double radius_m,
                               kitti::MapChunk& out) const = 0;

  /// The geometry a camera would actually see, which perception is cut from.
  ///
  /// A stored map is a *survey* of the world, not the world, and the two differ
  /// by the survey's error. That difference is the whole of what map matching
  /// has to overcome, so an evaluation that cuts perception from the map itself
  /// cannot measure map matching at all: the error appears identically on both
  /// sides, cancels, and the true pose is recovered exactly however wrong the
  /// map is.
  ///
  /// Defaults to `*this`, which is right for a real map -- there is no second
  /// source of truth available, and perception comes from a detector rather
  /// than from projecting geometry. Only a synthetic map, which generated the
  /// world before it surveyed it, can override this.
  virtual const IMapLoader& world() const { return *this; }

  /// What this map's geometry is worth, for the measurement covariance.
  ///
  /// Zero by default: a loaded map carries no statement of its own accuracy,
  /// and inventing one would be worse than admitting the gap. A synthetic map
  /// knows exactly, because it applied the error itself.
  virtual MapUncertainty uncertainty() const { return {}; }
};

}  // namespace cam_loc::map

#endif  // CAM_LOC_MAP_MAP_LOADER_H_
