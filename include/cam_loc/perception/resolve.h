#ifndef CAM_LOC_PERCEPTION_RESOLVE_H_
#define CAM_LOC_PERCEPTION_RESOLVE_H_

/// Per-frame perception selection: load JSON, synthesize oracle lanes, or apply
/// noise.

#include "cam_loc/kitti/types.h"
#include "cam_loc/map/map_loader.h"
#include "cam_loc/perception/noise.h"
#include "cam_loc/types/status.h"

namespace cam_loc::core {
class Projection;
}

namespace cam_loc::perception {

/// Where a frame's perception comes from.
///
/// The oracle projects the map at the **ground-truth** pose. That distinction
/// is the whole value of it: projecting from the filter's own estimate instead
/// would make the observation follow the estimate, and the match would report
/// success no matter how far off the estimate had drifted.
enum class PerceptionSource {
  /// JSON when present, oracle otherwise.
  kAuto,
  /// JSON only; an empty frame is a valid result.
  kFile,
  /// Oracle only: the map projected at the ground-truth pose.
  kOracle,
  /// kAuto, then seeded noise on top.
  kNoisy,
};

/// Which stages ran inside `ResolvePerception` (for logging and benchmarks).
struct PerceptionResolveInfo {
  bool loaded_from_file = false;
  bool synthesized = false;
  bool noise_applied = false;
};

/// Build the perception fed into map matching for one frame.
Status ResolvePerception(
    PerceptionSource source, const std::string& perception_root, int sequence,
    int frame, const map::IMapLoader& map_loader, double map_radius_m,
    const core::Projection& projection, const Mat44& T_world_gt,
    const PerceptionNoiseParams& noise, uint32_t noise_seed,
    kitti::FramePerception& out, PerceptionResolveInfo& info);

}  // namespace cam_loc::perception

#endif  // CAM_LOC_PERCEPTION_RESOLVE_H_
