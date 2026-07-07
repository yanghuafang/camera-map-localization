#pragma once

/// Per-frame perception selection: load JSON, synthesize oracle lanes, or apply noise.

#include <cam_loc/kitti/types.hpp>
#include <cam_loc/map/map_loader.hpp>
#include <cam_loc/perception/noise.hpp>
#include <cam_loc/types/status.hpp>

namespace cam_loc::core {
class Projection;
}

namespace cam_loc::perception {

enum class PerceptionSource {
  /// Load JSON when present; otherwise leave empty for engine fallback.
  kAuto,
  /// Load JSON only (may be empty).
  kFile,
  /// Synthesize from map at GT pose (oracle lanes).
  kOracle,
  /// Load or synthesize, then apply noise.
  kNoisy,
};

/// Which stages ran inside `resolvePerception` (for logging and benchmarks).
struct PerceptionResolveInfo {
  bool loaded_from_file = false;
  bool synthesized = false;
  bool noise_applied = false;
};

/// Build the perception fed into map matching for one frame.
Status resolvePerception(PerceptionSource source, const std::string& perception_root, int sequence,
                         int frame, const map::IMapLoader& map_loader, double map_radius_m,
                         const core::Projection& projection, const Mat44& T_world_gt,
                         const PerceptionNoiseParams& noise, uint32_t noise_seed,
                         kitti::FramePerception& out, PerceptionResolveInfo& info);

}  // namespace cam_loc::perception
