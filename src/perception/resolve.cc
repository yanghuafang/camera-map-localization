// Per-frame perception resolution: file load, oracle synthesis, or noisy
// corruption paths.

#include "cam_loc/perception/resolve.h"

#include "cam_loc/core/projection.h"
#include "cam_loc/perception/adapter.h"
#include "cam_loc/perception/synthesize.h"

namespace cam_loc::perception {

namespace {

Status SynthesizeAt(const map::IMapLoader& map_loader, double map_radius_m,
                    const core::Projection& projection, const Mat44& T_world,
                    int frame, kitti::FramePerception& out) {
  kitti::MapChunk local;
  if (map_loader.QueryLocalMap(T_world, map_radius_m, local) != Status::kOk ||
      local.polylines.empty()) {
    return Status::kInvalidArgument;
  }
  out = SynthesizeFromMap(local, projection, T_world, frame);
  return !out.empty() ? Status::kOk : Status::kInvalidArgument;
}

}  // namespace

Status ResolvePerception(
    PerceptionSource source, const std::string& perception_root, int sequence,
    int frame, const map::IMapLoader& map_loader, double map_radius_m,
    const core::Projection& projection, const Mat44& T_world_gt,
    const PerceptionNoiseParams& noise, uint32_t noise_seed,
    kitti::FramePerception& out, PerceptionResolveInfo& info) {
  info = PerceptionResolveInfo{};
  out = kitti::FramePerception{};
  out.frame = frame;

  // Oracle: always project local map at GT pose (no file read).
  if (source == PerceptionSource::kOracle) {
    const Status st = SynthesizeAt(map_loader, map_radius_m, projection,
                                   T_world_gt, frame, out);
    info.synthesized = st == Status::kOk;
    return st;
  }

  if (!perception_root.empty()) {
    LoadFramePerception(perception_root, sequence, frame, out);
    info.loaded_from_file = !out.empty();
  }

  // File: return whatever was loaded, empty included. A detector that found
  // nothing is a valid frame, not a failure.
  if (source == PerceptionSource::kFile) {
    return Status::kOk;
  }

  // Auto: files when there are files, the oracle otherwise. The oracle projects
  // the map at the ground-truth pose, so it is an upper bound on the backend
  // and is reported as synthesized -- never as a detection.
  if (source == PerceptionSource::kAuto) {
    if (!out.empty()) return Status::kOk;
    const Status st = SynthesizeAt(map_loader, map_radius_m, projection,
                                   T_world_gt, frame, out);
    info.synthesized = st == Status::kOk;
    return st;
  }

  // kNoisy: synthesize if needed, then apply configured noise.
  if (out.empty()) {
    const Status st = SynthesizeAt(map_loader, map_radius_m, projection,
                                   T_world_gt, frame, out);
    if (st != Status::kOk) return st;
    info.synthesized = true;
  }

  if (noise.Enabled()) {
    out = AddPerceptionNoise(out, noise, noise_seed);
    info.noise_applied = true;
  }
  return !out.empty() ? Status::kOk : Status::kInvalidArgument;
}

}  // namespace cam_loc::perception
