// Per-frame perception resolution: file load, oracle synthesis, or noisy corruption paths.

#include <cam_loc/perception/resolve.hpp>

#include <cam_loc/core/projection.hpp>
#include <cam_loc/perception/adapter.hpp>
#include <cam_loc/perception/synthesize.hpp>

namespace cam_loc::perception {

namespace {

bool hasGeometry(const kitti::FramePerception& p) {
  return !p.lane_lines.empty() || !p.road_boundaries.empty();
}

Status synthesizeAt(const map::IMapLoader& map_loader, double map_radius_m,
                    const core::Projection& projection, const Mat44& T_world, int frame,
                    kitti::FramePerception& out) {
  kitti::MapChunk local;
  if (map_loader.queryLocalMap(T_world, map_radius_m, local) != Status::kOk ||
      local.polylines.empty()) {
    return Status::kInvalidArgument;
  }
  out = synthesizeFromMap(local, projection, T_world, frame);
  return hasGeometry(out) ? Status::kOk : Status::kInvalidArgument;
}

}  // namespace

Status resolvePerception(PerceptionSource source, const std::string& perception_root, int sequence,
                         int frame, const map::IMapLoader& map_loader, double map_radius_m,
                         const core::Projection& projection, const Mat44& T_world_gt,
                         const PerceptionNoiseParams& noise, uint32_t noise_seed,
                         kitti::FramePerception& out, PerceptionResolveInfo& info) {
  info = PerceptionResolveInfo{};
  out = kitti::FramePerception{};
  out.frame = frame;

  // Oracle: always project local map at GT pose (no file read).
  if (source == PerceptionSource::kOracle) {
    const Status st = synthesizeAt(map_loader, map_radius_m, projection, T_world_gt, frame, out);
    info.synthesized = st == Status::kOk;
    return st;
  }

  if (!perception_root.empty()) {
    loadFramePerception(perception_root, sequence, frame, out);
    info.loaded_from_file = hasGeometry(out);
  }

  // File / Auto: return whatever was loaded (possibly empty).
  if (source == PerceptionSource::kFile) {
    return Status::kOk;
  }

  if (source == PerceptionSource::kAuto) {
    return Status::kOk;
  }

  // kNoisy: synthesize if needed, then apply configured noise.
  if (!hasGeometry(out)) {
    const Status st = synthesizeAt(map_loader, map_radius_m, projection, T_world_gt, frame, out);
    if (st != Status::kOk) return st;
    info.synthesized = true;
  }

  if (noise.enabled()) {
    out = addPerceptionNoise(out, noise, noise_seed);
    info.noise_applied = true;
  }
  return hasGeometry(out) ? Status::kOk : Status::kInvalidArgument;
}

}  // namespace cam_loc::perception
