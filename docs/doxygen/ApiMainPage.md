\mainpage camera-map-localization API reference

This is the generated reference for the public headers under `include/cam_loc/`
— a C++17 implementation of camera map-matching localization on KITTI Odometry.
It documents what each type is for, and above all **what frame and what units
each value is in**, which is where this problem is easiest to get wrong.

The narrative documentation is not here. It lives in the repository, and
[docs/README.md](https://github.com/yanghuafang/camera-map-localization/blob/main/docs/README.md)
is where to start. This page exists only as the landing page for the generated
site; it is not one of the guides.

## Read the frame conventions first

Every 3-D quantity in this API is in the **KITTI rectified cam0 frame** — X
right, Y down, Z forward — and "world" means the cam0 world that the odometry
poses are expressed in. `cam_loc::core::Projection` states the conventions and
the transforms between image, rig and BEV.

Two places currently contradict that, and they are marked with `@warning` on the
declarations themselves: `cam_loc::core::BevConfig` describes the bird's-eye
raster in a Z-up, X-forward vehicle frame, and `cam_loc::map::MapGeoref` returns
Z-up ENU. Resolving the split is the top item in
[docs/OPEN_ITEMS.md](https://github.com/yanghuafang/camera-map-localization/blob/main/docs/OPEN_ITEMS.md).

## How the layers fit

`include/cam_loc/` is an acyclic dependency graph with `core/` on top of the
data types. Reading it from the bottom up:

| Layer | Start at | Depends on |
| --- | --- | --- |
| `types/` | cam_loc::Status, cam_loc::LocalizationParams, cam_loc::LocalizationResult | nothing |
| `kitti/` | cam_loc::kitti::Calibration, cam_loc::kitti::Pose, cam_loc::kitti::FramePerception, cam_loc::kitti::MapChunk | `types/` |
| `map/` | cam_loc::map::IMapLoader, cam_loc::map::PolylineMap, cam_loc::map::MapGeoref | `kitti/` |
| `perception/` | cam_loc::perception::ResolvePerception, cam_loc::perception::PerceptionNoiseParams | `kitti/`, `map/` |
| `cuda/` | cam_loc::cuda::IsAvailable and the GPU entry points | `types/` only |
| `core/` | cam_loc::core::LocalizationEngine, cam_loc::core::PoseSampler, cam_loc::core::CostGrid, cam_loc::core::LocalizationKF | all of the above |
| `benchmark/`, `viz/` | cam_loc::benchmark::BenchmarkCase, cam_loc::viz::RenderFrameViz | `core/` |

`cam_loc::core::LocalizationEngine` is the entry point: give it a calibration
and a `cam_loc::map::IMapLoader`, then one `cam_loc::kitti::Egomotion` and one
`cam_loc::kitti::FramePerception` per frame.

## What this site adds over reading the headers

The prose is the same prose as in the headers, so reading the sources directly
loses nothing. What is easier to see rendered is the shape of the API: which
types the engine actually consumes, the `cam_loc::map::IMapLoader` hierarchy,
and the include graph that shows `core/` sitting on top of everything else.

For the per-frame algorithm, the input and output contract, and what each map
feature class constrains, see
[docs/ARCHITECTURE.md](https://github.com/yanghuafang/camera-map-localization/blob/main/docs/ARCHITECTURE.md).
