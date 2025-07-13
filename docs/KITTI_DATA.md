# KITTI Data Conventions

Datasets live beside the repository, not inside it:

```bash
D=../camera-map-localization-data
```

## Supported datasets

| Dataset | Use |
|---------|-----|
| KITTI Odometry (`data_odometry_*`) | Poses, calib, grayscale images |
| Semantic KITTI (optional) | Lane, road-boundary, pole and traffic-sign extraction |
| KITTI Raw (optional v2) | IMU/GPS for global prior |

## Input provenance

Which artifacts are downloaded, which are generated, and by what. The shapes,
frames and units these become on the way into the engine are in
[ARCHITECTURE.md](ARCHITECTURE.md#inputs-and-outputs).

| Artifact | Dataset | Produced by | Becomes |
|----------|---------|-------------|---------|
| `poses/XX.txt` | KITTI Odometry | `scripts/download_kitti_odometry.sh` — direct download, ~2 MB | GT pose, and the relative ego for the predict step |
| `dataset/sequences/XX/calib.txt` | KITTI Odometry | same script | Intrinsics, and the velodyne→cam0 extrinsic |
| `dataset/sequences/XX/velodyne/*.bin` | KITTI Odometry | Manual download (~80 GB archive) | Input to `preprocess_kitti --mode lidar` |
| `sequences/XX/labels/*.label` | Semantic KITTI | `scripts/download_semantic_kitti_labels.sh` | Input to `preprocess_kitti --mode lidar` |
| `dataset/sequences/XX/image_0/*.png` | KITTI Odometry (gray archive) | Manual download, **optional** | Visualization background only — never an algorithm input |
| `<repo>-data/smoke_kitti/` | — | `scripts/prepare_smoke_kitti.sh [frames]` — synthesized, no download | A straight synthetic sequence for the smoke test |
| `<repo>-data/perception/<seq>/<frame:06d>.lanes.json` | derived | C++ `preprocess_kitti` (`--mode lidar` or `--mode png`) | 2-D image-space perception: lanes, road edges, poles, signs |
| Trajectory corridor map | derived | C++ `TrajectoryCorridorMap`, from the GT poses at run time | 3-D world map (the default when no `--map-path`): lane geometry on the road, plus poles and signs |
| `<repo>-data/map/<seq>/*.json` | user-supplied | Exported by any external tool | 3-D world map |
| `*.osm` + `georef.json` | OpenStreetMap | User-supplied extract | 3-D world map, via `MapGeoref` |
| — | camera images | **TODO: run a camera perception model** | Would replace the `preprocess_kitti` output as the source of lane and boundary polylines |

Two things worth noticing in that table.

Nothing in the current pipeline reads a camera image. The "camera perception" is
either projected from the map or derived from SemanticKITTI's LiDAR labels, so
the only genuinely camera-derived input is the one marked TODO.

And the default map is built from ground-truth poses, so the oracle path is a
closed loop by construction: the map comes from GT, the perception is projected
from GT, and matching them recovers GT. That measures the backend — whether the
search, the frames and the filter agree — and not perception. It is an upper
bound, not an accuracy claim; the honest number needs a real map, which KITTI
Odometry does not ship. See [OPEN_ITEMS.md](OPEN_ITEMS.md).

## Directory layout (expected)

Datasets live beside the repository, not inside it — see
[BUILD.md](BUILD.md#dataset-directory).

```
camera-map-localization/            the repository
camera-map-localization-data/
  kitti_odometry/
    dataset/sequences/00/image_0/000000.png
    dataset/sequences/00/calib.txt
    poses/00.txt
  perception/                       # produced offline
    00/000000.lanes.json
  map/                              # optional override
    00/corridor.map.json
  smoke_kitti/                      # scripts/prepare_smoke_kitti.sh
```

## calib.txt parsing

Standard KITTI odometry calibration keys:

- `P0`, `P1` — 3×4 projection (rectified cam0/cam1)
- `R0_rect` — 3×3 rectification
- `Tr` — 3×4 velodyne → cam0

**Rig frame:** cam0 is the rig frame (`T_rig_cam0 = I`) — X right, Y down,
Z forward. The pose grid and the bird's-eye raster reason in a vehicle frame
(X forward, Y left, Z up); `core::Frames` is the only place that conversion
lives. See [ARCHITECTURE.md](ARCHITECTURE.md).

## Poses

`poses/KK.txt`: each line is 12 floats forming a 3×4 matrix `R|t` (row-major) — cam0 pose in world frame.

Derived:

- `T_world_cam0(i)` — global prior
- `T_curr_prev = T_world_cam0(i-1)^-1 * T_world_cam0(i)` — relative ego

## Timestamp

Odometry poses have no timestamps. Assume 10 Hz:

`timestamp_ns = frame_index * 100_000_000`

## Perception (offline contract)

File: `<repo>-data/perception/<seq>/<frame:06d>.lanes.json`

```json
{
  "frame": 42,
  "features": [
    {"type": "lane_solid", "points": [[1240.5, 380.2], [1100.0, 420.0]]},
    {"type": "road_edge",  "points": [[500.0, 700.0], [600.0, 650.0]]},
    {"type": "pole",       "points": [[812.0, 120.0], [812.0, 300.0]]},
    {"type": "sign",       "points": [[240.0, 96.0], [268.0, 96.0]]}
  ]
}
```

One list, not one per class: every polyline carries its own `type`, so a new
landmark class costs an enumerator and nothing else.

Types: `lane_solid`, `lane_dashed`, `road_edge`, `pole`, `sign` (the short forms
`solid`, `dashed`, `edge` are also accepted on read). Points are **rectified
image coordinates** (KITTI cam0, via `P0`).

Generate with `preprocess_kitti` (`--mode lidar` or `--mode png`).

### What the datasets can and cannot give

| Landmark | SemanticKITTI class | Extracted by |
|----------|--------------------|--------------|
| Lane marking | 60 | horizontal run scan |
| Road boundary | edge of road (40) | leftmost/rightmost road pixel per row |
| Pole | 80 | vertical run scan |
| Traffic sign | 81 | vertical run scan |
| Traffic light | **none** | — |
| Crosswalk | **none** | — |

The last two rows are not an omission. SemanticKITTI has no class for either, so
they cannot be extracted from this dataset at any effort — see
[ARCHITECTURE.md](ARCHITECTURE.md) for what each class would have constrained.

The scan orientation is chosen per class: lane markings and road edges run
across the image, poles and signs stand upright, and a horizontal scan meets a
pole one or two pixels at a time and discards it as too short.

## Map (trajectory corridor)

Auto-generated at runtime from GT poses if no file is provided.

Optional file `<repo>-data/map/<seq>/corridor.map.json`:

```json
{
  "polylines": [
    {"id": 0, "type": "lane_solid", "points": [[x,y,z], ...]}
  ]
}
```

Points in **world frame** (same as KITTI pose world).

Load with `run_sequence --map-path <repo>-data/map/00/corridor.map.json` (world-frame JSON).

### Native OSM + georef

```bash
run_sequence --map-path extract.osm \
  --map-georef "$D"/map/00/georef.json \
  --map-align-yaw   # optional: align +X to frame-0 motion
```

`georef.json`:

```json
{
  "origin_lat_deg": 49.0,
  "origin_lon_deg": 8.4,
  "origin_alt_m": 0,
  "world_yaw_deg": 0
}
```

`origin_lat/lon` is the geographic point that corresponds to world `(0,0,0)`.
`world_yaw_deg` rotates the local East/North plane so the map's forward axis
lines up with the sequence heading; the result is cam0 coordinates (X right,
Y down, Z forward), the same frame as the poses.

OSM ways with `highway=*`, `barrier=*`, or `man_made=kerb` are imported as polylines.

JSON may include a top-level `"georef"` block; per-polyline `"coord_frame": "wgs84"`
stores `[lat_deg, lon_deg, alt_m]` instead of world XYZ.

## Evaluation metrics

Reported by `eval_sequence` and `benchmark`:

- **Translation RMSE** — root-mean-square of ‖t_est − t_gt‖ over frames
- **Yaw RMSE** — root-mean-square heading difference (degrees), measured about
  the vehicle up axis
- **Match rate** — fraction of frames with successful map-matching update
