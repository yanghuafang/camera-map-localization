# KITTI Data Conventions

## Supported datasets

| Dataset | Use |
|---------|-----|
| KITTI Odometry (`data_odometry_*`) | Poses, calib, grayscale images |
| Semantic KITTI (optional) | Offline lane/boundary extraction |
| KITTI Raw (optional v2) | IMU/GPS for global prior |

## Directory layout (expected)

```
kitti_odometry/
  dataset/sequences/00/image_0/000000.png
  dataset/sequences/00/calib.txt
  poses/00.txt
data/perception/          # produced offline
  00/000000.lanes.json
data/map/                 # optional override
  00/corridor.map.json
```

## calib.txt parsing

Standard KITTI odometry calibration keys:

- `P0`, `P1` — 3×4 projection (rectified cam0/cam1)
- `R0_rect` — 3×3 rectification
- `Tr` — 3×4 velodyne → cam0

**Rig frame (v1):** cam0 is the rig frame (`T_rig_cam0 = I`).

## Poses

`poses/KK.txt`: each line is 12 floats forming a 3×4 matrix `R|t` (row-major) — cam0 pose in world frame.

Derived:

- `T_world_cam0(i)` — global prior
- `T_curr_prev = T_world_cam0(i-1)^-1 * T_world_cam0(i)` — relative ego

## Timestamp

Odometry poses have no timestamps. Assume 10 Hz:

`timestamp_ns = frame_index * 100_000_000`

## Perception (offline contract)

File: `data/perception/<seq>/<frame:06d>.lanes.json`

```json
{
  "frame": 42,
  "lane_lines": [
    {"type": "solid", "points": [[1240.5, 380.2], [1100.0, 420.0]]}
  ],
  "road_boundaries": [
    {"type": "edge", "points": [[500.0, 700.0], [600.0, 650.0]]}
  ]
}
```

Points are **rectified image coordinates** (KITTI cam0, using P0).

Generate with `preprocess_kitti` (`--mode lidar` or `--mode png`) or external tools.

## Map (trajectory corridor)

Auto-generated at runtime from GT poses if no file is provided.

Optional file `data/map/<seq>/corridor.map.json`:

```json
{
  "polylines": [
    {"id": 0, "type": "lane_solid", "points": [[x,y,z], ...]}
  ]
}
```

Points in **world frame** (same as KITTI pose world).

Load with `run_sequence --map-path data/map/00/corridor.map.json` (world-frame JSON).

### Native OSM + georef

```bash
run_sequence --map-path extract.osm \
  --map-georef data/map/00/georef.json \
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

`origin_lat/lon` is the geographic point that corresponds to KITTI world `(0,0,0)`.
`world_yaw_deg` rotates local East/North into KITTI XY (+Z up).

OSM ways with `highway=*`, `barrier=*`, or `man_made=kerb` are imported as polylines.

JSON may include a top-level `"georef"` block; per-polyline `"coord_frame": "wgs84"`
stores `[lat_deg, lon_deg, alt_m]` instead of world XYZ.

## Evaluation metrics

Reported by `eval_sequence` and `benchmark`:

- **Translation RMSE** — root-mean-square of ‖t_est − t_gt‖ over frames
- **Yaw RMSE** — mean absolute yaw difference (degrees)
- **Match rate** — fraction of frames with successful map-matching update
