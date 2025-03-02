# KITTI Data Conventions

Datasets live beside the repository, not inside it:

```bash
D=../camera-map-localization-data
```

## Supported datasets

| Dataset | Use |
|---------|-----|
| KITTI Odometry (`data_odometry_*`) | Poses, calib, grayscale images |

## Input provenance

Which artifacts are downloaded, which are generated, and by what.

| Artifact | Dataset | Produced by | Becomes |
|----------|---------|-------------|---------|
| `poses/XX.txt` | KITTI Odometry | `scripts/download_kitti_odometry.sh` — direct download, ~2 MB | GT pose, and the relative ego for the predict step |
| `dataset/sequences/XX/calib.txt` | KITTI Odometry | same script | Intrinsics, and the velodyne→cam0 extrinsic |
| `dataset/sequences/XX/image_0/*.png` | KITTI Odometry (gray archive) | Manual download, **optional** | Visualization background only — never an algorithm input |

## Directory layout (expected)

Datasets live beside the repository, not inside it — see
[BUILD.md](BUILD.md#build-directories).

```
camera-map-localization/            the repository
camera-map-localization-data/
  kitti_odometry/
    dataset/sequences/00/image_0/000000.png
    dataset/sequences/00/calib.txt
    poses/00.txt
```

## calib.txt parsing

Standard KITTI odometry calibration keys:

- `P0`, `P1` — 3×4 projection (rectified cam0/cam1)
- `R0_rect` — 3×3 rectification
- `Tr` — 3×4 velodyne → cam0

`P1` is parsed and never read: the right camera is unused, as stereo is
unimplemented. `R0_rect` and `Tr` are read only by `Calibration::T_cam0_velo()`.

**Rig frame:** cam0 is the rig frame (`T_rig_cam0 = I`) — X right, Y down,
Z forward. `core::Frames` is the only place the conversion to the vehicle frame
(X forward, Y left, Z up) lives.

## Poses

`poses/KK.txt`: each line is 12 floats forming a 3×4 matrix `R|t` (row-major) — cam0 pose in world frame.

Derived:

- `T_world_cam0(i)` — global prior
- `T_curr_prev = T_world_cam0(i-1)^-1 * T_world_cam0(i)` — relative ego

## Timestamp

Odometry poses have no timestamps. Assume 10 Hz:

`timestamp_ns = frame_index * 100_000_000`
