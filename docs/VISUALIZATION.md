# Visualization and debug

Paths used below, both siblings of the repository:

```bash
B=../camera-map-localization-build   # ./scripts/ci.sh writes here
D=../camera-map-localization-data    # datasets and evaluation output
```

`viz_frame` uses `LocalizationEngine::set_debug_capture(true)` to snapshot DT images, cost grids, and map chunks for the active frame.

## Offline PNG (`viz_frame`)

### Smoke example

```bash
./scripts/run_viz_smoke.sh
# Opens: "$D"/viz_smoke/frame_000020_panel.png
```

### Manual run

```bash
"$B"/apps/viz_frame/viz_frame \
  --kitti-root "$D"/smoke_kitti \
  --perception-mode oracle \
  --use-gt-plane \
  --frame 20 \
  --trajectory \
  --output-dir "$D"/viz
```

### Output files (per frame)

| File | Content |
|------|---------|
| `frame_NNNNNN_camera.png` | Camera (or gray canvas) + perception + projected map |
| `frame_NNNNNN_image_dt.png` | Image distance transform heatmap |
| `frame_NNNNNN_bev.png` | BEV map + perception overlay |
| `frame_NNNNNN_bev_dt.png` | BEV distance transform |
| `frame_NNNNNN_cost_xy.png` | Aggregated cost slice at best yaw; magenta = argmin |
| `frame_NNNNNN_topdown.png` | Top-down X–Z: map, GT (white), estimate (red) |
| `frame_NNNNNN_panel.png` | Composite of the above |
| `frame_NNNNNN_meta.json` | Costs, match flags, file list |
| `trajectory_gt_est.png` | Full sequence GT vs estimate (with `--trajectory`) |

KITTI `image_0/` images are loaded automatically when present.

## Eval CSV (non-visual debug)

```bash
"$B"/apps/eval_sequence/eval_sequence \
  --kitti-root "$D"/smoke_kitti \
  --perception-mode oracle \
  --use-gt-plane \
  --output-csv "$D"/eval.csv
```

Columns: `frame`, `translation_m`, `lateral_m`, `longitudinal_m`, `vertical_m`, `yaw_deg`, `min_cost`, `cost_spread`, `match`, `flat`, `synth`, `offset_m`.

`translation_m` is the unsigned 3-D norm; the three that follow are the same
error resolved onto the ground-truth vehicle axes and are **signed** — positive
is left of / ahead of / above truth. They reconstruct the norm exactly, so
`lateral_m² + longitudinal_m² + vertical_m² == translation_m²` is a valid check
on a row. Splitting them apart is what makes an along-track bias visible.
