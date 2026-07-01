# Multimodal Object Recognition: Parameter Tuning Guide

This guide covers the runtime parameters of the `mmor_node`
(`multimodal_object_recognition`). All parameters are declared in
`ros/src/multimodal_object_recognition_parameters.cpp` and are re-readable at
runtime through the `on_parameter_event` callback, so you can tune them live
with `ros2 param set` without restarting the node.

## Quick reference

```bash
# List the node (confirm the exact name first)
ros2 node list

# Read a parameter
ros2 param get /mmor sac_distance_threshold

# Set a parameter live
ros2 param set /mmor prism_min_height 0.02

# Dump all current values to a yaml you can reuse
ros2 param dump /mmor
```

Persistent defaults live in
`ros/config/scene_segmentation_constraints.yaml`. Edit that file for changes
you want to survive restarts; use `ros2 param set` for interactive tuning.

## Pipeline overview

The node runs two recognition paths that can be toggled independently:

- **RGB path** (`enable_rgb_recognizer`, default on): a 2D detector proposes
  bounding boxes; each box is back-projected into the organized point cloud
  (`getPointCloudROI`), the table plane is stripped with a z passthrough, and a
  pose is estimated per cluster.
- **Point-cloud path** (`enable_pc_recognizer`, default off): the full PCL
  scene-segmentation pipeline (voxel filter, crop box, plane extraction, prism,
  Euclidean clustering) runs on the cloud.

Knowing which path is active tells you which parameters actually affect your
output.

---

## 1. Downsampling (voxel grid)

| Parameter | Default | Range | Meaning |
|-----------|---------|-------|---------|
| `voxel_leaf_size` | 0.009 | 0.0 - 1.0 | Leaf edge length in metres for the voxel grid. |
| `voxel_filter_field_name` | `z` | - | Axis used for the pre-voxel field limit. |
| `voxel_filter_limit_min` | -0.15 | -10 - 10 | Lower bound on that axis. |
| `voxel_filter_limit_max` | 0.3 | -10 - 10 | Upper bound on that axis. |

**Elaboration.** `voxel_leaf_size` is the master speed/detail trade-off.
Smaller leaves keep more points (slower, more detail); larger leaves thin the
cloud (faster, coarser). For small LEGO parts, 0.009 m (9 mm) is already on the
coarse side.

**Examples.**
- Parts look chunky or small features vanish: lower to `0.005`.
- Pipeline is CPU-bound / laggy: raise to `0.012`-`0.015`.

---

## 2. Crop box filter

Enabled by default (`enable_cropbox_filter: true`). Defines the axis-aligned
workspace volume in the target frame (`base_link`). Everything outside is
discarded before segmentation. This is the cheapest, most reliable way to cut
away walls, floor, and far background.

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `cropbox_filter_x_limit_min` / `_max` | 0.0 / 0.8 | Forward extent (m). |
| `cropbox_filter_y_limit_min` / `_max` | -0.4 / 0.4 | Left/right extent (m). |
| `cropbox_filter_z_limit_min` / `_max` | -0.2 / 0.6 | Down/up extent (m). |

**Elaboration.** Set these to the physical table bounds relative to
`base_link`. Tightening the box removes clutter with zero risk of deforming the
objects themselves.

**Examples.**
- Robot sees the far wall behind the table: reduce `cropbox_filter_x_limit_max`
  to the table's far edge, e.g. `0.6`.
- Neighbouring table on the left leaks in: raise `cropbox_filter_y_limit_min`
  to `-0.25`.

---

## 3. Passthrough filter (optional)

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `enable_passthrough_filter` | false | Enable an extra single-axis passthrough. |
| `passthrough_filter_field_name` | `x` | Axis to filter. |
| `passthrough_filter_limit_min` / `_max` | 0.0 / 0.8 | Bounds (m). |

Redundant with the crop box in most setups. Use it only when you need a single
extra axis cut that the crop box does not already provide.

---

## 4. Normal estimation

| Parameter | Default | Range | Meaning |
|-----------|---------|-------|---------|
| `normal_radius_search` | 0.03 | 0.0 - 0.5 | Neighbourhood radius for surface normals (m). |
| `use_omp` | false | - | Use OpenMP-parallel normal estimation. |
| `num_cores` | 8 | 1 - 16 | Threads when `use_omp` is on. |

**Elaboration.** Larger `normal_radius_search` gives smoother, more stable
normals but blurs fine geometry and costs time. For small parts keep it near
the object scale (~0.03 m). Turn on `use_omp` if normal estimation dominates
runtime.

---

## 5. Plane segmentation (SAC)

Fits the dominant plane (the table) so it can be removed. Active in the
point-cloud path.

| Parameter | Default | Range | Meaning |
|-----------|---------|-------|---------|
| `sac_max_iterations` | 1000 | 0 - 100000 | RANSAC iterations. |
| `sac_distance_threshold` | 0.01 | 0.0 - 1.0 | Inlier band thickness (m). |
| `sac_optimize_coefficients` | true | - | Refine plane coefficients. |
| `sac_x_axis` / `sac_y_axis` / `sac_z_axis` | 0 / 0 / 1 | 0.0 - 1.0 | Expected plane normal axis. |
| `sac_eps_angle` | 0.09 | 0 - 1.57 | Allowed tilt from that axis (rad). |
| `sac_normal_distance_weight` | 0.05 | 0.0 - 1.0 | Weight of normal vs point distance. |

**Elaboration.**
- `sac_distance_threshold` is the key knob. Too small and a bumpy or slightly
  curved table is only partially removed (residual plane points survive as fake
  objects). Too large and it eats the bottoms of real objects.
- The axis constraint (`sac_z_axis = 1.0`, `sac_eps_angle = 0.09` ≈ 5.2°) forces
  a near-horizontal plane, which prevents RANSAC from latching onto a large
  vertical surface.

**Examples.**
- Table not fully removed, speckle left behind: raise `sac_distance_threshold`
  to `0.015`.
- Object bases getting clipped: lower it to `0.007`.
- Tilted / uneven table: widen `sac_eps_angle` to `0.15`.

---

## 6. Prism extraction (height band above the plane)

After the plane is found, points are kept only if they sit within a height band
above it. This is the primary table-vs-object separator in the point-cloud
path.

| Parameter | Default | Range | Meaning |
|-----------|---------|-------|---------|
| `prism_min_height` | 0.01 | 0.0 - 5.0 | Lower band edge above plane (m). |
| `prism_max_height` | 0.1 | 0.0 - 5.0 | Upper band edge above plane (m). |

**Elaboration.** `prism_min_height` is the main table-crop lever: raise it to
drop residual plane points, but not above the shortest object you must detect.
`prism_max_height` caps how tall an object can be.

**Examples.**
- Thin film of table still attached under objects: raise `prism_min_height` to
  `0.02`.
- Flat parts (single-plate LEGO) disappearing: lower to `0.006`.
- Tall towers cut off at the top: raise `prism_max_height` to `0.2`.

---

## 7. Radius outlier removal

| Parameter | Default | Range | Meaning |
|-----------|---------|-------|---------|
| `outlier_radius_search` | 0.03 | 0.0 - 10.0 | Neighbour search radius (m). |
| `outlier_min_neighbors` | 20 | 0 - 1000 | Min neighbours to survive. |

**Elaboration.** Removes floating speckle. Raising `outlier_min_neighbors` or
lowering the radius makes filtering more aggressive (cleaner, but can erase
sparse thin objects).

---

## 8. Euclidean clustering

Groups the remaining points into candidate objects and rejects clusters outside
the size/shape gates.

| Parameter | Default | Range | Meaning |
|-----------|---------|-------|---------|
| `cluster_tolerance` | 0.02 | 0.0 - 2.0 | Max gap within one cluster (m). |
| `cluster_min_size` | 25 | 0 - 1000 | Min points per cluster. |
| `cluster_max_size` | 20000 | 0 - 2^31-1 | Max points per cluster. |
| `cluster_min_height` | 0.011 | 0.0 - 5.0 | Min cluster height above polygon (m). |
| `cluster_max_height` | 0.09 | 0.0 - 5.0 | Max cluster height (m). |
| `cluster_max_length` | 0.25 | 0.0 - 5.0 | Max cluster length (m). |
| `cluster_min_distance_to_polygon` | 0.04 | 0.0 - 5.0 | Min distance from plane boundary (m). |
| `center_cluster` | false | - | Recenter cluster at origin. |
| `pad_cluster` | true | - | Pad clusters to a fixed size. |
| `padded_cluster_size` | 2048 | 128 - 4096 | Target point count when padding. |

**Elaboration.**
- `cluster_tolerance` controls merging vs splitting. Too large merges touching
  parts into one blob; too small shatters a single object into fragments.
- `cluster_min_size` / `cluster_min_height` reject noise and plane residue.
- `cluster_max_*` reject over-large blobs (usually merged clutter or unremoved
  plane).

**Examples.**
- Two adjacent bricks recognised as one: lower `cluster_tolerance` to `0.012`.
- One brick split into pieces: raise it to `0.03`.
- Noise dots detected as objects: raise `cluster_min_size` to `50`.

---

## 9. Pose / workspace geometry

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `object_height_above_workspace` | 0.022 | Assumed object centre height above table (m). |
| `container_height` | 0.07 | Container pose height (m). |
| `target_frame_id` | `base_link` | Frame all output poses are expressed in. |

---

## 10. RGB proposal path

These only matter when `enable_rgb_recognizer` is on. This is the default path,
and the one that drives the RViz cluster visualization.

| Parameter | Default | Range | Meaning |
|-----------|---------|-------|---------|
| `rgb_roi_adjustment` | 2 | 0 - 50 | Pixels the 2D box is grown before back-projection. |
| `rgb_bbox_min_diag` | 21 | 0 - 500 | Reject boxes with diagonal below this (px). |
| `rgb_bbox_max_diag` | 175 | 0 - 500 | Reject boxes with diagonal above this (px). |
| `rgb_cluster_filter_limit_min` | 0.0060 | -1.0 - 1.0 | Plane-strip offset above the ROI's lowest point (m). |
| `rgb_cluster_filter_limit_max` | 0.35 | -1.0 - 1.0 | Upper z offset above the ROI's lowest point (m). |
| `rgb_cluster_remove_outliers` | true | - | Statistical outlier removal on the ROI cloud. |

**Elaboration.**
- `rgb_roi_adjustment` grows the 2D box by N pixels on each side. Keep small
  (0-2). Large values push the ROI past the point cloud bounds and raise
  `"Pixel location is out of range."` from `pointcloud_utils_ros`.
- `rgb_bbox_min_diag` / `rgb_bbox_max_diag` gate detections by apparent size,
  filtering both tiny false positives and oversized misdetections.
- `rgb_cluster_filter_limit_min` is the table-crop knob for the RGB path. The
  ROI cloud is passthrough-filtered on z between
  `lowest_z + rgb_cluster_filter_limit_min` and
  `lowest_z + rgb_cluster_filter_limit_max`. Since the table sits at the lowest
  z, the default 6 mm only trims a thin sliver. This same filter now also runs
  on the published ROI cloud, so the RViz clusters respond to it live.

**Examples.**
- Table visible under objects in the RViz clusters: raise
  `rgb_cluster_filter_limit_min` to `0.015`-`0.020`.
  ```bash
  ros2 param set /mmor rgb_cluster_filter_limit_min 0.018
  ```
- Short/flat parts vanishing after that change: back off toward `0.010`.
- `"Pixel location is out of range."` in the logs: set `rgb_roi_adjustment` to
  `0`.

---

## 11. Region-of-interest gate (`enable_roi`)

Rejects detected object poses that fall outside a distance window from
`base_link`. Off by default in code, but enabled in the shipped yaml.

| Parameter | Default | Range | Meaning |
|-----------|---------|-------|---------|
| `enable_roi` | false | - | Enable the pose distance gate. |
| `roi_base_link_to_laser_distance` | 0.350 | 0.0 - 1.0 | Near cutoff on x from base link (m). |
| `roi_max_object_pose_x_to_base_link` | 0.700 | 0.0 - 2.0 | Far cutoff on x (m). |
| `roi_min_bbox_z` | 0.03 | 0.0 - 1.0 | Minimum object height to accept (m). |

**Elaboration.** Poses with `x < roi_base_link_to_laser_distance` or
`x > roi_max_object_pose_x_to_base_link` are dropped. Use it to ignore the robot
body (too near) and far background (too far).

**Examples.**
- Valid objects on the far side of the table are dropped: raise
  `roi_max_object_pose_x_to_base_link` to `0.9`.
- Robot chassis points detected as objects: raise
  `roi_base_link_to_laser_distance` to `0.4`.

---

## 12. Recognizer toggles

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `enable_rgb_recognizer` | true | Run the 2D-proposal path. |
| `enable_pc_recognizer` | false | Run the full PCL scene-segmentation path. |
| `octree_resolution` | 0.0025 | Octree leaf size for the PC recognizer (m). |
| `debug_mode` | true | Publish debug clouds/images and annotated bounding boxes. |

**Example.** Enable real plane segmentation live:
```bash
ros2 param set /mmor enable_pc_recognizer true
ros2 param set /mmor prism_min_height 0.02
ros2 param set /mmor sac_distance_threshold 0.012
```

---

## Suggested tuning workflow

1. **Bound the workspace first.** Set the crop box to the physical table. Most
   background clutter disappears here with no downside.
2. **Fix the plane removal.** Tune `sac_distance_threshold`, then
   `prism_min_height`, until the table is gone but object bases remain (PC path),
   or tune `rgb_cluster_filter_limit_min` (RGB path).
3. **Tune clustering.** Adjust `cluster_tolerance` for correct splitting/merging,
   then `cluster_min_size` / `cluster_min_height` to reject noise.
4. **Gate by size and distance.** Use `rgb_bbox_*` and the `enable_roi` window to
   drop implausible detections.
5. **Optimise speed last.** Raise `voxel_leaf_size` and enable `use_omp` only
   after accuracy is acceptable.
6. **Persist.** Once happy, copy the values into
   `ros/config/scene_segmentation_constraints.yaml` (or `ros2 param dump`).

## Tips

- Change one parameter at a time and watch the debug clouds in RViz.
- Keep `debug_mode: true` while tuning; turn it off for deployment to save
  bandwidth.
- Replace `/mmor` with your actual node name from `ros2 node list`.
