# KAU Object Detection

LiDAR-first ROS 2 Jazzy perception package for the KAU AMET PhysiCar platform.

## Current scope

The package subscribes to `/scan_filtered`, validates every incoming
`sensor_msgs/msg/LaserScan`, converts usable beams to XY, groups them with
scan-boundary-aware Adaptive Adjacent Point Clustering, and computes the
observed geometry of each cluster.

The current stage reports that geometry as read-only diagnostic logging. It does
not duplicate the platform `/scan_filter`, publish an Object List, transform to
an absolute frame, track objects, classify obstacles, command motion, or
implement State Machine behavior.

## Processing pipeline

```text
/scan_filtered
  -> LaserScan validation
  -> usable scan index extraction
  -> Polar-to-XY conversion
  -> Adaptive Adjacent Point Clustering (360-degree boundary aware)
  -> Cluster Geometry
  -> read-only diagnostic log
```

## Computed obstacle shape

Each cluster yields the following measured values:

- `closest point`: measured point with the smallest range, keeping its original
  scan index, XY, range, and angle
- `representative position`: observed surface centroid of the cluster points
- `observed width_m`: chord distance between the first and the last point of the
  cluster in scan order
- `max_extent_m`: largest distance between two observed points, evaluated over
  the footprint vertices
- `observed footprint`: counter-clockwise convex hull of the observed points,
  with no inflation
- `footprint_area_m2`: area of that footprint, zero for degenerate footprints
- `point_count` and the `[first..last]` scan index range of the cluster
- `wraps_scan_boundary`: whether the cluster crosses the +/-pi scan boundary

Degenerate clusters are handled explicitly: an empty cluster is invalid, and one
point, two points, or a collinear set produce a one- or two-vertex footprint with
zero area.

## Meaning and limits of the reported values

These values describe what the LiDAR actually observed. They are not a finished
obstacle model.

- `representative position` is the centroid of the observed surface, not the
  geometric centre of the physical obstacle. The unobserved rear side biases it
  toward the sensor.
- `width_m` is the observed chord width. It is neither the true full width of the
  obstacle nor a safety width, and it shrinks when an obstacle is partially
  occluded or sampled by few beams at long range.
- The footprint carries no safety margin and no inflation. It is the convex hull
  of the measurement points only.
- All coordinates are relative to the `lidar_link` sensor frame. They are not yet
  expressed in an absolute `odom` or `map` frame.
- There is no tracking identity and no object classification. Every scan is
  processed independently.

## Observed PhysiCar interface

- ROS distribution: ROS 2 Jazzy
- Topic: `/scan_filtered`
- Type: `sensor_msgs/msg/LaserScan`
- QoS: `SensorDataQoS`, best effort, volatile, keep last depth 1
- Frame: `lidar_link`
- Samples: 720 ranges and 720 intensities
- Angular span: -180 to +180 degrees
- Configured range: 0.1 to 16.0 m
- Measured processing rate: approximately 9.58 to 9.96 Hz
- `scan_time` and `time_increment`: zero in the simulator

These values are validation evidence, not hard-coded competition-map
assumptions. Runtime expectations remain configurable in YAML.

## Automated test status

```text
CTest: 11/11 passed
Summary: 102 tests, 0 errors, 0 failures, 11 skipped
```

The 11 skipped entries are not failures. `ament_cppcheck` emits one skipped
testcase per analysed source file when it finds no problems, and the cppcheck
test itself passes.

## Live verification on PhysiCar

Verified against the live `/scan_filtered` stream while approaching an obstacle:

- approach sequence of `closest_range_m`: about 2.10 m, 1.73 m, 1.20 m, 0.69 m,
  0.40 m
- stopped at about 0.4 m: `closest_range_m` stayed within about 0.395 to 0.402 m
- observed width over the same stopped segment: about 0.043 to 0.048 m
- `invalid_messages=0`
- `invalid_geometry=0`

The practice world also produces a single cluster spanning roughly 5 to 7 m.
That is the wall and track boundary staying connected under the current
provisional clustering thresholds, not a geometry computation error. The
reported shape correctly describes that merged observation. Threshold retuning
is scheduled for the real competition map.

## Build and test in PhysiCar

```bash
cd ~/physicar_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select kau_object_detection
source install/setup.bash
colcon test --packages-select kau_object_detection --event-handlers console_direct+
colcon test-result --verbose
```

## Run

```bash
cd ~/physicar_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch kau_object_detection laser_scan_clusterer.launch.py
```

Expected live summary:

```text
scan frame=lidar_link usable_points=720 clusters=8 clustered_points=681 ...
  rate_hz=9.84 invalid_messages=0 invalid_geometry=0
  near[0] points=3 closest=(...) closest_range_m=0.398 representative=(...)
    width_m=0.045 max_extent_m=0.045 footprint_vertices=3 footprint_area_m2=0.0001
    scan_indices=[574..576] wraps=0
```

The validation-only node remains available:

```bash
ros2 launch kau_object_detection laser_scan_validator.launch.py
```

Stop a node with `Ctrl+C`.

## Parameters

Parameters are stored in `config/laser_scan_validator.yaml` and
`config/laser_scan_clusterer.yaml`.

Scan input and validation:

- `input_topic`: scan input topic
- `expected_frame_id`: expected sensor frame; empty disables exact matching
- `minimum_sample_count`: structural minimum number of ranges
- `sample_count_tolerance`: allowed difference between angle-derived and actual
  sample counts
- `require_intensities`: require intensities to match ranges
- `summary_period_seconds`: compact live log interval

Clustering and geometry:

- `base_distance_threshold_m`: distance threshold floor between adjacent points
- `angular_resolution_scale`: scale applied to the range-dependent angular
  spacing term of the adaptive threshold
- `minimum_cluster_points`: smallest accepted cluster size
- `circular_scan_tolerance_rad`: tolerance for detecting a full-circle scan
  before merging clusters across the scan boundary
- `collinear_tolerance_m`: perpendicular distance below which a whole cluster
  counts as collinear and its footprint collapses to two points
- `log_cluster_geometry`: enable the per-cluster geometry log
- `geometry_log_cluster_count`: number of nearest clusters to log

Clustering thresholds are provisional until the competition map is available.

## Not implemented yet

- Object List message definition and publication
- `odom` or `map` TF transformation of the geometry
- planner-side inflation, collision checking, and avoidance path generation
- State Machine behavior
- object tracking and identity across scans
- obstacle classification and YOLO
- traffic-light detection
- final thresholds validated on the real competition map

Object Detection owns measured obstacle geometry and validity. Local Path
Planning owns safety inflation, collision checking, avoidance direction, and
path generation.
