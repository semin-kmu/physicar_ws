# kau_object_detection_lane

Localization-free LiDAR Object Detection and start signal detection for the
KAU AMET PhysiCar.

This package exists so the Lane-only driving stack can see obstacles without a
map, without `odom`, and without any localisation node running. It publishes
the Object List in **`base_link`**, and the only transform it reads is the
static `base_link <- lidar_link` that `robot_state_publisher` produces from the
URDF.

---

## 1. Independent package, not a wrapper

This package has **no build-time and no runtime dependency on
`kau_object_detection`**. It does not call its executables, link its libraries,
include its headers, or read files from its share directory. Verified by
`test/test_config_and_launch.py` on every build, and by:

```
$ colcon list --names-only --packages-up-to kau_object_detection_lane
kau_msgs
kau_object_detection_lane
```

`src/kau_object_detection` can be deleted and this package still builds and
runs.

### Source synchronisation — read this before fixing a bug

The detection logic here is a **fork taken on 2026-08-25** from
`kau_object_detection`, which was the team's validated code at that date. Each
forked file carries a banner naming the file it came from.

**Fixes do not propagate.** A bug fixed in one package stays broken in the
other until somebody ports it by hand. If you fix something in either package,
check whether the same code exists in the other one.

| this package | forked from | changes |
|---|---|---|
| `scan_validation.{hpp,cpp}` | `laser_scan_validator.{hpp,cpp}` | namespace, include paths |
| `laser_clusterer.{hpp,cpp}` | `laser_scan_clusterer.{hpp,cpp}` | namespace, include paths |
| `cluster_geometry.{hpp,cpp}` | `cluster_geometry.{hpp,cpp}` | namespace, include paths |
| `cone_occupancy.{hpp,cpp}` | `cone_occupancy.{hpp,cpp}` | namespace, include paths |
| `obstacle_markers.{hpp,cpp}` | `obstacle_markers.{hpp,cpp}` | namespace, include paths |
| `start_signal_logic.{hpp,cpp}` | `start_signal_logic.{hpp,cpp}` | namespace, include paths; **consecutive-green run replaced by a sliding window** (2026-08-25) |
| `green_lamp_detector.{hpp,cpp}` | `green_lamp_detector.{hpp,cpp}` | namespace, include paths |
| `start_signal_detector_node.cpp` | `start_signal_detector_node.cpp` | namespace, include paths; sliding-window parameters and log lines (2026-08-25) |
| `object_transform.{hpp,cpp}` | condensed from `track_geometry`, `track_roi`, `object_list_transform` | track ring, mount offset and world-pose transform removed |
| `object_list.{hpp,cpp}` | `object_list.{hpp,cpp}` | world-pose and ROI status overloads removed; default frame `map` → `base_link` |
| `lane_laser_object_detector_node.cpp` | `laser_scan_clusterer_node.cpp` | tracker, Gazebo world pose, track ROI, diagnostic twin and every map path removed |

Not forked in at all: `world_pose.{hpp,cpp}`, `track_geometry.{hpp,cpp}`,
`track_roi.{hpp,cpp}`, `obstacle_tracker.{hpp,cpp}`,
`object_list_diagnostic.{hpp,cpp}`, `laser_scan_validator_node.cpp`,
`config/amet_2026_track.yaml`.

---

## 2. LiDAR Object Detection

### Pipeline

```
/scan_filtered  sensor_msgs/LaserScan, frame_id=lidar_link, ~10 Hz
      |
      v
scan structure validation      (frame_id, sample count, finite/range checks)
      v
speckle / isolated point filter
      v
polar -> Cartesian
      v
adjacent clustering            (adaptive threshold, 360-degree wrap-around)
      v
cluster geometry               (convex hull footprint, chord width, max extent)
      v
cone occupancy gate            (width gate + nominal cone radius +
                                visible-slice centre correction)
      v
tf2  base_link <- lidar_link  at scan.header.stamp
      v
publish                        NO tracker, NO EMA, NO carry-over
      v
/perception/obstacles  kau_msgs/ObstacleCircleArray, frame_id=base_link
```

### Output contract

| | |
|---|---|
| topic | `/perception/obstacles` |
| type | `kau_msgs/msg/ObstacleCircleArray` |
| `header.frame_id` | **`base_link`** |
| `header.stamp` | the source LaserScan's measurement time (never the publish time) |
| `center_x` | **[m]** forward of the vehicle, `+x` ahead |
| `center_y` | **[m]** `+y` to the vehicle's left |
| `radius` | **[m]** physical obstacle radius, no safety margin |
| `confidence` | `1.0` on every accepted circle (binary approval, unchanged policy) |
| QoS | BEST_EFFORT / KEEP_LAST(1) / VOLATILE |
| rate | one message per input scan, so about 10 Hz |

Units are **metres**, matching `ObstacleCircle.msg`. `KauPath` uses centimetres;
do not mix them.

### Status

| status | meaning | obstacles |
|---|---|---|
| `STATUS_OK` (0) | this scan was observed and placed | this scan's circles, possibly empty |
| `STATUS_LIDAR_UNAVAILABLE` (1) | no scan, or the scan failed structural validation | always empty |
| `STATUS_POSE_UNAVAILABLE` (2) | **not reachable here** — there is no pose source to be stale | — |
| `STATUS_TRANSFORM_UNAVAILABLE` (3) | the `base_link <- lidar_link` lookup failed at the scan stamp | always empty |
| `STATUS_INTERNAL_ERROR` (4) | placement failed open despite a valid transform | always empty |

`STATUS_OK` with an empty array means **"observed, nothing there"**. A non-OK
status with an empty array means **"cannot tell"** — not "no obstacles".

A failing frame never republishes an earlier one, and recovery is immediate:
the next scan whose lookup succeeds is `STATUS_OK` again, with no
re-confirmation period.

### What "BEV" means here

In this project **BEV means the base_link ego Cartesian obstacle list above**,
not a pixel image. A 2D LaserScan converted to XY in a vehicle frame already
*is* a top-down metric representation.

Deliberately **not** implemented, and out of scope for this package:
`sensor_msgs/Image` BEV, `nav_msgs/OccupancyGrid`, `PointCloud2`, costmaps,
Frenet coordinates, vehicle inflation, safety margins, path corridor filtering.

### Why there is no tracker

The source package runs association + outlier gating + an EMA
(`smoothing_alpha`) on the published centres. That is correct in `map`, where a
stationary cone has stationary coordinates.

In `base_link` it is not, because the vehicle moves: a stationary cone changes
coordinates every frame. Feeding that into an EMA reports the obstacle
**farther away than it is**, and past a certain speed the outlier gate starts
rejecting the true measurement outright — the obstacle silently disappears from
the list. With the source package's tuning (`alpha` 0.2, outlier gate 0.20 m) at
10 Hz, the steady-state check is `step / alpha > 0.20 m`, i.e. it breaks above
roughly **0.4 m/s**.

So every scan here is judged on its own. There is no track id, no velocity
estimate, no confirmation-frame delay, no track timeout, and no state of any
kind between scans. `test/test_no_temporal_persistence.cpp` proves it.

The cost is that frame-to-frame jitter is passed through unfiltered, and the
`minimum_confirmation_frames` protection against a one-frame false positive is
gone with it. Smoothing an obstacle list in a moving frame is the consumer's
problem to solve with ego-motion, not something this node can do correctly on
its own.

### TF requirements

The node performs exactly one lookup:

```
tf_buffer.lookupTransform("base_link", scan.header.frame_id, scan.header.stamp)
```

* The lookup time is the **scan stamp**, never `TimePointZero`/latest.
* `map` is never looked up. `map -> odom` and `odom -> base_footprint` are not
  needed and are not read.
* `base_link <- lidar_link` is a **static** URDF transform, so
  `robot_state_publisher` alone satisfies it.
* The mount offset is inside that transform. The node never adds it again, and
  this package has no `lidar_offset_*` parameter for that reason.

---

## 3. Start signal detection

| | |
|---|---|
| input | `/camera/image_raw/compressed`, `sensor_msgs/CompressedImage`, SensorDataQoS |
| output | `/perception/start_permission`, `std_msgs/Bool` |
| QoS out | RELIABLE / KEEP_LAST(1) / VOLATILE |
| rate | `publish_period_s` 0.1, so about 10 Hz, regardless of the camera |

### Confirmation policy — sliding window (changed 2026-08-25)

The source package required **`green_confirmation_frames` greens in a row**. One
dropped or mis-classified camera frame sent the count back to zero, and the
vehicle occasionally never left the start line.

This package instead latches on **`green_required_frames` greens within the last
`green_window_frames` usable frames** — 4 of 8 as shipped. A miss now costs one
sample instead of the whole run.

| parameter | value | meaning |
|---|---|---|
| `green_window_frames` | `8` | how many recent samples the decision looks at (~0.53 s at 15 Hz) |
| `green_required_frames` | `4` | green samples needed inside that window |

The window does **not** have to be full: four greens as the first four frames
latch immediately. Rules:

| samples (oldest → newest) | result |
|---|---|
| `T T T T` | latch — 4 hits |
| `T F T F T F T` | latch — 4 of the last 7 |
| `T F T F T F F F` | no — 3 hits |
| `T T T F F F F F` | no — 3 hits |
| greens at 0, 4, 8, 11 of 12 | no — the first two have left the window |

`green_required_frames` must satisfy `1 <= required <= window`. Anything else,
including a threshold the window can never reach, makes the node **refuse to
start** rather than forbid the start silently.

Full policy:

* starts `false`;
* red, yellow, an unrecognised signal and a decode failure all enter the window
  as **non-green samples** — an undecodable frame is treated conservatively,
  since it can evict an older green and so only ever makes latching harder;
* a decode failure additionally does **not** refresh the camera liveness clock;
* `green_required_frames` greens inside the window latch it `true`;
* once latched it stays `true` — nothing takes it back, and the reported hit
  count freezes at the value that latched it;
* restarting the node starts from `false` again;
* `camera_timeout_s` without a usable frame **clears the whole window**, so
  greens seen before a long outage cannot combine with greens seen after it.
  No effect once latched.

Camera only: no TF, no map, no odom. Independent of localisation by
construction.

### What is still unchanged

**Only the temporal decision changed.** The spatial detection — ROI, HSV bounds,
component area, aspect ratio, fill ratio — plus the topics, message type, QoS,
publish period, camera timeout, JPEG decode path, connected-component method and
the permanent latch are all the copied values, untouched. That is deliberate:
the effect of the sliding window can be attributed to the window alone.

Wider traffic-light hardening — ROI adjustment, dynamic ROI, full-frame search,
black panel detection, circularity, RGB/HSV combination, adaptive thresholds —
is a separate task and has **not** been done here.
`test_config_and_launch.py` pins the spatial values so a change cannot slip in
unnoticed.

---

## 4. RViz markers

| | |
|---|---|
| topic | `/perception/obstacle_markers` |
| type | `visualization_msgs/msg/MarkerArray` |
| `frame_id` | same as the Object List (`base_link`) |
| `stamp` | same as the Object List |

Built from the very message published on `/perception/obstacles`, so what RViz
draws and what the planner receives can never disagree. ADD/DELETE bookkeeping
plus `obstacle_marker_lifetime_s` clears stale circles when the count drops, the
list empties, or the status turns non-OK.

**Markers are a debug aid, not a planner input.** Set
`obstacle_marker_enabled: false` to stop advertising the topic, for instance to
save CPU on the real vehicle; nothing about the Object List changes.

---

## 5. Running

Build:

```bash
cd ~/physicar_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select kau_object_detection_lane
```

Simulator (Gazebo drives `/clock`):

```bash
ros2 launch kau_object_detection_lane perception_lane.launch.py
```

Real vehicle:

```bash
ros2 launch kau_object_detection_lane perception_lane.launch.py use_sim_time:=false
```

Individually:

```bash
ros2 launch kau_object_detection_lane lane_object_detector.launch.py
ros2 launch kau_object_detection_lane start_signal_detector.launch.py
```

Selectively, from the combined file:

```bash
ros2 launch kau_object_detection_lane perception_lane.launch.py start_signal:=false
ros2 launch kau_object_detection_lane perception_lane.launch.py lidar:=false
ros2 launch kau_object_detection_lane perception_lane.launch.py markers:=false
```

Nodes: `/lane_laser_object_detector`, `/start_signal_detector`.

`use_sim_time` is deliberately **not** pinned in the YAML. Pinning it would make
it impossible to turn off on the real vehicle, where there is no `/clock` and
`now()` would stay at 0.

---

## 6. Never run alongside the map-frame node

**Do not run this package at the same time as `kau_object_detection`.**

| topic | this package | `kau_object_detection` |
|---|---|---|
| `/perception/obstacles` | `base_link` | `map` (default) |
| `/perception/start_permission` | published | published |

Two publishers on `/perception/obstacles` in **different frames** is the worst
possible failure: the planner receives coordinates from both, in two coordinate
systems, with no way to tell them apart. Obstacles will be placed metres from
where they are.

This package will not kill anything for you. Stop the other stack yourself
before starting this one, and check with:

```bash
ros2 topic info /perception/obstacles --verbose   # expect exactly 1 publisher
```

The two are alternatives, one per driving mode:

* map-based driving (Global Path + AMCL/Cartographer + Local Planner)
  → `kau_object_detection`
* Lane-only driving (no localisation) → `kau_object_detection_lane`

---

## 7. What the Local Planner still owns

This package publishes **measured physical obstacle geometry only**. Everything
below is the planner's responsibility and is deliberately absent here:

* vehicle size, inflation and safety margin;
* collision checking and avoidance direction;
* Frenet / station-lateral conversion;
* path corridor filtering;
* any smoothing of the obstacle stream, including ego-motion compensation if
  the jitter turns out to matter.

Object Detection must not command vehicle motion, and does not.

---

## 8. Limitations

* **No temporal filtering at all.** A single-frame false positive reaches the
  planner. See "Why there is no tracker".
* **Walls, fences and people** are rejected by the cluster width / max-extent
  gate and the cone occupancy gate, which are tuned for competition cones. This
  node reports cone-sized obstacles, not general occupancy.
* **`visible_slice_radius_m` was copied, not re-derived.** Its source comment
  cites a 0.20 m LiDAR height, which is what the URDF TF gives; the simulator
  SDF places the sensor at 0.182 m. Re-deriving the value is a separate task.
* **The start signal gates are unhardened** and assume the vehicle is at the
  known start pose with the lamp inside the fixed ROI.
* **No BEV image, grid or point cloud** is produced. See "What BEV means here".
* `STATUS_POSE_UNAVAILABLE` is unreachable; the constant exists only to keep the
  enum aligned with the message.

---

## 9. Tests

```bash
colcon test --packages-select kau_object_detection_lane
colcon test-result --verbose --test-result-base build/kau_object_detection_lane
```

| suite | cases | covers |
|---|---|---|
| `test_scan_validation` | 9 | frame/sample/finite/range validation |
| `test_laser_clusterer` | 19 | polar→XY, speckle filter, wrap-around clustering, candidate acceptance |
| `test_cluster_geometry` | 22 | convex hull footprint, chord width, max extent |
| `test_cone_occupancy` | 18 | width gate, nominal radius, visible-slice correction |
| `test_object_transform` | 12 | rigid transform, mount offset applied exactly once, transform failure, statelessness |
| `test_object_list` | 16 | status decision, message frame/stamp/units, empty-on-failure |
| `test_no_temporal_persistence` | 8 | first-frame publish, no carry-over, no EMA lag, per-scan stamps |
| `test_obstacle_markers` | 10 | ADD/DELETE/lifetime, purity |
| `test_start_signal_logic` | 24 | sliding window, eviction, red/yellow/unknown, latch, camera timeout, parameter validation |
| `test_green_lamp_detector` | 29 | HSV/shape gates, decode-failure handling |
| `test_config_and_launch` | 16 | YAML syntax, forbidden parameters absent, spatial values pinned, sliding-window values, launch validity, independence |

`ament_cppcheck` reports every file as skipped in this workspace. That is a
pre-existing environment behaviour: `kau_object_detection` shows the same thing
(45/45 skipped).
