# AMET Object Detection progress

Progress is tracked by stage rather than by date. The historical host Docker
implementation is excluded from active progress.

## Completed stages

- PhysiCar environment and ROS interface survey: topics, QoS, live scan
  geometry, odometry, camera, and TF
- ROS 2 Jazzy C++17 package setup with native build
- LaserScan validation: structure, metadata, and usable-beam classification
- Polar-to-XY conversion with preserved original scan indices
- Adaptive Adjacent Point Clustering
- 360-degree scan boundary handling, including cluster merging across the
  +/-pi wrap for full-circle scans
- Cluster geometry: closest point
- Cluster geometry: representative position as observed surface centroid
- Cluster geometry: observed width
- Cluster geometry: max extent
- Cluster geometry: observed convex-hull footprint without inflation
- Automated tests and lint: CTest 11/11 passed, 102 tests, 0 errors,
  0 failures, 11 skipped (cppcheck no-findings entries)
- Live verification against the real `/scan_filtered` stream at
  approximately 9.58 to 9.96 Hz with `invalid_messages=0` and
  `invalid_geometry=0`
- Shape stability verification while approaching and stopping in front of an
  obstacle: `closest_range_m` about 2.10, 1.73, 1.20, 0.69, 0.40 m during the
  approach, then about 0.395 to 0.402 m while stopped near 0.4 m, with an
  observed width of about 0.043 to 0.048 m over the same segment

## Progress by scope

- Scope the user committed to implement and verify before 8/17: 100 percent
  complete
- Preparation scope including the planner interface agreement and real-map
  validation: approximately 90 percent
- Full Object Detection roadmap: approximately 40 percent

## Remaining work

- Agree the Object List output contract with Local Path Planning
- Decide the output frame: `lidar_link`, `base_link`, `odom`, or `map`
- Define the timestamp and freshness policy
- Distinguish an empty observation from a sensor failure in the output
  representation
- Implement the custom message and publisher once the contract is agreed
- Re-validate clustering thresholds and the large merged wall cluster on the
  real 8/18 competition map
- Traffic-light baseline for the one-time start transition
- Health, heartbeat, and failure handling
- Re-evaluate object classification or YOLO only if it proves necessary

## Role split with Local Path Planning

Object Detection:

- provide measured obstacle position, size, and observed footprint
- preserve measurement timestamp, frame, original scan index, and validity
- publish the Object List once the contract is agreed

Local Path Planning:

- apply vehicle dimensions and safety margin
- perform obstacle inflation
- perform collision checking
- decide the avoidance direction
- generate the local path

State Machine and the central System/Mission Manager own motion permission,
stop, recovery, and resume behavior.

## Known observation

The practice world yields one cluster spanning roughly 5 to 7 m. The wall and
track boundary stay connected under the current provisional thresholds. The
geometry values correctly describe that merged observation, so this is a
threshold tuning item for the real map, not a geometry defect.
