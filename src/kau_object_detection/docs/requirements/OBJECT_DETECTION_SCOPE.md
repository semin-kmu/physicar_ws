# Object Detection scope

## Ownership

Object Detection owns online LiDAR measurement validation, polar-to-XY conversion, clustering, measured obstacle geometry, timestamp-correct coordinate transformation, validity/freshness, and publication.

Local Path Planning owns collision checking, safety-margin inflation, avoidance direction, and path generation. State Machine and the central System/Mission Manager own motion permission, stop, recovery, and resume behavior.

## Baseline decisions

- Detect obstacles online from LiDAR. Do not use simulator truth or hard-coded obstacle coordinates as production perception input.
- Use the camera only for the one-time start traffic-light transition.
- Do not include YOLO in the baseline.
- Preserve frame, measurement timestamp, original scan index, and validity through the pipeline.
- Keep map-dependent and algorithm thresholds in YAML.
- Publish measured size and footprint without planner safety inflation.
- Transform geometry to the planner-required absolute frame with timestamp-correct TF. The final `map` versus `odom` contract is pending.

## Current implementation boundary

This stage implements only LaserScan structural validation and usable-beam classification. It does not implement XY conversion, clustering, geometry, TF transformation, Object List publication, health messages, traffic-light detection, or vehicle control.

