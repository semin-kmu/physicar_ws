# KAU AMET Object Detection workspace rules

- This package runs natively in the PhysiCar AI ROS 2 Jazzy workspace. Do not use the archived host Docker commands here.
- Object Detection is LiDAR-first. The camera is reserved for the one-time start traffic light.
- Subscribe to `/scan_filtered` with sensor-data/best-effort QoS. Keep the input topic and expected frame configurable.
- Do not hard-code practice-map obstacle coordinates, cone positions, or map-dependent thresholds.
- Keep tunable thresholds in YAML.
- Preserve measurement timestamp, frame, and original scan indices through perception processing.
- Object Detection owns measured obstacle geometry and validity. Local Planning owns safety inflation, collision checking, avoidance direction, and path generation.
- Object Detection must not command vehicle motion. State Machine and System/Mission Manager behavior is owned by other team members.
- The final absolute output frame remains a team interface decision. Current PhysiCar provides `odom`; no `map` frame was observed on 2026-08-14.
- Do not modify `app.physicar`, `.devcontainer/`, `examples/`, or other platform-owned files outside this package.
- External backup is user-managed. Do not configure a remote or push unless the user explicitly asks.
- A stage is complete only after automated tests pass, changes are reviewed, and the user verifies the behavior in PhysiCar.

