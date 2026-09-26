# Architecture Decisions

This is a lightweight Architecture Decision Record. “Accepted” means the decision is visible in the current repository baseline; it does not imply fresh runtime verification.

## ADR-001 — Use GitHub as shared project state

- **Decision:** Track collaboration rules, current status, tasks, architecture, decisions and experiments in the repository.
- **Evidence:** `AGENTS.md`, `PROJECT_STATUS.md`, `TASKS.md` and `docs/` created for this project.
- **Reason:** Provide ChatGPT and Codex with one versioned, reviewable state source.
- **Status:** Accepted.

## ADR-002 — Consume the GO2 onboard UTLiDAR over DDS

- **Decision:** Use `/utlidar/cloud` and `/utlidar/imu` published by the GO2 over ROS 2/CycloneDDS; do not start an additional PC-side LiDAR driver in the current mainline.
- **Evidence:** Root `README.md`, `code/scripts/go2_lidar_preflight.sh`, `transform_everything.py`.
- **Reason:** Reason not documented beyond the statement that GO2 publishes these streams directly.
- **Status:** Accepted in current baseline; hardware availability is `NOT VERIFIED` in this documentation cycle.

## ADR-003 — Use Point-LIO for localization and registered point cloud

- **Decision:** Transform the GO2 sensor streams and feed them into `point_lio_unilidar`; expose registered scan and state estimation through launch remaps.
- **Evidence:** `mapping_utlidar.launch`, `utlidar.yaml`, `laserMapping.cpp`.
- **Reason:** Reason not documented.
- **Status:** Accepted; accuracy and real-time behavior need verification.

## ADR-004 — Combine FAR global planning with local planning/path following

- **Decision:** Feed exploration goals to FAR, FAR waypoints to `localPlanner`, and the local path to `pathFollower`.
- **Evidence:** `nav_launch.sh`, FAR/local planner source interfaces and launch files.
- **Reason:** Reason not documented.
- **Status:** Accepted in current baseline.

## ADR-005 — Generate exploration goals with WP5 frontier logic

- **Decision:** Accumulate `/terrain_map`, select frontier observation points and publish them as `/goal_point`; use FAR reach feedback and local BFS distance for reselection.
- **Evidence:** `wp5_explore_node.py`.
- **Reason:** Reason not documented.
- **Status:** Accepted in the 2026-08-31 rollback baseline; performance remains problematic according to README experiments.

## ADR-006 — Place a safety gate between path following and GO2 Sport API

- **Decision:** Route `/cmd_vel_raw` through `go2_safety_gate`, with DISARMED/ARMED/FAULT state, dry-run, limits, timeouts and lease checks, before publishing `/api/sport/request`.
- **Evidence:** `safety_gate.py`, `nav_launch.sh`.
- **Reason:** The source documents prevention of unarmed commands, stale-command continuation and loss of robot-state/lease control.
- **Status:** Accepted; physical safety performance is `NOT VERIFIED`.

## ADR-007 — Keep runtime data and build products out of Git

- **Decision:** Do not track colcon output, runtime logs, rosbag databases, generated analysis data, caches or local credentials.
- **Evidence:** Root `.gitignore`.
- **Reason:** Keep the source repository reviewable and avoid large/generated/sensitive artifacts.
- **Status:** Accepted. Raw experiment bags must be managed separately with an explicit location and checksum if used later.
