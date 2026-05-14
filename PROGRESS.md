# bartbot 2D → 3D Navigation Upgrade

**Robot:** Tracked rescue robot  
**Sensors:** Livox MID-360 LiDAR + RealSense D435i on 6-DOF arm  
**Odometry:** DLIO (direct_lidar_inertial_odometry)  
**Stack:** ROS2 Jazzy, Nav2

---

## Phase 1 — Nav2 3D Stack (Complete)

Replace 2D Nav2 (DWB + NavFn + LaserScan) with a 3D-capable stack using MPPI, SMAC Hybrid-A*, and voxel costmaps fed directly from point clouds.

### Files Created

#### `config/nav2_params_3d.yaml`
New parameter file replacing `nav2_params.yaml` for 3D operation.

- **Controller:** `nav2_mppi_controller::MPPIController` replaces `dwb_core::DWBLocalPlanner`
  - DiffDrive motion model, batch_size 2000, 56 time steps at 0.05 s
  - Critics: ConstraintCritic, GoalCritic, GoalAngleCritic, PathAlignCritic, PathFollowCritic, PathAngleCritic, PreferForwardCritic
- **Planner:** `nav2_smac_planner::SmacPlannerHybrid` replaces `nav2_navfn_planner::NavfnPlanner`
  - Reeds-Shepp motion model, 0.40 m min turning radius, 5 s planning budget
- **Local costmap:** VoxelLayer (`/dlio/odom_node/pointcloud/deskewed`) + ObstacleLayer (`/bartbot_camera/depth/color/points`, frame `camera_link_d435i`) + InflationLayer
- **Global costmap:** StaticLayer + VoxelLayer + ObstacleLayer (RealSense) + InflationLayer
- **AMCL removed:** DLIO provides odometry directly; lifecycle manager list updated accordingly
- **map_server:** `yaml_filename` set to `/home/peak/ros2_ws/src/bartbot_nav/map/map.yaml`

#### `launch/bringup_3d_nav.launch.py`
New launch file replacing `bringup_launch.py` for 3D operation.

- `pointcloud_to_laserscan` node removed (no longer needed)
- `IncludeLaunchDescription` for `bartbot_description.launch.py` added to bring up `robot_state_publisher` and (optionally) `joint_state_publisher` for the full arm TF chain (`base_link → ... → camera_link_d435i`)
- `use_joint_state_publisher` argument (default `true`) gates `joint_state_publisher` — set `false` when the arm driver publishes `/joint_states`
- Default `params_file` points to `nav2_params_3d.yaml`
- AMCL node removed from launch

#### `map/map.yaml`
Created — was missing; `map_server` could not start without it.

- References existing `dlio_map2_clean_map.pgm`
- Resolution: 0.05 m/px, standard Nav2 thresholds

### Files Modified

#### `direct_lidar_inertial_odometry/cfg/dlio.yaml`
- Added `use_sim_time: false`
- Added explicit frame IDs: `frames/odom: odom`, `frames/baselink: base_link`, `frames/lidar: lidar_base_link`, `frames/imu: imu_link`

#### `bartbot_description/launch/bartbot_description.launch.py`
- Added `use_joint_state_publisher` `DeclareLaunchArgument` (default `true`)
- `joint_state_publisher` node gated with `IfCondition(use_joint_state_publisher)`

---

## Known Issues

| # | Issue | Status |
|---|-------|--------|
| 1 | **joint_state_publisher conflict** — if the arm driver publishes `/joint_states`, running `joint_state_publisher` alongside it causes duplicate/conflicting messages to `robot_state_publisher`. Workaround: launch with `use_joint_state_publisher:=false`. Needs confirmation once arm driver is running. | Pending |
| 2 | **`imu_link` frame name** — `dlio.yaml` sets `frames/imu: imu_link`; confirm this matches the frame your IMU driver actually publishes. | Pending |
| 3 | **`map.yaml` origin** — origin is `[0.0, 0.0, 0.0]`. If the PGM was saved with a non-zero origin, update the `map.yaml` accordingly. | Pending |
| 4 | **SMAC turning radius** — 0.40 m is an estimate for the tracked platform. Measure and tune against actual track geometry. | Pending |

---

## Phase 2 — 3D Point Cloud Localisation

**Goal:** Replace the 2D PGM map + AMCL pipeline with point-cloud-based localisation against `dlio_map2_clean.ply`.

**Approach:**
- Load `dlio_map2_clean.ply` as a reference map using a point cloud map server (e.g. `pcl_ros` or a custom map publisher)
- Use **DLIO** in localisation-only mode (or NDT/ICP scan matching) to register live `/dlio/odom_node/pointcloud/deskewed` against the PLY map
- Alternatively: use **`lidarslam`** or **`hdl_localization`** (NDT) which accept a PCD/PLY prior map
- Publish a corrected `map → odom` transform to close the localisation loop into Nav2
- Remove `map_server` + `amcl` from the lifecycle manager once 3D localisation is confirmed

**Key files to create/modify:**
- `config/localisation_3d.yaml` — NDT/ICP params, map path pointing to `dlio_map2_clean.ply`
- `launch/bringup_3d_nav.launch.py` — add localisation node, remove `map_server`/`amcl`

---

## Phase 3 — Elevation Mapping for Terrain

**Goal:** Build a real-time elevation map so the planner can reason about slopes, steps, and traversable terrain — critical for a rescue robot in unstructured environments.

**Approach:**
- Integrate **`elevation_mapping`** (ETH Zurich) subscribing to `/dlio/odom_node/pointcloud/deskewed` and odometry from `/dlio/odom_node/odom`
- Configure robot-centric elevation map (rolling window, ~5 × 5 m at 0.05 m resolution)
- Expose the elevation map as a Nav2 costmap layer via `elevation_mapping_cupy` or a custom bridge
- Add a traversability filter: cells exceeding a slope/step threshold marked as lethal

**Key files to create/modify:**
- `config/elevation_mapping.yaml` — sensor input, map size, filters
- `launch/bringup_3d_nav.launch.py` — add `elevation_mapping` node
- `config/nav2_params_3d.yaml` — add elevation costmap layer to local costmap

---

## Phase 4 — Full 3D Waypoint Navigation

**Goal:** End-to-end autonomous 3D waypoint navigation with robust recovery behaviours suited to rescue scenarios.

**Approach:**
- Upgrade Nav2 BT to use 3D-aware behaviour trees (ComputePathThroughPoses, custom recovery branches)
- Add **velocity smoother** (`nav2_velocity_smoother`) to handle tracked-robot jerk limits
- Integrate **RVIZ2 3D goal panel** or a mission manager node for operator waypoint input
- Add **slope-aware MPPI cost term** using elevation map gradient as an additional critic
- Implement **stall/stuck detection** tuned for tracked platform (tracks can spin in place — `SimpleProgressChecker` thresholds need revision)
- Add **GPS/fiducial re-localisation** hook for long-range drift correction if operating outdoors

**Key files to create/modify:**
- `config/nav2_params_3d.yaml` — velocity smoother, revised progress checker thresholds, BT path
- `launch/bringup_3d_nav.launch.py` — velocity smoother node, mission manager
- `bt/` — custom BT XML for 3D recovery sequence
