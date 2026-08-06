# ROS 2 PX4 Stereo VO/VIO Project

This repository provides a ROS 2, PX4 SITL, and Gazebo simulation pipeline for testing vision-based UAV navigation.

The current system uses:

- a custom PX4 X500 model with two forward-facing stereo cameras;
- YAML-defined flight paths;
- PX4 offboard control;
- ROS bag recording;
- trajectory analysis;
- offline stereo VO and stereo-IMU VIO evaluation.

## Project objective

The purpose of the project is to compare different camera-based navigation methods for an indoor UAV without relying on GPS.

The planned evaluation includes:

- stereo visual odometry;
- stereo visual-inertial odometry;
- camera-based localization;
- camera-IMU localization.

A single simulated flight can be recorded as a ROS bag and processed afterward by multiple estimation methods. Their estimated trajectories can then be compared with the commanded path and PX4 reference odometry.

## System architecture

```text
Gazebo simulation
│
├── PX4 SITL
│   └── /fmu/out/vehicle_odometry
│           │
│           v
│      Local pose adapter
│           │
│           v
│      /uav/local_pose
│           │
│           v
│      YAML path executor
│           │
│           v
│        /cmd_vel
│           │
│           v
│      ROS 2 offboard controller
│           │
│           v
│          PX4
│
└── Stereo camera model
    ├── /stereo/left/image_raw
    ├── /stereo/right/image_raw
    ├── /stereo/left/camera_info
    └── /stereo/right/camera_info
            │
            v
       ROS bag recording
            │
            v
       Offline VO / VIO
            │
            v
       Trajectory analysis
```

## Repository structure

```text
ROS2_PX4_Project/
├── path_specs/                 YAML flight-path definitions
├── px4_offboard_control/       ROS 2 control package
├── scripts/
│   ├── analysis/               Path and trajectory analysis
│   ├── batch/                  Repeated experiments and ROS bag recording
│   ├── camera/                 Stereo Gazebo-to-ROS bridges
│   ├── control/                Local pose, TF, and offboard control
│   ├── flight/                 YAML path execution
│   ├── setup/                  Reproducible installation and build scripts
│   └── simulation/             PX4, Gazebo, and DDS startup scripts
├── simulation_assets/          Repository-contained Gazebo assets
├── mono_cam_left/              Left stereo-camera model
├── mono_cam_right/             Right stereo-camera model
├── x500_dual_cam/              Custom stereo UAV model
├── Dockerfile                  ROS 2 and Gazebo container environment
├── docker-compose.yml          Container configuration
├── requirements.txt            Python analysis dependencies
└── SETUP.md                    Complete first-time installation guide
```

Generated ROS bags, analysis files, VINS output, and batch experiment folders are excluded from Git.

## First-time installation

Follow:

```text
SETUP.md
```

It explains how to:

- build the Docker image;
- start the container;
- install the pinned external repositories;
- install the custom PX4 files;
- build Micro XRCE-DDS Agent;
- build the ROS 2 workspace;
- test the Gazebo simulation.

## Normal startup workflow

Open a separate container terminal for every process.

In each new terminal:

```bash
docker exec -it px4_ros2_container bash
cd /home/user/ros2_ws/ROS2_PX4_Project
```

### Terminal 1 — Micro XRCE-DDS Agent

```bash
./scripts/simulation/start_agent.sh
```

### Terminal 2 — PX4 and Gazebo

```bash
./scripts/simulation/sim_environment_menu.sh
```

For the repository-contained test environment, select:

```text
vio_marker_arena
```

### Terminal 3 — Stereo camera bridges

```bash
./scripts/camera/start_camera_bridges_all_worlds.sh
```

Published ROS 2 topics:

```text
/stereo/left/image_raw
/stereo/right/image_raw
/stereo/left/camera_info
/stereo/right/camera_info
```

### Terminal 4 — Local pose adapter

```bash
./scripts/control/start_local_pose_from_px4.sh
```

This converts:

```text
/fmu/out/vehicle_odometry
```

into:

```text
/uav/local_pose
```

The path executor is intentionally connected to the generic `/uav/local_pose` topic. The PX4 adapter can later be replaced by a VO, VIO, or localization provider without changing the flight executor.

### Terminal 5 — TF broadcaster

```bash
./scripts/control/start_tf_broadcaster.sh
```

### Terminal 6 — Offboard controller

```bash
./scripts/control/start_offboard_control.sh
```

### Terminal 7 — Path analyzer

```bash
./scripts/flight/path_workflow_menu.sh
```

Select a YAML path and choose:

```text
Start analyzer only
```

### Terminal 8 — Flight execution

Start the menu again:

```bash
./scripts/flight/path_workflow_menu.sh
```

Select the same YAML path and choose:

```text
Execute flight only
```

The executor follows the selected path and returns the UAV to the path start position after completion.

## YAML flight paths

Flight definitions are stored in:

```text
path_specs/
```

The current segment executor supports commands such as:

```yaml
type: segment_path

segments:
  - move:
      x_m: 1.0
      y_m: 0.0
      z_m: 0.0

  - arc:
      radius_m: 1.5
      angle_deg: 360
      direction: ccw
```

Batch experiments currently require:

```yaml
type: segment_path
```

## Batch experiments

Start the batch workflow with:

```bash
./scripts/batch/run_batch_experiment.sh
```

The batch runner can:

- repeat one selected flight path;
- record stereo images and camera information;
- record PX4 IMU and odometry data;
- record local pose and TF;
- run the path analyzer;
- generate numeric summaries and an HTML report.

Generated experiments are stored under:

```text
batch_experiments/<timestamp>_<world>_<path>/
```

Important generated files include:

```text
summary/batch_report.txt
summary/batch_summary.csv
summary/numeric_stats.csv
```

The `batch_experiments/` directory is ignored by Git because a single experiment can contain several gigabytes of ROS bag data.

## Trajectory analysis

The analysis scripts can compare:

- the commanded YAML reference path;
- PX4 vehicle odometry;
- the actually flown local path;
- a VO or VIO estimate.

Relevant scripts include:

```text
scripts/analysis/analyze_path_from_spec.py
scripts/analysis/compare_vins_to_reference.py
scripts/analysis/start_path_analysis.sh
```

Generated analysis files are written to ignored output directories such as:

```text
path_analysis/
vins_output/
batch_experiments/
```

## External software versions

The tested external repositories are pinned in:

```text
scripts/setup/versions.env
```

Current pinned revisions:

```text
PX4-Autopilot
715837cd5a12061ee7abb20be5f757a4edf2ee13

px4_msgs
7abb73740198f51485c03761458e30f77d8a36dd

Micro-XRCE-DDS-Agent
155cfaaf8b7abac2e85d4a62d3649b09ace0be55
```

## Simulation worlds

The reproducible default world is:

```text
vio_marker_arena
```

Its required world file is included in this repository.

Some additional worlds use large external Gazebo assets that are not committed to Git. They may require a separate asset installation:

```text
office_cpr_px4
simple_env_2_px4
simple_env_2_px4_clean
```

## Stopping the system

Stop the running simulation processes:

```bash
./scripts/simulation/stop_all.sh
```

Stop the container from Windows PowerShell:

```powershell
docker compose stop
```

## Current status

Implemented:

- reproducible Docker environment;
- pinned PX4, `px4_msgs`, and Micro XRCE-DDS Agent versions;
- custom stereo X500 model;
- ROS 2 stereo camera bridges;
- PX4 offboard velocity control;
- YAML path execution;
- automatic return to the path start;
- repeated batch experiments;
- ROS bag recording;
- path and trajectory analysis;
- offline stereo VO/VIO comparison support.

Large raw datasets and generated outputs remain local and are intentionally excluded from the repository.
