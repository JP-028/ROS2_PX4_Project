# First-Time Setup


## 1. Host requirements

Tested host setup:

- Windows 10 or Windows 11
- Docker Desktop
- XLaunch or another X server
- Git

## 2. Software stack

The project uses:

- ROS 2 Humble
- Gazebo Harmonic
- PX4 SITL
- Micro XRCE-DDS Agent
- px4_msgs

## 3. Folder structure

The repository should be located inside a ROS 2 workspace:

```text
ros2_ws/
└── ROS2_PX4_Project/
```

## 4. Clone the repository

Create a ROS 2 workspace and clone this repository into it.

Run in Windows PowerShell:

```powershell
mkdir ros2_ws
cd ros2_ws
git clone https://github.com/JP-028/ROS2_PX4_Project.git
cd ROS2_PX4_Project
git switch refactor-script-structure
```

The resulting folder structure is:

```text
ros2_ws/
└── ROS2_PX4_Project/
```

After the development branch has been merged, use `main` instead of `refactor-script-structure`.

## 5. Build the Docker image

From the `ROS2_PX4_Project` folder in Windows PowerShell, run:

```powershell
docker compose build
```

## 6. Start the container

From Windows PowerShell, run:

```powershell
docker compose up -d
```

Open a shell inside the container:

```powershell
docker exec -it px4_ros2_container bash
```

The repository is available inside the container at:

```text
/home/user/ros2_ws/ROS2_PX4_Project
```

## 7. Install the pinned external repositories

Inside the container, run:

```bash
cd /home/user/ros2_ws/ROS2_PX4_Project
./scripts/setup/install_external_repositories.sh
```

This installs or verifies:

- PX4-Autopilot
- px4_msgs
- Micro XRCE-DDS Agent
- the custom PX4 airframe
- the stereo camera models
- the reproducible Gazebo world

## 8. Build the project

Inside the container, run:

```bash
cd /home/user/ros2_ws/ROS2_PX4_Project
./scripts/setup/build_project.sh
```

This builds:

- Micro XRCE-DDS Agent
- px4_msgs
- px4_offboard_control

## 9. Start the X server

Before opening Gazebo, start XLaunch on Windows.

Recommended settings:

1. Multiple windows
2. Start no client
3. Disable access control

The container uses:

```text
DISPLAY=host.docker.internal:0
```

## 10. Verify the installation

Inside the container, run:

```bash
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash

command -v MicroXRCEAgent
ros2 pkg prefix px4_msgs
ros2 pkg prefix px4_offboard_control
ros2 pkg prefix ros_gz_bridge
gz sim --versions
```

## 11. Test the simulation

Open two separate container terminals.

Terminal 1:

```bash
cd /home/user/ros2_ws/ROS2_PX4_Project
./scripts/simulation/start_agent.sh
```

Terminal 2:

```bash
cd /home/user/ros2_ws/ROS2_PX4_Project
./scripts/simulation/sim_environment_menu.sh
```

Select:

```text
vio_marker_arena
```

This is the default reproducible world included in the repository.

## 12. Stop the environment

Inside the container:

```bash
cd /home/user/ros2_ws/ROS2_PX4_Project
./scripts/simulation/stop_all.sh
```

From Windows PowerShell:

```powershell
docker compose stop
```

To remove the container without deleting the workspace:

```powershell
docker compose down
```
