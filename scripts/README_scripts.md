# Script Versions

## version_1

Old scripts before the Gazebo Harmonic bridge fix.

## version_2

Current working scripts for:

- ROS2 Humble
- PX4 SITL
- Gazebo Harmonic
- x500 dual camera
- working ROS2 camera feed

## One-time requirement

Run this once inside the container if the camera bridge does not work:

    apt update
    apt install -y ros-humble-ros-gzharmonic-bridge

## Start from closed system

### PowerShell

Run this first in Windows PowerShell:

    docker start px4_ros2_container

For every terminal below, open a new PowerShell and enter:

    docker exec -it px4_ros2_container bash

Then run the matching command inside the container.

## Terminal 1 - MicroXRCEAgent

    cd /home/user/ros2_ws/ROS2_PX4_Project
    ./scripts/version_2/start_agent.sh

## Terminal 2 - PX4 SITL + Gazebo dual camera

    cd /home/user/ros2_ws/ROS2_PX4_Project
    ./scripts/version_2/start_px4_gazebo_dual_cam.sh

Wait until Gazebo opens and the drone is visible.

## Terminal 3 - Offboard control bridge

    cd /home/user/ros2_ws/ROS2_PX4_Project
    ./scripts/version_2/start_offboard_control.sh

## Terminal 4 - Keyboard control

    cd /home/user/ros2_ws/ROS2_PX4_Project
    ./scripts/version_2/start_keyboard_control.sh

Controls:

    w / Arrow Up       forward
    s / Arrow Down     backward
    a / Arrow Left     left
    d / Arrow Right    right
    i                  up
    k                  down
    j                  yaw left
    l                  yaw right
    Space              stop
    q                  quit

## Terminal 5 - Front camera bridge

    cd /home/user/ros2_ws/ROS2_PX4_Project
    ./scripts/version_2/start_camera_bridge_front.sh

## Terminal 6 - Check front camera feed

    cd /home/user/ros2_ws/ROS2_PX4_Project
    ./scripts/version_2/check_camera_feed_front.sh

You should see an average rate.

## Terminal 7 - Open camera viewer

    cd /home/user/ros2_ws/ROS2_PX4_Project
    ./scripts/version_2/open_camera_view.sh

In the rqt_image_view window, select:

    /world/default/model/x500_dual_cam_0/link/camera_link/sensor/camera/image

## Optional - Circle flight instead of keyboard control

Use this after Terminal 1, Terminal 2 and Terminal 3 are running:

    cd /home/user/ros2_ws/ROS2_PX4_Project
    ./scripts/version_2/start_circle_flight.sh

