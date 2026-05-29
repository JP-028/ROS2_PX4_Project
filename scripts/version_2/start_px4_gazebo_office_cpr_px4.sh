#!/bin/bash
set -e

export DISPLAY=host.docker.internal:0
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=1
export GZ_RENDER_ENGINE=ogre

export ASSET_DIR=/home/user/sim_assets/gazebo_models_worlds_collection
export PX4_DIR=/home/user/PX4-Autopilot

# PX4-Modelle + CPR-Modelle verfügbar machen
export GZ_SIM_RESOURCE_PATH=$ASSET_DIR:$ASSET_DIR/models:$ASSET_DIR/worlds:$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds:$GZ_SIM_RESOURCE_PATH

# Neue PX4-kompatible World ohne .sdf
export PX4_GZ_WORLD=office_cpr_px4

# Drohne ungefähr in die freie Mitte setzen
export PX4_GZ_MODEL_POSE="0,0,0.15,0,0,0"

cd "$PX4_DIR"

PX4_SYS_AUTOSTART=4050 make px4_sitl gz_x500_dual_cam 2> >(grep -v "Could not resolve file" >&2)

