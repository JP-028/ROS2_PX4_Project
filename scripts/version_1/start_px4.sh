#!/bin/bash
cd /home/user/PX4-Autopilot
export DISPLAY=host.docker.internal:0
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=1
export GZ_RENDER_ENGINE=ogre
PX4_SYS_AUTOSTART=4050 make px4_sitl gz_x500_dual_cam
