#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

MicroXRCEAgent udp4 -p 8888
