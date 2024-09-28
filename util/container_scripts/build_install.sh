#!/usr/bin/env bash
source ~/.bashrc
source /opt/ros/humble/setup.bash

cd /home/sentient-beings/mnt/ws
rm -rf build log install
colcon build --symlink-install
source install/setup.bash