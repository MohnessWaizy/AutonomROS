#!/bin/bash

source ros.sh
source ros/install_aarch64/local_setup.bash
ros2 launch nav2_local car_launch.py autostart:=true
