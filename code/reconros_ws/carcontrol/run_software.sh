#!/bin/bash

sudo bash -c '
export CYCLONEDDS_URI=file:///home/xilinx/config.xml &&
/opt/reconos/reconos_init.sh &&
source /opt/ros/galactic/setup.bash &&
source /home/xilinx/carcontrol/build.msg/install/local_setup.bash &&
source /home/xilinx/ros/install_aarch64/local_setup.bash &&
export ROS_DOMAIN_ID=53 &&
/home/xilinx/carcontrol/carcontrol' 
