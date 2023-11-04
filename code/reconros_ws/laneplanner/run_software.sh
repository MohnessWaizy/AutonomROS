#!/bin/bash

sudo sh -c '/opt/reconos/reconos_init.sh && . /opt/ros/galactic/setup.sh && . /mnt/project/build.msg/install/local_setup.sh && export ROS_DOMAIN_ID=53 && /home/xilinx/pwmcontrol/carcontrol' 
