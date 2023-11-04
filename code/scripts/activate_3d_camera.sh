#!/bin/bash

source ros.sh
ros2 lifecycle set /astra_pro_publisher configure
ros2 lifecycle set /astra_pro_publisher activate
