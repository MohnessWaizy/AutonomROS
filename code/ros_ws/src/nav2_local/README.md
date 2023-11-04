# Nav2

## launch files

- localization_launch.py:
    - adapted from nav2_bringup localization_launch.py 
    - starts the localization process needed for nav2. 
    - until the use of the valve tracker this only starts the nav2 map server and the lifecycle manager

- navigation_launch.py: 
    - adapted from nav2_bringup navigation_launch.py 
    - starts the navigation process of nav2: controller, planner and recovery server, the bt navigator & the waypoint follower

- simulation_launch.py:
    - launches the localization_launch.py & navigation_launch.py
    - launches a static tf publisher for the transformation between "map" and "odom". Only for simulation purposes, since this will be done by the valve tracker system.
    - launches rviz & gazebo with predefined settings

## To use

Go into the ros_ws dir.

```bash
colcon build
```
After that you need to source the resulting local_setup.bash. 
```bash 
source install/local_setup.bash
```
To start the simulation with nav2:
```bash
ros2 launch nav2_local simulation_launch.py
```
