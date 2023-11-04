# Run simulation

in ros_ws use 

```bash
colcon build
```
After that you need to source the resulting local_setup.bash. 
```bash 
source install/local_setup.bash
```
To start the gazebo simulator:
```bash
ros2 launch slash_description gazebo.launch.py
```

To start rviz with the model loaded:
```bash
ros2 launch slash_description rviz.launch.py
```