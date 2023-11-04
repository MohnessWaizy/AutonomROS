# ORBBEC Astra Pro 3D camera node

This node publishes a stream of a depth image and a color image.

## Prerequisites

The AstraSDK must be installed in `/opt/AstraSDK`.

## Run

First export the following:
```
export LD_LIBRARY_PATH=/opt/AstraSDK/lib/:$LD_LIBRARY_PATH
```

To run it, run:
```
ros2 run astra_pro astra_pro_publisher
```

This node is a lifecycle/managed node, control it with:
```
ros2 lifecycle set /astra_pro_publisher <transition>
```
Hereby, `<transition>` is the transition you want to make.

Images can only be viewed via RViz. You must set `Best Effort` as Reliability Policy for the topic.

Based on: https://github.com/Lien182/ros2_astra_pro_node and https://github.com/orbbec/ros_astra_camera (for camera info parameters)
