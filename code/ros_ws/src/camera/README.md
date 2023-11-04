# Camera node

## Run

To start the node, type:

`ros2 run camera cam2image`

By default, the camera will run with a resolution of 320x240 at 30 FPS, using the Sensor Data QoS profile.
The QoS policies for reliability, history and depth can be individually changed using the parameters of the same name.

For a list of all possible parameters, run:

`ros2 run camera cam2image -h`


## Source

The code has been taken from https://github.com/ros2/demos/tree/rolling/image_tools and amended for our use.