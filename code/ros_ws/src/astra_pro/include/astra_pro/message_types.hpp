#ifndef ASTRA_PRO__MESSAGE_TYPES_HPP_
#define ASTRA_PRO__MESSAGE_TYPES_HPP_

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "iceoryx_msg/msg/image640rgb.hpp"
#include "iceoryx_msg/msg/image640mono.hpp"

#define RGB_IMAGE_MSG_TYPE iceoryx_msg::msg::Image640rgb
#define DEPTH_IMAGE_MSG_TYPE iceoryx_msg::msg::Image640mono
#define CAMERA_INFO_MSG_TYPE sensor_msgs::msg::CameraInfo
#define POINT_CLOUD_MSG_TYPE sensor_msgs::msg::PointCloud2

#endif  // ASTRA_PRO__MESSAGE_TYPES_HPP_
