// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "opencv2/highgui/highgui.hpp"

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "iceoryx_msg/msg/image640rgb.hpp"

#include "image_tools/visibility_control.h"


namespace image_tools
{
class Cam2Image : public rclcpp::Node
{
public:
  IMAGE_TOOLS_PUBLIC
  explicit Cam2Image(const rclcpp::NodeOptions & options)
  : Node("cam2image", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    // Do not execute if a --help option was provided
    if (help(options.arguments())) {
      // TODO(jacobperron): Replace with a mechanism for a node to "unload" itself
      // from a container.
      exit(0);
    }
    parse_parameters();
    initialize();
  }

private:
  IMAGE_TOOLS_LOCAL
  void initialize()
  {
    pub_ = create_publisher<iceoryx_msg::msg::Image640rgb>("image", rclcpp::QoS(rclcpp::KeepLast(1)));

    // Initialize OpenCV video capture stream.
    cap.open(device_id_, cv::CAP_V4L2);

    // Set the width and height based on command line arguments.
    cap.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(640));
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(480));
    cap.set(cv::CAP_PROP_FPS, 30);
    if (!cap.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open video stream");
      throw std::runtime_error("Could not open video stream");
    }

    // Start main timer loop
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / freq_)),
      std::bind(&Cam2Image::timerCallback, this));
  }

  /// Publish camera, or burger, image.
  IMAGE_TOOLS_LOCAL
  void timerCallback()
  {
    cv::Mat frame;

    // Initialize a shared pointer to an Image message.
    auto msg_borrow = pub_->borrow_loaned_message();
    auto & msg = msg_borrow.get();

    // Get the frame from the video capture.
    cap >> frame;

    // If no frame was grabbed, return early
    if (frame.empty()) {
      return;
    }

    // cv::flip(frame, frame, 1);

    // Convert to a ROS image
    size_t size = frame.step * frame.rows;
    memcpy(&msg.data[0], frame.data, size);

    // Publish the image message and increment the frame_id.
    pub_->publish(std::move(msg));
  }

  IMAGE_TOOLS_LOCAL
  bool help(const std::vector<std::string> args)
  {
    if (std::find(args.begin(), args.end(), "--help") != args.end() ||
      std::find(args.begin(), args.end(), "-h") != args.end())
    {
      std::stringstream ss;
      ss << "Usage: cam2image [-h] [--ros-args [-p param:=value] ...]" << std::endl;
      ss << "Publish images from a camera stream." << std::endl;
      ss << "Example: ros2 run image_tools cam2image --ros-args -p reliability:=best_effort";
      ss << std::endl << std::endl;
      ss << "Options:" << std::endl;
      ss << "  -h, --help\tDisplay this help message and exit";
      ss << std::endl << std::endl;
      ss << "Parameters:" << std::endl;
      ss << "  frequency\tPublish frequency in Hz. Default value is 30";
      ss << std::endl;
      ss << "  device_id\tDevice ID of the camera. 0 (default) selects the default camera device.";
      ss << std::endl;
      ss << "  frame_id\t\tID of the sensor frame. Default value is 'camera_frame'";
      ss << std::endl << std::endl;
      ss << "Note: try running v4l2-ctl --list-formats-ext to obtain a list of valid values.";
      ss << std::endl;
      std::cout << ss.str();
      return true;
    }
    return false;
  }

  IMAGE_TOOLS_LOCAL
  void parse_parameters()
  {
    // Declare and get remaining parameters
    freq_ = this->declare_parameter("frequency", 30.0);
    device_id_ = static_cast<int>(this->declare_parameter("device_id", 0));
    frame_id_ = this->declare_parameter("frame_id", "camera_frame");
  }

  /// Convert an OpenCV matrix (cv::Mat) to a ROS Image message.
  cv::VideoCapture cap;

  rclcpp::Publisher<iceoryx_msg::msg::Image640rgb>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ROS parameters
  double freq_;
  std::string frame_id_;
  int device_id_;
};

}  // namespace image_tools

RCLCPP_COMPONENTS_REGISTER_NODE(image_tools::Cam2Image)
