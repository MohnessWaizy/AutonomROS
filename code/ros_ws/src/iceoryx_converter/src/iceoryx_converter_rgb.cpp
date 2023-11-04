#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "iceoryx_msg/msg/image640rgb.hpp"

class IceoryxConverterRgb : public rclcpp::Node
{
public:
  IceoryxConverterRgb()
  : Node("iceoryx_converter_rgb")
  {
    this->declare_parameter("frame_id", "base_link");
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw_converted", 1);
    subscription_ = this->create_subscription<iceoryx_msg::msg::Image640rgb>(
      "image_raw", 1, std::bind(
        &IceoryxConverterRgb::image_callback, this,
        std::placeholders::_1));

    converted.header.frame_id = this->get_parameter("frame_id").get_parameter_value().get<std::string>();
    converted.encoding = "rgb8";
    converted.width = 640;
    converted.height = 480;
    converted.data.resize(converted.width * converted.height * 3);
  }

private:
  void image_callback(iceoryx_msg::msg::Image640rgb::SharedPtr image)
  {
    memcpy(&converted.data[0], &image->data, converted.width * converted.height * 3);
    publisher_->publish(converted);
  }
  sensor_msgs::msg::Image converted;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<iceoryx_msg::msg::Image640rgb>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IceoryxConverterRgb>());
  rclcpp::shutdown();
  return 0;
}
