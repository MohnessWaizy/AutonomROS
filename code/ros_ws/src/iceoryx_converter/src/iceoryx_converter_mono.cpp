#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "iceoryx_msg/msg/image640mono.hpp"

class IceoryxConverterMono : public rclcpp::Node
{
public:
  IceoryxConverterMono()
  : Node("iceoryx_converter_mono")
  {
    this->declare_parameter("frame_id", "base_link");
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("depth/image_raw_converted", 10);
    subscription_ = this->create_subscription<iceoryx_msg::msg::Image640mono>(
      "depth/image_raw", 10, std::bind(
        &IceoryxConverterMono::image_callback, this,
        std::placeholders::_1));
    
    converted.header.frame_id = this->get_parameter("frame_id").get_parameter_value().get<std::string>();
    converted.encoding = "mono16";
    converted.width = 640;
    converted.height = 480;
    converted.data.resize(converted.width * converted.height * 2);
  }

private:
  void image_callback(iceoryx_msg::msg::Image640mono::SharedPtr image)
  {
    memcpy(&converted.data[0], &image->data, converted.width * converted.height * 2);
    publisher_->publish(converted);
  }
  sensor_msgs::msg::Image converted;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<iceoryx_msg::msg::Image640mono>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IceoryxConverterMono>());
  rclcpp::shutdown();
  return 0;
}
