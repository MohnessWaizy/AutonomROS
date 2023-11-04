#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "iceoryx_msg/msg/point_cloud640.hpp"

class IceoryxConverterPointCloud : public rclcpp::Node
{
public:
  IceoryxConverterPointCloud()
  : Node("iceoryx_converter_rgb")
  {
    this->declare_parameter("frame_id", "depth_camera_link");
    auto qos = rclcpp::QoS(1);
    qos.best_effort();
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points_converted", 10);
    subscription_ = this->create_subscription<iceoryx_msg::msg::PointCloud640>(
      "points", qos, std::bind(
        &IceoryxConverterPointCloud::image_callback, this,
        std::placeholders::_1));

    converted.header.frame_id = this->get_parameter("frame_id").get_parameter_value().get<std::string>();
    converted.width = 640;
    converted.height = 480;

    sensor_msgs::msg::PointField x_field;
    x_field.name = "x";
    x_field.count = 1;
    x_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    x_field.offset = 0;
    converted.fields.push_back(std::move(x_field));

    sensor_msgs::msg::PointField y_field;
    y_field.name = "y";
    y_field.count = 1;
    y_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    y_field.offset = 4;
    converted.fields.push_back(std::move(y_field));

    sensor_msgs::msg::PointField z_field;
    z_field.name = "z";
    z_field.count = 1;
    z_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    z_field.offset = 8;
    converted.fields.push_back(std::move(z_field));

    sensor_msgs::msg::PointField rgb_field;
    rgb_field.name = "rgb";
    rgb_field.count = 1;
    rgb_field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    rgb_field.offset = 12;
    converted.fields.push_back(std::move(rgb_field));

    converted.point_step = 16;
    converted.row_step = converted.width * converted.point_step;
    converted.data.resize(converted.height * converted.row_step);
  }

private:
  void image_callback(iceoryx_msg::msg::PointCloud640::SharedPtr point_cloud)
  {
    memcpy(
      &converted.data[0], &point_cloud->data,
      converted.width * converted.height * converted.point_step);
    publisher_->publish(converted);
  }
  sensor_msgs::msg::PointCloud2 converted;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<iceoryx_msg::msg::PointCloud640>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IceoryxConverterPointCloud>());
  rclcpp::shutdown();
  return 0;
}
