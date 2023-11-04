#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "iceoryx_msg/msg/lane_points.hpp"

struct point_cloud_point {
  float x;
  float y;
  float z;
};

class IceoryxConverterLines : public rclcpp::Node
{
public:
  IceoryxConverterLines()
  : Node("iceoryx_converter_lines")
  {
    this->declare_parameter("frame_id", "base_link");
    auto qos = rclcpp::QoS(1);
    qos.best_effort();
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("line_converted", 10);
    subscription_ = this->create_subscription<iceoryx_msg::msg::LanePoints>(
      "/lane_detector/central_line/point_cloud", qos, std::bind(
        &IceoryxConverterLines::line_callback, this,
        std::placeholders::_1));

    converted.header.frame_id = this->get_parameter("frame_id").get_parameter_value().get<std::string>();
    converted.width = 30;
    converted.height = 1;

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

    converted.point_step = 12;
    converted.row_step = converted.width * converted.point_step;
    converted.data.resize(converted.height * converted.row_step);
  }

private:
  void line_callback(iceoryx_msg::msg::LanePoints::SharedPtr lines)
  {
    if (lines->valid) {
      for (int i = 0; i < 30; ++i) {
        struct point_cloud_point * point = (struct point_cloud_point *) &converted.data[i * 12];
        point->x = lines->points.at(i).x;
        point->y = lines->points.at(i).y;
        point->z = 0;
      }
    } else {
      for (int i = 0; i < 30; ++i) {
        struct point_cloud_point * point = (struct point_cloud_point *) &converted.data[i * 12];
        point->x = 0;
        point->y = 0;
        point->z = 0;
      }
    }
    publisher_->publish(converted);
  }
  sensor_msgs::msg::PointCloud2 converted;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<iceoryx_msg::msg::LanePoints>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IceoryxConverterLines>());
  rclcpp::shutdown();
  return 0;
}
