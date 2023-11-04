#include <memory.h>
#include <memory>
#include <string>

#include "astra_pro/astra_pro_frame_listener.hpp"


AstraProFrameListener::AstraProFrameListener(
  bool pointcloud,
  std::function<void(const astra::ColorFrame & color_image_frame)> handle_color_image,
  std::function<void(const astra::DepthFrame & depth_image_frame)> handle_depth_image,
  std::function<void(const astra::ColorFrame & color_image_frame,
  const astra::DepthFrame & depth_image_frame)> generate_point_cloud
)
: pointcloud(pointcloud),
  handle_color_image_(handle_color_image),
  handle_depth_image_(handle_depth_image),
  generate_point_cloud_(generate_point_cloud)
{
}

void AstraProFrameListener::on_frame_ready(astra::StreamReader & reader, astra::Frame & frame)
{
  const astra::ColorFrame color_image_frame = frame.get<astra::ColorFrame>();
  if (color_image_frame.is_valid()) {
    handle_color_image_(color_image_frame);
  }

  const astra::DepthFrame depth_image_frame = frame.get<astra::DepthFrame>();
  if (depth_image_frame.is_valid()) {
    handle_depth_image_(depth_image_frame);
  }

  if (pointcloud && color_image_frame.is_valid() && depth_image_frame.is_valid()) {
    generate_point_cloud_(color_image_frame, depth_image_frame);
  }
}
