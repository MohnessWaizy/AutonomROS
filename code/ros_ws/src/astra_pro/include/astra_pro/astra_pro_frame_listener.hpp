#include <astra/astra.hpp>
#include <memory>
#include <string>

#ifndef ASTRA_PRO__ASTRA_PRO_FRAME_LISTENER_HPP_
#define ASTRA_PRO__ASTRA_PRO_FRAME_LISTENER_HPP_

class AstraProFrameListener : public astra::FrameListener
{
public:
  AstraProFrameListener(
    bool pointcloud,
    std::function<void(const astra::ColorFrame & color_image_frame)> handle_color_image,
    std::function<void(const astra::DepthFrame & depth_image_frame)> handle_depth_image,
    std::function<void(const astra::ColorFrame & color_image_frame,
    const astra::DepthFrame & depth_image_frame)> generate_point_cloud
  );

  void on_frame_ready(astra::StreamReader & reader, astra::Frame & frame) override;

private:
  std::function<void(const astra::ColorFrame & color_image_frame)> handle_color_image_;
  std::function<void(const astra::DepthFrame & depth_image_frame)> handle_depth_image_;
  std::function<void(const astra::ColorFrame & color_image_frame,
    const astra::DepthFrame & depth_image_frame)> generate_point_cloud_;
  bool pointcloud;
};

#endif /* ASTRA_PRO__ASTRA_PRO_FRAME_LISTENER_HPP_ */
