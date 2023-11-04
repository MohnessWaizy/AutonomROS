#include <astra/astra.hpp>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "astra_pro/message_types.hpp"
#include "astra_pro/astra_pro_frame_listener.hpp"

#ifndef ASTRA_PRO__ASTRA_PRO_PUBLISHER_HPP_
#define ASTRA_PRO__ASTRA_PRO_PUBLISHER_HPP_

class AstraProPublisher : public rclcpp_lifecycle::LifecycleNode
{
public:
  AstraProPublisher();
  ~AstraProPublisher();

  // Lifecycle
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State &) override;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State &) override;

  void handle_color_image(const astra::ColorFrame & color_image_frame);
  void handle_depth_image(const astra::DepthFrame & depth_image_frame);
  void generate_point_cloud(
    const astra::ColorFrame & color_image_frame,
    const astra::DepthFrame & depth_image_frame);

private:
  std::string frame_id_;
  int64_t image_width_;
  int64_t image_height_;
  bool pointcloud_;
  orbbec_camera_params astra_camera_params_;
  rclcpp_lifecycle::LifecyclePublisher<RGB_IMAGE_MSG_TYPE>::SharedPtr image_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<CAMERA_INFO_MSG_TYPE>::SharedPtr
    image_info_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<DEPTH_IMAGE_MSG_TYPE>::SharedPtr depth_image_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<CAMERA_INFO_MSG_TYPE>::SharedPtr
    depth_image_info_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<POINT_CLOUD_MSG_TYPE>::SharedPtr
    point_cloud_publisher_;
  std::unique_ptr<astra::StreamSet> astra_stream_set_;
  std::unique_ptr<astra::StreamReader> astra_reader_;
  // std::unique_ptr<astra::ColorStream> astra_color_stream_;
  std::unique_ptr<astra::DepthStream> astra_depth_stream_;
  std::unique_ptr<AstraProFrameListener> astra_pro_frame_listener_;
  std::thread update_thread_;
  bool running_ = false;

  void update_loop();
};

#endif  // ASTRA_PRO__ASTRA_PRO_PUBLISHER_HPP_
