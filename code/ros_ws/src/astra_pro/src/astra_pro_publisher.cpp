#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "astra_pro/astra_pro_publisher.hpp"
#include "sensor_msgs/distortion_models.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"


AstraProPublisher::AstraProPublisher()
: LifecycleNode("astra_pro_publisher")
{
}

AstraProPublisher::~AstraProPublisher()
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn AstraProPublisher::
on_configure(const rclcpp_lifecycle::State &)
{
  // Workaround, because these parameters cannot be undeclared
  if (!this->has_parameter("frame_id")) {
    this->declare_parameter<std::string>("frame_id", "depth_camera_link");
  }
  if (!this->has_parameter("image_width")) {
    this->declare_parameter<int64_t>("image_width", 160);
  }
  if (!this->has_parameter("image_height")) {
    this->declare_parameter<int64_t>("image_height", 120);
  }
  if (!this->has_parameter("pointcloud")) {
    this->declare_parameter<bool>("pointcloud", true);
  }
  RCLCPP_DEBUG(get_logger(), "Parameters declared");

  this->get_parameter<std::string>("frame_id", frame_id_);
  this->get_parameter<int64_t>("image_width", image_width_);
  this->get_parameter<int64_t>("image_height", image_height_);
  this->get_parameter("pointcloud", pointcloud_);
  RCLCPP_DEBUG(get_logger(), "Parameters configured");

  image_publisher_ =
    this->create_publisher<RGB_IMAGE_MSG_TYPE>(
    "image_raw",
    1);
  image_info_publisher_ =
    this->create_publisher<CAMERA_INFO_MSG_TYPE>(
    "camera_info",
    1);
  depth_image_publisher_ = this->create_publisher<DEPTH_IMAGE_MSG_TYPE>(
    "depth/image_raw", 1);
  depth_image_info_publisher_ =
    this->create_publisher<CAMERA_INFO_MSG_TYPE>(
    "depth/camera_info",
    1);
  if (pointcloud_) {
    point_cloud_publisher_ = this->create_publisher<POINT_CLOUD_MSG_TYPE>(
      "points", 1);
  }
  RCLCPP_DEBUG(get_logger(), "Publishers created");

  astra::initialize();
  RCLCPP_DEBUG(get_logger(), "Astra initialized");
  astra_stream_set_ = std::make_unique<astra::StreamSet>();
  if (!astra_stream_set_->is_available()) {
    RCLCPP_ERROR(get_logger(), "No camera connected");
    astra_stream_set_.reset();
    astra::terminate();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }
  astra_reader_ = std::make_unique<astra::StreamReader>(astra_stream_set_->create_reader());
  // astra_color_stream_ = std::make_unique<astra::ColorStream>(
  //   astra_reader_->stream<astra::ColorStream>());
  // astra_color_stream_->enable_mirroring(false);
  // astra::ImageStreamMode color_steam_mode(astra_color_stream_->mode());
  // color_steam_mode.set_width(image_width_);
  // color_steam_mode.set_height(image_height_);
  // astra_color_stream_->set_mode(color_steam_mode);
  astra_depth_stream_ = std::make_unique<astra::DepthStream>(
    astra_reader_->stream<astra::DepthStream>());
  astra_depth_stream_->enable_mirroring(false);
  astra::ImageStreamMode depth_steam_mode(astra_depth_stream_->mode());
  depth_steam_mode.set_width(image_width_);
  depth_steam_mode.set_height(image_height_);
  depth_steam_mode.set_fps(30);
  astra_depth_stream_->set_mode(depth_steam_mode);
  RCLCPP_DEBUG(get_logger(), "Astra configured");

  astra::DeviceController astra_device_controller(*astra_stream_set_);
  astra_device_controller.get_orbbec_camera_params(astra_camera_params_);
  astra_pro_frame_listener_ = std::make_unique<AstraProFrameListener>(
    pointcloud_,
    std::bind(
      &AstraProPublisher::handle_color_image, this, std::placeholders::_1),
    std::bind(
      &AstraProPublisher::handle_depth_image, this, std::placeholders::_1),
    std::bind(
      &AstraProPublisher::generate_point_cloud, this, std::placeholders::_1,
      std::placeholders::_2)
  );
  astra_reader_->add_listener(*astra_pro_frame_listener_);
  RCLCPP_DEBUG(get_logger(), "Listener added");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn AstraProPublisher::
on_activate(const rclcpp_lifecycle::State &)
{
  image_publisher_->on_activate();
  image_info_publisher_->on_activate();
  depth_image_publisher_->on_activate();
  depth_image_info_publisher_->on_activate();
  if (pointcloud_) {
    point_cloud_publisher_->on_activate();
  }
  RCLCPP_DEBUG(get_logger(), "Publishers activated");

  // astra_color_stream_->start();
  astra_depth_stream_->start();
  RCLCPP_DEBUG(get_logger(), "Streams started");

  running_ = true;
  update_thread_ = std::thread(&AstraProPublisher::update_loop, this);
  RCLCPP_DEBUG(get_logger(), "Update thread started");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn AstraProPublisher::
on_deactivate(const rclcpp_lifecycle::State &)
{
  running_ = false;
  update_thread_.join();
  // astra_color_stream_->stop();
  astra_depth_stream_->stop();
  RCLCPP_DEBUG(get_logger(), "Streams stopped");

  image_publisher_->on_deactivate();
  image_info_publisher_->on_deactivate();
  depth_image_publisher_->on_deactivate();
  depth_image_info_publisher_->on_deactivate();
  if (pointcloud_) {
    point_cloud_publisher_->on_deactivate();
  }
  RCLCPP_DEBUG(get_logger(), "Publishers deactivated");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn AstraProPublisher::
on_cleanup(const rclcpp_lifecycle::State &)
{
  astra_reader_->remove_listener(*astra_pro_frame_listener_);
  astra_pro_frame_listener_.reset();
  RCLCPP_DEBUG(get_logger(), "Listener removed");

  astra_depth_stream_.reset();
  // astra_color_stream_.reset();
  astra_reader_.reset();
  astra_stream_set_.reset();

  astra::terminate();
  RCLCPP_DEBUG(get_logger(), "Astra terminated");

  image_publisher_.reset();
  image_info_publisher_.reset();
  depth_image_publisher_.reset();
  depth_image_info_publisher_.reset();
  point_cloud_publisher_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn AstraProPublisher::
on_shutdown(const rclcpp_lifecycle::State &)
{
  if (running_) {
    RCLCPP_DEBUG(get_logger(), "Still running, so stopping");
    running_ = false;
    update_thread_.join();
  }
  if (astra_stream_set_) {
    RCLCPP_DEBUG(get_logger(), "No cleanup yet, so doing it now");
    astra_reader_->remove_listener(*astra_pro_frame_listener_);
    astra::terminate();
  }
  astra_pro_frame_listener_.reset();

  astra_depth_stream_.reset();
  // astra_color_stream_.reset();
  astra_reader_.reset();
  astra_stream_set_.reset();

  image_publisher_.reset();
  image_info_publisher_.reset();
  depth_image_publisher_.reset();
  depth_image_info_publisher_.reset();
  point_cloud_publisher_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void AstraProPublisher::handle_color_image(const astra::ColorFrame & color_image_frame)
{
  // Color image
  auto image_loan = image_publisher_->borrow_loaned_message();
  auto & image = image_loan.get();
  // - Copy
  auto data_size = color_image_frame.byte_length();
  memcpy(&image.data[0], color_image_frame.data(), data_size);
  // - Publish (somehow LifecyclePublisher does not support it)
  image_publisher_->Publisher::publish(std::move(image_loan));

  // Camera Info
  auto camera_info_loan = image_info_publisher_->borrow_loaned_message();
  auto & camera_info = camera_info_loan.get();
  // - Init
  camera_info.header.frame_id = frame_id_;
  camera_info.width = image_width_;
  camera_info.height = image_height_;
  camera_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  // -- Intrinsic camera matrix for color camera
  camera_info.k.fill(0);
  camera_info.k[0] = astra_camera_params_.r_intr_p[0];  // fx
  camera_info.k[2] = astra_camera_params_.r_intr_p[2];  // cx
  camera_info.k[4] = astra_camera_params_.r_intr_p[1];  // fy
  camera_info.k[5] = astra_camera_params_.r_intr_p[3];  // cy
  camera_info.k[8] = 1.0;
  // -- distortion parameters for color camera
  camera_info.d.assign(5, 0.0);
  for (int i = 0; i < 5; ++i) {
    camera_info.d[i] = astra_camera_params_.r_k[i];
  }
  // -- Rectification matrix for color camera (get from right to left camera)
  camera_info.r.fill(0.0);
  for (int i = 0; i < 9; ++i) {
    camera_info.r[i] = astra_camera_params_.r2l_r[i];
  }
  // -- Projection matrix for color camera
  camera_info.p.fill(0.0);
  camera_info.p[0] = astra_camera_params_.r_intr_p[0];  // fx
  camera_info.p[2] = astra_camera_params_.r_intr_p[2];  // cx
  camera_info.p[3] = astra_camera_params_.r2l_t[0];  // t1 / Tx
  camera_info.p[5] = astra_camera_params_.r_intr_p[1];  // fy
  camera_info.p[6] = astra_camera_params_.r_intr_p[3];  // cy
  camera_info.p[7] = astra_camera_params_.r2l_t[1];  // t2 / Ty
  camera_info.p[10] = 1.0;
  camera_info.p[11] = astra_camera_params_.r2l_t[2];  // t3
  // - Publish (somehow LifecyclePublisher does not support it)
  image_info_publisher_->Publisher::publish(std::move(camera_info_loan));
}

void AstraProPublisher::handle_depth_image(const astra::DepthFrame & depth_image_frame)
{
  // Depth image
  auto image_loan = depth_image_publisher_->borrow_loaned_message();
  auto & image = image_loan.get();
  // - Copy
  auto data_size = depth_image_frame.byte_length();
  memcpy(&image.data[0], depth_image_frame.data(), data_size);
  // - Publish (somehow LifecyclePublisher does not support it)
  depth_image_publisher_->Publisher::publish(std::move(image_loan));

  // Camera Info
  auto camera_info_loan = depth_image_info_publisher_->borrow_loaned_message();
  auto & camera_info = camera_info_loan.get();
  // - Init
  camera_info.header.frame_id = frame_id_;
  camera_info.width = image_width_;
  camera_info.height = image_height_;
  camera_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  // -- Intrinsic camera matrix for depth camera
  camera_info.k.fill(0.0);
  camera_info.k[0] = astra_camera_params_.l_intr_p[0];  // fx
  camera_info.k[2] = astra_camera_params_.l_intr_p[2];  // cx
  camera_info.k[4] = astra_camera_params_.l_intr_p[1];  // fy
  camera_info.k[5] = astra_camera_params_.l_intr_p[3];  // cy
  camera_info.k[8] = 1.0;
  // -- distortion parameters for depth camera
  camera_info.d.assign(5, 0.0);
  for (int i = 0; i < 5; ++i) {
    camera_info.d[i] = astra_camera_params_.l_k[i];
  }
  // -- Rectification matrix for depth camera (identity)
  camera_info.r.fill(0.0);
  camera_info.r[0] = 1.0;
  camera_info.r[4] = 1.0;
  camera_info.r[8] = 1.0;
  // -- Projection matrix for depth camera
  camera_info.p.fill(0.0);
  camera_info.p[0] = astra_camera_params_.l_intr_p[0];  // fx
  camera_info.p[2] = astra_camera_params_.l_intr_p[2];  // cx
  camera_info.p[5] = astra_camera_params_.l_intr_p[1];  // fy
  camera_info.p[6] = astra_camera_params_.l_intr_p[3];  // cy
  camera_info.p[10] = 1.0;
  // - Publish (somehow LifecyclePublisher does not support it)
  depth_image_info_publisher_->Publisher::publish(std::move(camera_info_loan));
}

void AstraProPublisher::generate_point_cloud(
  const astra::ColorFrame & color_image_frame,
  const astra::DepthFrame & depth_image_frame)
{
  auto point_cloud_loan = point_cloud_publisher_->borrow_loaned_message();
  auto & point_cloud = point_cloud_loan.get();

  sensor_msgs::PointCloud2Modifier point_cloud_modifier(point_cloud);

  point_cloud.header.frame_id = frame_id_;
  point_cloud.width = image_width_;
  point_cloud.height = image_height_;
  point_cloud_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

  sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(point_cloud, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(point_cloud, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(point_cloud, "b");
  for (size_t i = 0;
    i < depth_image_frame.length() && i < color_image_frame.length() &&
    i < point_cloud_modifier.size();
    ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
  {
    float x, y, z = 0;
    astra_depth_stream_->coordinateMapper().convert_depth_to_world(
      i % point_cloud.width,
      i / point_cloud.width,
      depth_image_frame.data()[i],
      x,
      y,
      z
    );
    *iter_x = z * 0.001f;
    *iter_y = -x * 0.001f;
    *iter_z = y * 0.001f;
    *iter_r = color_image_frame.data()[i].r;
    *iter_g = color_image_frame.data()[i].g;
    *iter_b = color_image_frame.data()[i].b;
  }

  point_cloud_publisher_->Publisher::publish(std::move(point_cloud_loan));
}

void AstraProPublisher::update_loop()
{
  RCLCPP_DEBUG(get_logger(), "Starting update_loop");
  do {
    astra_update();
  } while (running_);
  RCLCPP_DEBUG(get_logger(), "Stopping update_loop");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AstraProPublisher>()->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
