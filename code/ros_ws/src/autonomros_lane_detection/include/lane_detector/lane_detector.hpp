#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include <iostream>
#include <sstream>
#include <math.h>
#include <cstdlib>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>

#include "iceoryx_msg/msg/image640rgb.hpp"
#include "iceoryx_msg/msg/lane_points.hpp"

#ifndef LANE_DETECTOR__LANE_DETECTOR_HPP_
#define LANE_DETECTOR__LANE_DETECTOR_HPP_

#define DEBUG_LINE_THICKNESS 10
#define DEBUG_MASK 0
#define DEBUG_BIRD_VIEW 0
#define DEBUG_FIT_LINE 0
#define DEBUG_TRAJECTORY_LINE 0
#define DEBUG_TRAJECTORY_IMAGE 0
#define DEBUG_COLOR_BALANCING 0

#define CAR 1

class Lane_Detector : public rclcpp::Node
{
public:
  Lane_Detector();
  void init();

private:
  void threshold_white(cv::Mat & input_image, cv::Mat & mask);
  void threshold_yellow(cv::Mat & input_image, cv::Mat & mask);

  void bird_view(cv::Mat & mask, cv::Mat & top_view);
  std::vector<double> fit_line(cv::Mat & top_view);
  std::vector<double> calculate_trajectory(std::vector<cv::Point2d> & points);

  std::vector<cv::Point2d> calculate_trajectory_points(
    std::vector<double> & coefficients,
    bool yellow_line);
  std::vector<cv::Point2d> calculate_line_points(std::vector<double> & coefficients);
  std::vector<cv::Point2d> bird_view_back_points(std::vector<cv::Point2d> & points);

  cv::Point3f calculate_transform(int u, int v);
  void topic_callback(const iceoryx_msg::msg::Image640rgb::SharedPtr msg);

  cv::Mat cameraMatrix, distCoeffs, rotationVector;
  cv::Mat rotationMatrix, translationVector;
  cv::Mat invR_x_invM_x_uv1, invR_x_tvec, wcPoint;


  double Z = 0;
  double s = 0;

  std::vector<cv::Point2d> image_points;
  std::vector<cv::Point3d> world_points;


  int image_width;
  int image_height;

  //Parameter
  //Hue-Saturation-Lightness
  // Simulator
  #if !CAR
  std::vector<double> lower_white = {0, 0, 215};
  std::vector<double> upper_white = {179, 35, 255};
  std::vector<double> lower_yellow = {10, 70, 95};
  std::vector<double> upper_yellow = {100, 255, 255};
  #endif

  // Car camera
  #if CAR
  std::vector<double> lower_white = {70, 0, 195};
  std::vector<double> upper_white = {90, 87, 255};
  std::vector<double> lower_yellow = {30, 95, 50};
  std::vector<double> upper_yellow = {34, 255, 255};
  #endif

  //ROI (later as real parameters)
  // Simulator
  #if !CAR
  std::vector<cv::Point2f> roi_points = {cv::Point2f(-268, 480), cv::Point2f(908, 480), cv::Point2f(
      236, 268), cv::Point2f(404, 268)};
  #endif
  // Car
  #if CAR
  std::vector<cv::Point2f> roi_points = {cv::Point2f(-361, 480), cv::Point2f(996, 480), cv::Point2f(
      218, 221), cv::Point2f(418, 221)};
  #endif
  int padding = 0.25 * 640;
  std::vector<cv::Point2f> desired_roi_points = {
    cv::Point2f(padding, 480),
    cv::Point2f(640 - padding, 480),
    cv::Point2f(padding, 0),
    cv::Point2f(640 - padding, 0)
  };
  cv::Mat transformation_matrix = cv::getPerspectiveTransform(roi_points, desired_roi_points);
  cv::Mat transformation_matrix_back = cv::getPerspectiveTransform(desired_roi_points, roi_points);

  int number_of_points = 30;
  double pixels_into_line = 150.0;

  //sub&pubs
  rclcpp::Subscription<iceoryx_msg::msg::Image640rgb>::SharedPtr camera_image_subscriber_;
  rclcpp::Publisher<iceoryx_msg::msg::LanePoints>::SharedPtr central_line_publisher_;

  #if DEBUG_COLOR_BALANCING
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_balancing_publisher_;
  #endif

  #if DEBUG_MASK
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_publisher_;
  #endif

  #if DEBUG_BIRD_VIEW
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr bird_view_publisher_;
  #endif

  #if DEBUG_FIT_LINE
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr fit_line_publisher_;
  #endif

  #if DEBUG_TRAJECTORY_LINE
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr trajectory_line_publisher_;
  #endif

  #if DEBUG_TRAJECTORY_IMAGE
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr trajectory_image_publisher_;
  #endif
};

#endif /* LANE_DETECTOR__LANE_DETECTOR_HPP_ */
