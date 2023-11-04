#include "lane_detector/lane_detector.hpp"
#include <cmath>
#include "std_msgs/msg/header.hpp"
#include <armadillo>

Lane_Detector::Lane_Detector()
: Node("lane_detector")
{
}

void Lane_Detector::init()
{
  RCLCPP_INFO(this->get_logger(), "start lane_detector");

  //either get the parameter in opencv yaml file, therefore uncomment code after cv::Filestorage and give the correct path for the yaml files here
  auto intrinsics_path_ = this->declare_parameter<std::string>(
    "intrinsics_path",
    "install/autonomros_lane_detection/share/autonomros_lane_detection/params/intrinsics.yml");
  auto pointsets_path_ = this->declare_parameter<std::string>(
    "pointsets_path",
    "install/autonomros_lane_detection/share/autonomros_lane_detection/params/pointsets.yml");
  RCLCPP_INFO(this->get_logger(), "Path intrinsics: %s", intrinsics_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "Path pointsets: %s", pointsets_path_.c_str());

  //init pub&subs
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos = qos.best_effort();
  camera_image_subscriber_ = this->create_subscription<iceoryx_msg::msg::Image640rgb>(
    "/camera_sensor1/image_raw", qos,
    std::bind(&Lane_Detector::topic_callback, this, std::placeholders::_1));
  central_line_publisher_ = this->create_publisher<iceoryx_msg::msg::LanePoints>(
    "lane_detector/central_line/point_cloud", 1);

  //Load intrinsics and points.
  RCLCPP_INFO(this->get_logger(), "Try load intrinsics & points");

  cv::FileStorage intrin(intrinsics_path_.c_str(), cv::FileStorage::READ);
  cv::FileStorage points(pointsets_path_.c_str(), cv::FileStorage::READ);
  intrin["camera_matrix"] >> cameraMatrix;
  intrin["distortion_coefficients"] >> distCoeffs;
  intrin["image_height"] >> image_height;
  intrin["image_width"] >> image_width;
  points["imagepoints"] >> image_points;
  points["worldpoints"] >> world_points;
  // Generate our matrix; rvec and tvec are the output.
  RCLCPP_INFO(this->get_logger(), "calculate rotation & translation Vector & rotationMatrix");
  solvePnP(
    world_points, image_points, this->cameraMatrix, distCoeffs, rotationVector,
    translationVector);
  Rodrigues(rotationVector, rotationMatrix);
  this->rotationMatrix = this->rotationMatrix.inv();
  this->cameraMatrix = this->cameraMatrix.inv();
  this->invR_x_tvec = this->rotationMatrix * this->translationVector;

  #if DEBUG_MASK
  mask_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
    "lane_detector/debug/mask", 10);
  #endif

  #if DEBUG_BIRD_VIEW
  bird_view_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
    "lane_detector/debug/bird_view", 10);
  #endif

  #if DEBUG_FIT_LINE
  fit_line_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
    "lane_detector/debug/fit_line", 10);
  #endif

  #if DEBUG_TRAJECTORY_LINE
  trajectory_line_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
    "lane_detector/debug/trajectory_line", 10);
  #endif

  #if DEBUG_TRAJECTORY_IMAGE
  trajectory_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
    "lane_detector/debug/trajectory_image", 10);
  #endif
}

void Lane_Detector::threshold_white(cv::Mat & input_image, cv::Mat & mask)
{
  cv::inRange(input_image, lower_white, upper_white, mask);
}

void Lane_Detector::threshold_yellow(cv::Mat & input_image, cv::Mat & mask)
{
  cv::inRange(input_image, lower_yellow, upper_yellow, mask);
}

void Lane_Detector::bird_view(cv::Mat & mask, cv::Mat & top_view)
{
  cv::warpPerspective(mask, top_view, transformation_matrix, cv::Size(640, 480));
}

std::vector<double> Lane_Detector::fit_line(cv::Mat & top_view)
{
  std::vector<cv::Point2i> points;
  cv::findNonZero(top_view, points);
  std::vector<double> xValues;
  std::vector<double> yValues;
  std::vector<double> weights;
  for (cv::Point2i point : points) {
    xValues.push_back(point.y);
    yValues.push_back(point.x);
    weights.push_back(std::pow(point.y, 8));
  }
  arma::vec res = arma::polyfit(
    arma::conv_to<arma::vec>::from(
      xValues), arma::conv_to<arma::vec>::from(yValues), 2);
  std::vector<double> result;
  result.push_back(res[0]);
  result.push_back(res[1]);
  result.push_back(res[2]);
  return result;
}

std::vector<double> Lane_Detector::calculate_trajectory(std::vector<cv::Point2d> & points)
{
  std::vector<double> xValues;
  std::vector<double> yValues;
  for (cv::Point2d point : points) {
    xValues.push_back(point.x);
    yValues.push_back(point.y);
  }
  for (int i = 0; i < 60; ++i) {
    xValues.push_back(480);
    yValues.push_back(320);
    xValues.push_back(470);
    yValues.push_back(320);
    xValues.push_back(460);
    yValues.push_back(320);
  }
  arma::vec res = arma::polyfit(
    arma::conv_to<arma::vec>::from(
      xValues), arma::conv_to<arma::vec>::from(yValues), 3);
  std::vector<double> result;
  result.push_back(res[0]);
  result.push_back(res[1]);
  result.push_back(res[2]);
  result.push_back(res[3]);
  return result;
}

std::vector<cv::Point2d> Lane_Detector::calculate_trajectory_points(
  std::vector<double> & coefficients, bool yellow_line)
{
  std::vector<cv::Point2d> points;
  for (int i = 0; i < number_of_points; ++i) {
    double x = (double) i * (480.0 / number_of_points);
    double y = coefficients[0] * std::pow(x, 2) + coefficients[1] * x + coefficients[2];
    double slope = 2.0 * coefficients[0] * x + coefficients[1];

    double factor = 1.0;
    if (yellow_line) {
      factor = -1.0;
    }

    double x_s = factor * (slope * pixels_into_line) / std::sqrt(std::pow(slope, 2) + 1);
    double y_s = -(1.0 / slope) * x_s;
    points.push_back(cv::Point2d(x + x_s, y + y_s));
  }
  return points;
}

std::vector<cv::Point2d> Lane_Detector::calculate_line_points(
  std::vector<double> & coefficients)
{
  std::vector<cv::Point2d> points;
  for (int i = 0; i < number_of_points; ++i) {
    double x = i * (480.0 / number_of_points);
    if (coefficients.size() == 3) {
      double y = coefficients[0] * std::pow(x, 2) + coefficients[1] * x + coefficients[2];
      points.push_back(cv::Point2d(x, y));
    }
    if (coefficients.size() == 4) {
      double y = coefficients[0] *
        std::pow(x, 3) + coefficients[1] * std::pow(x, 2) + coefficients[2] * x + coefficients[3];
      points.push_back(cv::Point2d(x, y));
    }
  }
  return points;
}

std::vector<cv::Point2d> Lane_Detector::bird_view_back_points(std::vector<cv::Point2d> & points)
{
  std::vector<cv::Point2d> corrected_points;
  for (cv::Point2d point : points) {
    // https://stackoverflow.com/questions/55656057/how-to-use-cv2-warpperspective-with-only-one-source-point-x-y
    double d =
      transformation_matrix_back.at<double>(2, 0) * point.y +
      transformation_matrix_back.at<double>(2, 1) * point.x +
      transformation_matrix_back.at<double>(2, 2);
    corrected_points.push_back(
      cv::Point2d(
        (transformation_matrix_back.at<double>(
          0,
          0) * point.y +
        transformation_matrix_back.at<double>(
          0,
          1) * point.x + transformation_matrix_back.at<double>(0, 2)) / d,
        (transformation_matrix_back.at<double>(
          1,
          0) * point.y +
        transformation_matrix_back.at<double>(
          1,
          1) * point.x + transformation_matrix_back.at<double>(1, 2)) / d
      )
    );
  }
  return corrected_points;
}

cv::Point3f Lane_Detector::calculate_transform(int u, int v)
{
  cv::Mat screenCoordinates = cv::Mat::ones(3, 1, cv::DataType<double>::type);
  screenCoordinates.at<double>(0, 0) = u;
  screenCoordinates.at<double>(1, 0) = v;
  screenCoordinates.at<double>(2, 0) = 1;               // f=1

  static auto tmp_1 = this->rotationMatrix * this->cameraMatrix;
  static auto tmp_2 = this->invR_x_tvec.at<double>(2, 0);

  // s and point calculation, described here:
  // https://stackoverflow.com/questions/12299870/computing-x-y-coordinate-3d-from-image-point
  this->invR_x_invM_x_uv1 = tmp_1 * screenCoordinates;
  this->s = tmp_2 / this->invR_x_invM_x_uv1.at<double>(2, 0);
  this->wcPoint = this->rotationMatrix *
    (this->s * this->cameraMatrix * screenCoordinates - this->translationVector);
  cv::Point3f worldCoordinates(this->wcPoint.at<double>(0, 0), this->wcPoint.at<double>(
      1,
      0), this->wcPoint.at<double>(
      2, 0));

  return worldCoordinates;
}

void Lane_Detector::topic_callback(const iceoryx_msg::msg::Image640rgb::SharedPtr msg)
{
  cv_bridge::CvImagePtr cv_ptr;

  sensor_msgs::msg::Image image;
  image.encoding = "bgr8";
  image.width = 640;
  image.height = 480;
  image.step = image.width * 3;
  image.data.resize(image.width * image.height * 3);
  memcpy(&image.data[0], &msg->data[0], image.width * image.height * 3);

  cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);

  //convert input image from BGR to HSV color scheme
  cv::Mat hsv;
  cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);

  bool yellow_line = true;
  cv::Mat mask;
  threshold_yellow(hsv, mask);

  cv::Mat top_view;
  bird_view(mask, top_view);

  //check for enough yellow points
  //else use white lane
  if (cv::countNonZero(top_view) < 2000) {
    yellow_line = false;
    threshold_white(hsv, mask);
    bird_view(mask, top_view);
  }

  //check for enough white points
  //else no line detected
  if (cv::countNonZero(top_view) < 1000) {
    RCLCPP_INFO(this->get_logger(), "NO LINE");
    return;
  }

  #if DEBUG_MASK
  sensor_msgs::msg::Image mask_img;
  auto mask_img_bridge = cv_bridge::CvImage(
    std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, mask);
  mask_img_bridge.toImageMsg(mask_img);
  mask_publisher_->publish(mask_img);
  #endif

  #if DEBUG_BIRD_VIEW
  sensor_msgs::msg::Image bird_view_img;
  auto bird_view_img_bridge = cv_bridge::CvImage(
    std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, top_view);
  bird_view_img_bridge.toImageMsg(bird_view_img);
  bird_view_publisher_->publish(bird_view_img);
  #endif

  std::vector<double> coefficients = fit_line(top_view);

  #if DEBUG_FIT_LINE
  std::vector<cv::Point2d> line_points = calculate_line_points(coefficients);
  std::vector<cv::Point2i> cv_line_points;
  for (cv::Point2d & point : line_points) {
    cv_line_points.push_back(cv::Point2i(point.y, point.x));
  }
  cv::Mat fit_line_mat;
  cv::cvtColor(top_view, fit_line_mat, cv::COLOR_GRAY2RGB);
  cv::polylines(fit_line_mat, cv_line_points, false, cv::Scalar(0, 152, 0), DEBUG_LINE_THICKNESS);
  sensor_msgs::msg::Image fit_line_img;
  auto fit_line_img_bridge = cv_bridge::CvImage(
    std_msgs::msg::Header(), sensor_msgs::image_encodings::RGB8, fit_line_mat);
  fit_line_img_bridge.toImageMsg(fit_line_img);
  fit_line_publisher_->publish(fit_line_img);
  #endif

  std::vector<cv::Point2d> points = calculate_trajectory_points(coefficients, yellow_line);
  std::vector<double> coefficients_trajectory = calculate_trajectory(points);
  points = calculate_line_points(coefficients_trajectory);

  #if DEBUG_TRAJECTORY_LINE
  std::vector<cv::Point2i> cv_points;
  for (cv::Point2d & point : points) {
    cv_points.push_back(cv::Point2i(point.y, point.x));
  }
  cv::Mat trajectory_line_mat;
  cv::cvtColor(top_view, trajectory_line_mat, cv::COLOR_GRAY2RGB);
  cv::polylines(trajectory_line_mat, cv_points, false, cv::Scalar(0, 152, 0), DEBUG_LINE_THICKNESS);
  sensor_msgs::msg::Image trajectory_line_img;
  auto trajectory_line_img_bridge = cv_bridge::CvImage(
    std_msgs::msg::Header(), sensor_msgs::image_encodings::RGB8, trajectory_line_mat);
  trajectory_line_img_bridge.toImageMsg(trajectory_line_img);
  trajectory_line_publisher_->publish(trajectory_line_img);
  #endif

  std::vector<cv::Point2d> corrected_points = bird_view_back_points(points);

  #if DEBUG_TRAJECTORY_IMAGE
  std::vector<cv::Point2i> cv_corrected_points;
  for (cv::Point2d & point : corrected_points) {
    cv_corrected_points.push_back(cv::Point2i(point.x, point.y));
  }
  cv::polylines(
    cv_ptr->image, cv_corrected_points, false, cv::Scalar(
      0, 152,
      0), DEBUG_LINE_THICKNESS);
  sensor_msgs::msg::Image trajectory_image_img;
  auto trajectory_image_img_bridge = cv_bridge::CvImage(
    std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, cv_ptr->image);
  trajectory_image_img_bridge.toImageMsg(trajectory_image_img);
  trajectory_image_publisher_->publish(trajectory_image_img);
  #endif

  auto message_loan = central_line_publisher_->borrow_loaned_message();
  auto & central_line_msg = message_loan.get();
  for (int i = 0; i < 30; ++i) {
    auto & point = corrected_points.at(i);
    auto & msg_point = central_line_msg.points.at(i);
    cv::Point3f base_link_point = calculate_transform(point.y, point.x);
    msg_point.x = base_link_point.x;
    msg_point.y = base_link_point.y;
  }
  
  central_line_msg.valid = true;
  central_line_publisher_->publish(std::move(message_loan));
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto detector = std::make_shared<Lane_Detector>();
  detector->init();
  rclcpp::spin(detector);
  rclcpp::shutdown();
  return 0;

}
