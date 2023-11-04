#include <chrono>
#include <cmath>
#include <memory>
#include "carcontroller/carcontroller.hpp"

using namespace std::chrono_literals;

Carcontroller::Carcontroller()
: Node("carcontroller")
{
}

void Carcontroller::init()
{
  pid_controller_ = std::make_unique<control_toolbox::PidROS>(this->shared_from_this());
  pid_controller_->initPid(1, 0, 0, 10, 0, false);
  while (!pid_controller_->initPid() && rclcpp::ok()) {
    rclcpp::sleep_for(1s);
  }

  emergency_service_ =
    this->create_service<std_srvs::srv::SetBool>(
    "emergency",
    std::bind(&Carcontroller::set_emergency, this, std::placeholders::_1, std::placeholders::_2));
  odometry_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
    "odometry/filtered",
    rclcpp::QoS(
      rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data),
      rmw_qos_profile_sensor_data),
    std::bind(&Carcontroller::update_odometry, this, std::placeholders::_1)
  );
  setpoint_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel",
    10,
    std::bind(&Carcontroller::update_setpoint, this, std::placeholders::_1)
  );
  command_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/modelcar/drive_cmd", 10);

  last_call_ = this->now();
  setpoint_last_ = this->now();
  odometry_last_ = this->now();
  timer_ = create_wall_timer(33ms, std::bind(&Carcontroller::control_cycle, this));
}

void Carcontroller::control_cycle()
{
  auto current_emergency_time = this->now();
  if (current_emergency_time - setpoint_last_ > rclcpp::Duration(1s) ||
    current_emergency_time - odometry_last_ > rclcpp::Duration(1s) ||
    emergency)
  {
    if (!emergency_msg_) {
      RCLCPP_WARN(get_logger(), "Emergency detected");
      emergency_msg_ = true;
    }
    geometry_msgs::msg::Twist tmp_;
    tmp_.angular.z = 0.0;
    tmp_.linear.x = 0.0;
    command_publisher_->publish(tmp_);
    return;
  }
  if (emergency_msg_) {
    RCLCPP_WARN(get_logger(), "Resume to normal operation");
    emergency_msg_ = false;
    pid_controller_->reset();
    last_call_ = this->now();
  }
  double velocity = std::sqrt(
    std::pow(odometry_msg_.twist.twist.linear.x, 2) +
    std::pow(odometry_msg_.twist.twist.linear.y, 2) +
    std::pow(odometry_msg_.twist.twist.linear.z, 2)
  );
  double error = setpoint_msg_.linear.x - velocity;
  auto current_time = this->now();
  double command_vel = pid_controller_->computeCommand(
    error,
    current_time - last_call_
  );
  last_call_ = current_time;
  if (command_vel > 1) {
    RCLCPP_WARN(get_logger(), "Too high value");
    command_vel = 1;
  } else if (command_vel < -1) {
    RCLCPP_WARN(get_logger(), "Too low value");
    command_vel = -1;
  } else if (command_vel == 0) {
    // avoid driving backwards by never go into neutral
    command_vel = 0.0;
  }
  geometry_msgs::msg::Twist tmp_;
  tmp_.angular.z = setpoint_msg_.angular.z / setpoint_msg_.linear.x;
  tmp_.linear.x = command_vel;
  command_publisher_->publish(tmp_);
}

void Carcontroller::update_odometry(const nav_msgs::msg::Odometry & msg)
{
  odometry_msg_ = msg;
  odometry_last_ = this->now();
}

void Carcontroller::update_setpoint(const geometry_msgs::msg::Twist & msg)
{
  setpoint_msg_ = msg;
  setpoint_last_ = this->now();
}

void Carcontroller::set_emergency(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response
)
{
  emergency = request->data;
  response->success = true;
  response->message = "";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto controller = std::make_shared<Carcontroller>();
  controller->init();
  rclcpp::spin(controller);
  rclcpp::shutdown();
  return 0;
}
