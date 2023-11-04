#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "control_toolbox/pid_ros.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/set_bool.hpp"

#ifndef CARCONTROLLER__CARCONTROLLER_HPP_
#define CARCONTROLLER__CARCONTROLLER_HPP_

class Carcontroller : public rclcpp::Node
{
public:
  Carcontroller();
  void init();

private:
  void control_cycle();
  void update_odometry(const nav_msgs::msg::Odometry & msg);
  void update_setpoint(const geometry_msgs::msg::Twist & msg);
  void set_emergency(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response
  );

  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Odometry odometry_msg_;
  rclcpp::Time odometry_last_;
  geometry_msgs::msg::Twist setpoint_msg_;
  rclcpp::Time setpoint_last_;

  std::unique_ptr<control_toolbox::PidROS> pid_controller_;
  rclcpp::Time last_call_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr emergency_service_;
  bool emergency = false;
  bool emergency_msg_ = false;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr setpoint_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
};

#endif /* CARCONTROLLER__CARCONTROLLER_HPP_ */
