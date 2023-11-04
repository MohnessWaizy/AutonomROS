#include "obstacle_layer/obstacle_layer.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace autonomros_obstacle_costmap_plugin
{
ObstacleLayer::ObstacleLayer()
{

}

void ObstacleLayer::onInitialize()
{
  RCLCPP_INFO(
    logger_,
    "AutonomROS Obstacle Layer: onInitialize()"
  );

  enabled_ = true;
  default_value_ = nav2_costmap_2d::FREE_SPACE;
  current_ = true;

  matchSize();

  auto node = node_.lock();

  auto qos = rclcpp::QoS(1);
  qos.best_effort();
  grid_sub_ =
    node->create_subscription<iceoryx_msg::msg::Grid234>(
    "obstacles", qos,
    std::bind(&ObstacleLayer::grid_callback, this, std::placeholders::_1));
}

void ObstacleLayer::grid_callback(iceoryx_msg::msg::Grid234::SharedPtr grid)
{
  auto node = node_.lock();
  auto time = node->now();
  obstacles_mutex_.lock();
  obstacles_.clear();
  if (tf_->canTransform("odom", "base_link", time, rclcpp::Duration::from_seconds(1))) {
    for (int gx = 0; gx < 18; ++gx) {
      for (int gy = 0; gy < 13; ++gy) {
        unsigned int index = gx * 13 + gy;
        geometry_msgs::msg::PoseStamped point;
        point.header.frame_id = "base_link";
        point.header.stamp = time;
        point.pose.position.x = gx * 0.05 + 0.3;
        point.pose.position.y = gy * 0.05 - 0.325;
        auto odom_point = tf_->transform(point, "odom");
        struct ObstaclePoint obstacle;
        obstacle.x = odom_point.pose.position.x;
        obstacle.y = odom_point.pose.position.y;
        obstacle.occupied = grid->data[index] > 250;
        obstacles_.push_back(obstacle);
      }
    }
  }
  obstacles_mutex_.unlock();
}

void ObstacleLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);

  obstacles_mutex_.lock();
  for (auto obstacle : obstacles_) {
    unsigned int mx, my;
    if (worldToMap(obstacle.x, obstacle.y, mx, my)) {
      touch(obstacle.x, obstacle.y, min_x, min_y, max_x, max_y);
      if (obstacle.occupied) {
        costmap_[getIndex(mx, my)] = nav2_costmap_2d::LETHAL_OBSTACLE;
      } else {
        costmap_[getIndex(mx, my)] = nav2_costmap_2d::FREE_SPACE;
      }
    }
  }
  obstacles_mutex_.unlock();
}

void ObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
}


void ObstacleLayer::reset()
{
  resetMaps();
}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(autonomros_obstacle_costmap_plugin::ObstacleLayer, nav2_costmap_2d::Layer)
