#ifndef OBSTACLE_LAYER_HPP_
#define OBSTACLE_LAYER_HPP_

#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "iceoryx_msg/msg/grid234.hpp"

namespace autonomros_obstacle_costmap_plugin
{

struct ObstaclePoint {
  double x;
  double y;
  bool occupied;
};

class ObstacleLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  ObstacleLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  virtual void reset();

  virtual bool isClearable() {return true;}

private:
  void grid_callback(iceoryx_msg::msg::Grid234::SharedPtr);

  rclcpp::Subscription<iceoryx_msg::msg::Grid234>::SharedPtr grid_sub_;

  std::vector<struct ObstaclePoint> obstacles_;
  std::mutex obstacles_mutex_;
};

} // namespace autonomros_obstacle_costmap_plugin

#endif  // OBSTACLE_LAYER_HPP_
