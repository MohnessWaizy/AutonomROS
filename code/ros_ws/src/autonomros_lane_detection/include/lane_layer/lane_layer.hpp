/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2020, Samsung R&D Institute Russia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Alexey Merzlyakov
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_costmap2d_plugin.html
 *********************************************************************/
#ifndef LANE_LAYER_HPP_
#define LANE_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "message_filters/subscriber.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "tf2_ros/message_filter.h"

namespace nav2_costmap_2d
{

class LaneLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  LaneLayer();

  virtual void onInitialize();
  void pointCloud2Callback(sensor_msgs::msg::PointCloud2::ConstSharedPtr message,
    const std::shared_ptr<nav2_costmap_2d::ObservationBuffer> & buffer);

  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);


  //virtual void matchSize();

   /**
   * @brief Activate the layer
   */
  virtual void activate();

  virtual void deactivate();


  virtual void reset();
 
  bool getMarkingObservations(std::vector<nav2_costmap_2d::Observation> & marking_observations) const;

  bool getClearingObservations(
    std::vector<nav2_costmap_2d::Observation> & clearing_observations) const;



  virtual void onFootprintChanged();

  virtual bool isClearable() {return false;}

  void resetBuffersLastUpdated();

private:
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

  // Indicates that the entire gradient should be recalculated next time.
  bool need_recalculation_;

  // Size of gradient in cells
  int GRADIENT_SIZE = 20;
  // Step of increasing cost per one cell in gradient
  int GRADIENT_FACTOR = 10;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_sub_;
  sensor_msgs::msg::PointCloud2::SharedPtr point_buffer_;

protected:
  virtual void raytraceFreespace(
  const nav2_costmap_2d::Observation & clearing_observation,
  double * min_x, double * min_y,
  double * max_x,
  double * max_y);

  void updateRaytraceBounds(
  double ox, double oy, double wx, double wy, double max_range, double min_range,
  double * min_x, double * min_y,
  double * max_x,
  double * max_y);


  std::vector<geometry_msgs::msg::Point> transformed_footprint_;
  bool footprint_clearing_enabled_;
    void updateFootprint(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);

  std::string global_frame_;  ///< @brief The global frame for the costmap
  double max_obstacle_height_;  ///< @brief Max Obstacle Height

  // @brief Used for the observation message filters
  std::vector<std::shared_ptr<message_filters::SubscriberBase>> observation_subscribers_;
  /// @brief Used to make sure that transforms are available for each sensor
  std::vector<std::shared_ptr<tf2_ros::MessageFilterBase>> observation_notifiers_;
  /// @brief Used to store observations from various sensors
  std::vector<std::shared_ptr<nav2_costmap_2d::ObservationBuffer>> observation_buffers_;
  // @brief Used to store observation buffers used for marking obstacles
  std::vector<std::shared_ptr<nav2_costmap_2d::ObservationBuffer>> marking_buffers_;
  /// @brief Used to store observation buffers used for clearing obstacles
  std::vector<std::shared_ptr<nav2_costmap_2d::ObservationBuffer>> clearing_buffers_;
  bool rolling_window_;
  bool was_reset_;
  int combination_method_;
  
    // Used only for testing purposes
  std::vector<nav2_costmap_2d::Observation> static_clearing_observations_;
  std::vector<nav2_costmap_2d::Observation> static_marking_observations_;

};

}  // namespace nav2_gradient_costmap_plugin

#endif  // GRADIENT_LAYER_HPP_