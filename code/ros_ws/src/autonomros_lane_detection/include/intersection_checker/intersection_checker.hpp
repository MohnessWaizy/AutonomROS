#include <iostream>
#include <string>
#include <sstream>
#include <chrono>
#include <functional>
#include <memory>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "mqtt/client.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#ifndef INTERSECTION_CHECKER__INTERSECTION_CHECKER_HPP_
#define INTERSECTION_CHECKER__INTERSECTION_CHECKER_HPP_

class Intersection_Checker : public rclcpp::Node
{
public:
  Intersection_Checker();
  ~Intersection_Checker(); 

  /*
  * @brief: Checks the position of the car and publishes the controller to use
  */
  void check_position();

  /*
  * @brief: call function for the Behavior Tree condition node isIntersectionFree
  * @param: free_intersection: set the stop signal to for the BT. 0 - Stop. 1 - Drive.
  */
  void bt_call(bool free_intersection);
  
  /*
  * @brief: request the virtual traffic light for a free intersection
  * @param: intersection: the intersection_id to request the virtual traffic light for
  */
  void vtl_request(std::string intersection);

  /*
  * @brief: send a free message to the virtual traffic light to signal a free intersection
  */
  void vtl_free();
private:
  //flags for requesting & controller switching
  bool switch_controller = false;
  bool new_intersection = true;

  //service for the behavior tree
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr intersection_publisher_;
  

  //timer for checking the position
  rclcpp::TimerBase::SharedPtr timer_{nullptr};

  //controller publisher
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  
  //transform listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
  
  char car_id;  // 0 = car1, 1 = car2 
  
  //mqtt
  std::string ADDRESS;
  const std::string CLIENT_ID	{ "mqtt_bridge" };
  mqtt::client *cli_;

  //qos profile for publisher
  rmw_qos_profile_t qos_profile =
  {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    10,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
  };
};

#endif 