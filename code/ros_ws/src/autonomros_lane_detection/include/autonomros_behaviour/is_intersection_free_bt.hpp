
#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__PIPELINE_SEQUENCE_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__PIPELINE_SEQUENCE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <string>
#include <memory>
#include <chrono>

#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"


namespace nav2_behavior_tree
{

using std::placeholders::_1;  

class IntersectionFreeChecker : public BT::ConditionNode
{
  
  public:
    IntersectionFreeChecker(
      const std::string & condition_name,
      const BT::NodeConfiguration & conf);

    BT::NodeStatus tick() override;

    void intersection_callback(const typename std_msgs::msg::Bool::SharedPtr msg);
     
  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    std::thread callback_group_executor_thread;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr intersection_sub_;
    
    rmw_qos_profile_t qos_profile =
    {
      RMW_QOS_POLICY_HISTORY_KEEP_LAST,
      10,
      RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      RMW_QOS_POLICY_DURABILITY_VOLATILE,
      RMW_QOS_DEADLINE_DEFAULT,
      RMW_QOS_LIFESPAN_DEFAULT,
      RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
      RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
      false
    };

    bool free = true;
 
};
}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__PIPELINE_SEQUENCE_HPP_