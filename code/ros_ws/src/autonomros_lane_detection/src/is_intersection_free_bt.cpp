#include "autonomros_behaviour/is_intersection_free_bt.hpp"

namespace nav2_behavior_tree
{

IntersectionFreeChecker::IntersectionFreeChecker(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive,
    false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());


  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;
  intersection_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/intersection_check",
    qos,
    std::bind(&IntersectionFreeChecker::intersection_callback, this, _1),
    sub_option);

  RCLCPP_INFO(node_->get_logger(), "Condition Node IntersectionFreeChecker initialized.");
}

BT::NodeStatus IntersectionFreeChecker::tick()
{
  //process the sub 
  callback_group_executor_.spin_some();
  
  if(free){
    RCLCPP_DEBUG(node_->get_logger(), "true");
    return BT::NodeStatus::SUCCESS;
  } 
    RCLCPP_DEBUG(node_->get_logger(), "false");
    return BT::NodeStatus::FAILURE;
}

void IntersectionFreeChecker::intersection_callback(const typename std_msgs::msg::Bool::SharedPtr msg)
{
  free = msg->data;
  RCLCPP_INFO(node_->get_logger(), "free: '%s'", free ? "True" : "False");
}




#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::
      IntersectionFreeChecker>(name, config);
    };

  factory.registerBuilder<nav2_behavior_tree::
  IntersectionFreeChecker>("IntersectionFree", builder);
}

}
