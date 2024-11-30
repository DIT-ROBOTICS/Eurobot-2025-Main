#pragma once

// Basic Libraries
#include <filesystem>
#include <fstream>
#include <chrono>
#include <memory>
#include <string>

// ROS related Libraries
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

// Behavior Tree Libraries
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/action_node.h"

// ************* simple navigation
// ROS Messages
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"

// TF2 Libraries
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/impl/utils.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"

// namespace BT
// {
// template <>
// inline geometry_msgs::msg::TwistStamped convertFromString(BT::StringView str);
// template <>
// inline int convertFromString(StringView str);
// }  // namespace BT

// Test node
class Testing : public BT::StatefulActionNode
{
public:
  Testing(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config)
  {}

  /* Node remapping function */
  static BT::PortsList providedPorts();

  /* Start and running function */
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;

  /* Halt function */
  void onHalted() override;

private:
  int tick_count = 0;

  std::string input;
};

// Simple navigation node
// class SimpleNavigation : public BT::RosTopicPubNode<geometry_msgs::msg::Twist>
class SimpleNavigation : public BT::StatefulActionNode
{
public:
  // SimpleNavigation(const std::string& name, const BT::NodeConfig& config,
  //                  const RosNodeParams& params)
  //   : BT::RosTopicPubNode<geometry_msgs::msg::Twist>(name, config, params)
  SimpleNavigation(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("bt_action_publisher");
    RCLCPP_INFO(node_->get_logger(), "test1");
    publisher_ = node_->create_publisher<std_msgs::msg::String>("bt_topic", 10);
    // **************** simple navigation
    // Initialize ROS Publisher
    pub_cmd_vel_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    // Initialize ROS Subscriber
    sub_loc_vel_ = node_->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&SimpleNavigation::locVelCallback, this, std::placeholders::_1));
    // Initialize ROS Timer (25HZ)
    // auto timer_callback = [this]() -> void {
    //   timerCallback();
    // };
    // timer_ = node_->create_wall_timer(std::chrono::milliseconds(500), timer_callback);
    timer_ = node_->create_wall_timer(std::chrono::milliseconds(40), std::bind(&SimpleNavigation::timerCallback, this));
  }

  /* Node remapping function */
  static BT::PortsList providedPorts();

  /* Start and running function */
  BT::NodeStatus onStart();
  BT::NodeStatus onRunning();

  /* Halt function - not implemented */
  void onHalted();

  /* Finished function */
  void setFinished();

  // **************** simple navigation
  void setGoal(geometry_msgs::msg::TwistStamped goal_position);

  std::vector<std::function<void()>> finish_callbacks_;

private:
  bool finished_ = false;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  // **************** simple navigation
  /* ROS Publisher */
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;

  /* ROS Subscriber */
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_loc_vel_;

  /* ROS Timer */
  rclcpp::TimerBase::SharedPtr timer_;

  /* ROS Callbacks */
  void locVelCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback(/*const rclcpp::TimerEvent& event*/);

  /* ROS Items */
  geometry_msgs::msg::Twist cmd_vel_;
  nav_msgs::msg::Odometry loc_;
  geometry_msgs::msg::TwistStamped goal_;

  /* Other */
  bool is_trigger_ = false;

  // Time for current race (100 seconds)
  double race_elapsed_time_ = 0;
};