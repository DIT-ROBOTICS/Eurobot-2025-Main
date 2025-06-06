#pragma once

#include <filesystem>
#include <fstream>
#include <deque>
#include <bitset>
#include <chrono>
#include <cmath>

// BT
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "behaviortree_ros2/bt_topic_pub_node.hpp"
// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// self defined message
#include "btcpp_ros2_interfaces/action/navigation.hpp"
#include "example_interfaces/action/fibonacci.hpp"
//message
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
// tf2 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/impl/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#define PI 3.1415926

using namespace BT;

// Convert string to message
namespace BT {
  template <> inline geometry_msgs::msg::PoseStamped convertFromString(StringView str);
  template <> inline geometry_msgs::msg::TwistStamped convertFromString(StringView str);
  template <> inline int convertFromString(StringView str);
  template <> inline std::deque<int> convertFromString(StringView str);
}

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

/*******************/
/* Topic Publisher */
/*******************/
// A topic publisher node
// Without API
class TopicPubTest : public BT::StatefulActionNode
{
public:
  TopicPubTest(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node)
    : BT::StatefulActionNode(name, config), node_(node) {
    publisher_ = node_->create_publisher<std_msgs::msg::Int32>("number", 10);
  }
  /* Node remapping function */
  static BT::PortsList providedPorts();
  /* Start and running function */
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  /* Halt function */
  void onHalted() override;
private:
  // node
  std::shared_ptr<rclcpp::Node> node_;
  // about msg
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  std_msgs::msg::Int32 number;
  // params
  int a = 0;
};

/********************/
/* Topic Subscriber */
/********************/
// A topic subscriber node
// Without API
class TopicSubTest : public BT::StatefulActionNode
{
public:
  TopicSubTest(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node)
    : BT::StatefulActionNode(name, config), node_(node) {
    subscription_ = node_->create_subscription<std_msgs::msg::Int32>("number", 10, std::bind(&TopicSubTest::topic_callback, this, std::placeholders::_1));
  }
  /* Node remapping function */
  static BT::PortsList providedPorts();
  /* Start and running function */
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  /* Halt function */
  void onHalted() override;
private:
  // function
  BT::NodeStatus topic_callback(const std_msgs::msg::Int32::SharedPtr msg);
  // node
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
  std::shared_ptr<rclcpp::Node> node_;
  // params
  int number = 0;
};

class StandardTopicPub : public RosTopicPubNode<std_msgs::msg::Int32>
{
public:
  explicit StandardTopicPub(const std::string& name, const BT::NodeConfig& conf, const RosNodeParams& params)
    : RosTopicPubNode<std_msgs::msg::Int32>(name, conf, params)
  {}
  /* Node remapping function */
  static BT::PortsList providedPorts();
  /* Start and running function */
  bool setMessage(std_msgs::msg::Int32& msg);

private:
};

/********************/
/* Topic Subscriber */
/********************/
// Use API provided by package behaviortree_ros2
// To create a topic subscriber node
class StandardTopicSub : public BT::RosTopicSubNode<std_msgs::msg::Int32>
{
public:
  explicit StandardTopicSub(const std::string& name, const BT::NodeConfig& conf, const RosNodeParams& params)
    : RosTopicSubNode<std_msgs::msg::Int32>(name, conf, params)
  {}
  /* Node remapping function */
  static BT::PortsList providedPorts();
  /* Start and running function */
  NodeStatus onTick(const std::shared_ptr<std_msgs::msg::Int32>& last_msg) override;
  bool latchLastMessage() const override;
private:
};

/************************/
/* Location tf listener */
/************************/
// A BT node used to listen to the robot's pose from tf2
class LocalizationTemp : public BT::StatefulActionNode
{
public:
  LocalizationTemp(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node)
    : BT::StatefulActionNode(name, config), node_(node), tf_buffer_(node_->get_clock()), listener_(tf_buffer_)
  {}

  /* Node remapping function */
  static BT::PortsList providedPorts();

  /* Start and running function */
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void UpdateRobotPose();

  /* Halt function */
  void onHalted() override;

private:
  int tick_count = 0;
  geometry_msgs::msg::TwistStamped robot_pose_;
  std::shared_ptr<rclcpp::Node> node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;

  std::string input;
};

/**************************/
/* Simple navigation node */
/**************************/
// A BT node used to communicate with simple navigation server (btcpp_ros2_interfaces)
class NavigationTemp : public BT::RosActionNode<btcpp_ros2_interfaces::action::Navigation>
{
public:
  NavigationTemp(const std::string& name, const BT::NodeConfig& conf,
                   const BT::RosNodeParams& params)
    : RosActionNode<btcpp_ros2_interfaces::action::Navigation>(name, conf, params)
  {}

  /* Node remapping function */
  static PortsList providedPorts();
  bool setGoal(RosActionNode::Goal& goal) override;
  NodeStatus onResultReceived(const WrappedResult& wr) override;
  virtual NodeStatus onFailure(ActionNodeErrorCode error) override;
  NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback);
private:
};

/*************************/
/* Basic ROS Action node */
/*************************/
// A ROS action client using the API
// Used to test the communication with the firmware
class BTMission : public BT::RosActionNode<example_interfaces::action::Fibonacci> 
{
public:
  BTMission(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
    : RosActionNode<example_interfaces::action::Fibonacci>(name, conf, params)
  {}
  /* Node remapping function */
  static PortsList providedPorts();
  bool setGoal(RosActionNode::Goal& goal) override;
  NodeStatus onResultReceived(const WrappedResult& wr) override;
  virtual NodeStatus onFailure(ActionNodeErrorCode error) override;
  NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback);
private:
  // mission status
  bool mission_finished_ = false;
  bool mission_failed_ = false;
  // message params
  int order_;
  std::deque<int> sequence_;
  std::deque<int> partial_sequence_;
};

/**************************/
/* Navigation action node */
/**************************/
// Write an ROS action to communicate with navigation server (NavigateToPose) 
// without using the API privided by package behaviortree_ros2
class NavAction : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigation = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  NavAction(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node)
    : BT::StatefulActionNode(name, config), node_(node)
  {}
  /* Node remapping function */
  static BT::PortsList providedPorts();
  /* Start and running function */
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  void send_goal();
  void goal_response_callback(GoalHandleNavigation::SharedPtr goal_handle);
  void feedback_callback(
    GoalHandleNavigation::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  BT::NodeStatus result_callback(const GoalHandleNavigation::WrappedResult & result);

private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  // nav status
  bool nav_finished_ = false;
  bool nav_error_ = false;
  // bot pose
  geometry_msgs::msg::PoseStamped goal_;
  geometry_msgs::msg::PoseStamped current_pose_;
};

/************************/
/* Parallel Test node 1 */
/************************/
class count_5 : public BT::StatefulActionNode
{
public:
  count_5(const std::string& name, const BT::NodeConfig& config)
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
/************************/
/* Parallel Test node 2 */
/************************/
class count_10 : public BT::StatefulActionNode
{
public:
  count_10(const std::string& name, const BT::NodeConfig& config)
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

/************************/
/* Parallel Test node 3 */
/************************/
class count_15 : public BT::StatefulActionNode
{
public:
  count_15(const std::string& name, const BT::NodeConfig& config)
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

/************************/
/* Parallel Test node 4 */
/************************/
class Parallel_check : public BT::StatefulActionNode
{
public:
  Parallel_check(const std::string& name, const BT::NodeConfig& config)
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

  std::string input;
};