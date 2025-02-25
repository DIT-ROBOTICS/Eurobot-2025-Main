#pragma once

// BT
#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <behaviortree_ros2/bt_topic_pub_node.hpp>
// ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
// self defined message
#include "btcpp_ros2_interfaces/action/navigation.hpp"
#include "example_interfaces/action/fibonacci.hpp"
//message
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
// tf2 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/impl/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

using namespace BT;

namespace BT {
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
  std::shared_ptr<rclcpp::Node> node_;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  std_msgs::msg::Int32 number;

  int a = 0;
  int b = 0;
};

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
  // void execute() {
  //   subscription_ = node_->create_subscription<std_msgs::msg::Int32>("number", 10, std::bind(&TopicSubTest1::topic_callback, this, std::placeholders::_1));
  // }
  void topic_callback(const std_msgs::msg::Int32::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
  std::shared_ptr<rclcpp::Node> node_;
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

// Simple navigation node
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

class BTMission : public BT::RosActionNode<example_interfaces::action::Fibonacci> {

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

  bool mission_finished_ = false;
  bool mission_failed_ = false;

  int order_;
  std::deque<int> sequence_;
  std::deque<int> partial_sequence_;
};