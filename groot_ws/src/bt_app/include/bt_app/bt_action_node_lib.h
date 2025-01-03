#pragma once

#include <behaviortree_ros2/bt_action_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

#include "btcpp_ros2_interfaces/action/navigation.hpp"

using namespace BT;

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

// class LocalizationTemp : public RosTopicNode<msg::Localization>
// {
// public:
//   LocalizationTemp(const std::string& name, const BT::NodeConfig& config,
//                    const RosNodeParams& params)
//     : BT::RosTopicPubNode<geometry_msgs::msg::Twist>(name, config, params)
//   {}

// private:
// };

// Simple navigation node
// class NavigationTemp : public BT::RosTopicPubNode<geometry_msgs::msg::Twist>
class NavigationTemp : public RosActionNode<btcpp_ros2_interfaces::action::Navigation>
{
public:
  NavigationTemp(const std::string& name, const NodeConfig& conf,
                   const RosNodeParams& params)
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