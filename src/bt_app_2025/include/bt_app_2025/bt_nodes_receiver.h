#pragma once

// Use C++ libraries
#include <filesystem>
#include <fstream>
#include <deque>
#include <bitset>

// Use behavior tree
#include <behaviortree_ros2/bt_action_node.hpp>

// Use ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

// Use ros message
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
// tf2 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/impl/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

// Use self define message
#include "btcpp_ros2_interfaces/action/nav_mission.hpp"
#include "btcpp_ros2_interfaces/action/firmware_mission.hpp"

using namespace BT;

namespace BT {
    template <> inline geometry_msgs::msg::TwistStamped convertFromString(StringView str);
    template <> inline int convertFromString(StringView str);
    template <> inline std::deque<int> convertFromString(StringView str);
}

/***************/
/* LocReceiver */
/***************/
class LocReceiver : public BT::StatefulActionNode
{
public:
  LocReceiver(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node)
    : BT::StatefulActionNode(name, config), node_(node), tf_buffer_(node_->get_clock()), listener_(tf_buffer_)
  {}

  /* Node remapping function */
  static BT::PortsList providedPorts();

  /* Start and running function */
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void UpdateRobotPose();
  void UpdateRivalPose();

  /* Halt function */
  void onHalted() override;

private:
  int tick_count = 0;
  geometry_msgs::msg::TwistStamped robot_pose_;
  geometry_msgs::msg::TwistStamped rival_pose_;
  std::shared_ptr<rclcpp::Node> node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;

  std::string input;
};

/***************/
/* NavReceiver */
/***************/
class NavReceiver : public BT::StatefulActionNode
{
public:
  NavReceiver(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node)
    : BT::StatefulActionNode(name, config), node_(node) {
    subscription_ = node_->create_subscription<geometry_msgs::msg::TwistStamped>("rival/predict_goal", 10, std::bind(&NavReceiver::topic_callback, this, std::placeholders::_1));
  }

  /* Node remapping function */
  static BT::PortsList providedPorts();

  /* Start and running function */
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;

  /* Halt function */
  void onHalted() override;

private:
  void topic_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_;
  std::shared_ptr<rclcpp::Node> node_;
  geometry_msgs::msg::TwistStamped rival_predict_goal_;
};

/***************/
/* CamReceiver */
/***************/
class CamReceiver : public BT::StatefulActionNode
{
public:
  LocReceiver(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node)
    : BT::StatefulActionNode(name, config), node_(node) {
    sub_global_info_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>("/robot/objects/global_info", 10, std::bind(&CamReceiver::global_info_callback, this, std::placeholders::_1));
    sub_banner_info_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>("/robot/objects/global_info_banner", 10, std::bind(&CamReceiver::banner_info_callback, this, std::placeholders::_1));
    sub_local_info_ = node_->create_subscription<geometry_msgs::msg::PoseArray>("/robot/objects/local_info", 10, std::bind(&CamReceiver::local_info_callback, this, std::placeholders::_1));
  }

  /* Node remapping function */
  static BT::PortsList providedPorts();

  /* Start and running function */
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;

  /* Halt function */
  void onHalted() override;

private:
  void global_info_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void banner_info_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void local_info_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_global_info_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_banner_info_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_local_info_;
  std::shared_ptr<rclcpp::Node> node_;
  int number = 0;
};