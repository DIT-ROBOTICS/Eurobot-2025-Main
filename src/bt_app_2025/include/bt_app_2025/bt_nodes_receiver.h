#pragma once

// Use C++ libraries
#include <filesystem>
#include <fstream>
#include <deque>
#include <bitset>

// Use behavior tree
#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>

// Use ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

// Use ros message
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"
// tf2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/impl/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

// Use self define message

using namespace BT;

// namespace BT {
//     template <> inline geometry_msgs::msg::TwistStamped convertFromString(StringView str);
//     template <> inline int convertFromString(StringView str);
//     template <> inline std::deque<int> convertFromString(StringView str);
// }

/***************/
/* LocReceiver */
/***************/
// receive the robot pose and rival pose from localization team
class LocReceiver
{
public:
  LocReceiver(const RosNodeParams& params)
    : node_(params.nh.lock()), tf_buffer_(node_->get_clock()), listener_(tf_buffer_)
  {
    node_->get_parameter("frame_id", frame_id_);
  }
  static bool UpdateRobotPose(geometry_msgs::msg::PoseStamped &robot_pose_, tf2_ros::Buffer &tf_buffer_, std::string frame_id_);
  static bool UpdateRivalPose(geometry_msgs::msg::PoseStamped &rival_pose_, tf2_ros::Buffer &tf_buffer_, std::string frame_id_);
private:
  geometry_msgs::msg::PoseStamped robot_pose_;
  geometry_msgs::msg::PoseStamped rival_pose_;
  std::shared_ptr<rclcpp::Node> node_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;
  std::string frame_id_;
};

/***************/
/* NavReceiver */
/***************/
// Receive the rival goal predicted by navigation team
// store the prediction to the BT blackboard
class NavReceiver : public BT::SyncActionNode
{
public:
  NavReceiver(const std::string& name, const BT::NodeConfig& config, const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
    : BT::SyncActionNode(name, config), node_(params.nh.lock()), blackboard_(blackboard)
  {}

  /* Node remapping function */
  static BT::PortsList providedPorts();

  /* Start and running function */
  BT::NodeStatus tick() override;

private:
  void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  std::shared_ptr<rclcpp::Node> node_;
  BT::Blackboard::Ptr blackboard_;
  geometry_msgs::msg::PoseStamped rival_predict_goal_;
};

/***************/
/* CamReceiver */
/***************/
// receive the vision data
// reconstruct and organize to the message
// that is useful for main team
// and then store to the BT blackboard
class CamReceiver : public BT::SyncActionNode
{
public:
  CamReceiver(const std::string& name, const BT::NodeConfig& config, const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
    : BT::SyncActionNode(name, config), node_(params.nh.lock()), blackboard_(blackboard)
  {}

  /* Node remapping function */
  static BT::PortsList providedPorts();

  /* Start and running function */
  BT::NodeStatus tick() override;

private:
  void materials_info_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void mission_info_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void score_callback(const std_msgs::msg::Int32::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_materials_info_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_mission_info_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_score_;

  std_msgs::msg::Int32MultiArray materials_info_;
  std_msgs::msg::Int32 score_;

  std::shared_ptr<rclcpp::Node> node_;
  BT::Blackboard::Ptr blackboard_;
  int mission_info_;
  int score_main_, score_vision_;
};

class TopicSubTest : public BT::StatefulActionNode
{
public:
  TopicSubTest(const std::string& name, const BT::NodeConfig& config, const RosNodeParams& params)
    : BT::StatefulActionNode(name, config), node_(params.nh) {
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