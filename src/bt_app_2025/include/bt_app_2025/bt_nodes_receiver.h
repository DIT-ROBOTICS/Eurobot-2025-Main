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
  LocReceiver(std::shared_ptr<rclcpp::Node> node)
    : node_(node), tf_buffer_(node_->get_clock()), listener_(tf_buffer_)
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
  NavReceiver(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node, BT::Blackboard::Ptr blackboard)
    : BT::SyncActionNode(name, config), node_(node), blackboard_(blackboard)
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
  CamReceiver(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node, BT::Blackboard::Ptr blackboard)
    : BT::SyncActionNode(name, config), node_(node), blackboard_(blackboard)
  {}

  /* Node remapping function */
  static BT::PortsList providedPorts();

  /* Start and running function */
  BT::NodeStatus tick() override;

private:
  void materials_info_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void banner_info_callback(const std_msgs::msg::Int32::SharedPtr msg);
  void obstacles_info_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_materials_info_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_banner_info_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_obstacles_info_;

  geometry_msgs::msg::PoseArray materials_info_;
  std_msgs::msg::Int32 banner_info_;
  geometry_msgs::msg::PoseArray obstacles_info_;

  std::shared_ptr<rclcpp::Node> node_;
  BT::Blackboard::Ptr blackboard_;
};