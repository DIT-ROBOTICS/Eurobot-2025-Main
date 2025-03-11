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
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

// tf2 
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// #include <tf2/impl/utils.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/buffer.h>
// #include <tf2/exceptions.h>

// Use self define message
#include "btcpp_ros2_interfaces/action/firmware_mission.hpp"

using namespace BT;

namespace BT {
    template <> inline geometry_msgs::msg::TwistStamped convertFromString(StringView str);
    template <> inline int convertFromString(StringView str);
    template <> inline std::deque<int> convertFromString(StringView str);
}

/********************/
/* Firmware Mission */
/********************/
class BTMission : public BT::RosActionNode<btcpp_ros2_interfaces::action::FirmwareMission> {

public:
  // BTMission(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Kernel> kernel, bool mec_callback)
  //     : BT::StatefulActionNode(name, config), kernel_(kernel), mec_callback_(mec_callback) {}
  BTMission(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
    : RosActionNode<btcpp_ros2_interfaces::action::FirmwareMission>(name, conf, params)
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

  std::deque<int> mission_type_;
  std::deque<int> mission_sub_type_;
  char progress_;
  int result_;
};

/******************/
/* Banner Mission */
/******************/
// class BannerMission : public BT::RosActionNode<btcpp_ros2_interfaces::action::FirmwareMission> {

// public:
//     // BTMission(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Kernel> kernel, bool mec_callback)
//     //     : BT::StatefulActionNode(name, config), kernel_(kernel), mec_callback_(mec_callback) {}
//     BannerMission(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
//         : RosActionNode<btcpp_ros2_interfaces::action::FirmwareMission>(name, conf, params)
//     {}

//     /* Node remapping function */
//     static PortsList providedPorts();
//     bool setGoal(RosActionNode::Goal& goal) override;
//     NodeStatus onResultReceived(const WrappedResult& wr) override;
//     virtual NodeStatus onFailure(ActionNodeErrorCode error) override;
//     NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback);

// private:
// };

/********************************/
/* Simple Node to activate SIMA */
/********************************/
class SIMAactivate : public BT::SyncActionNode {
public:
  SIMAactivate(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node)
    : BT::SyncActionNode(name, config), node_(node)
  {}

  /* Node remapping function */
  static BT::PortsList providedPorts() {
    return {}; 
  }

  /* Start and running function */
  BT::NodeStatus tick() override;
private:
  bool wakeUpSIMA();
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::TimerBase::SharedPtr timer_;
  float current_time_;
  bool mission_finished_ = false;
};

class MissionFinisher : public BT::StatefulActionNode
{
public:
  MissionFinisher(const std::string& name, const BT::NodeConfig& config, BT::Blackboard::Ptr blackboard)
    : BT::StatefulActionNode(name, config), blackboard_(blackboard)
  {
    matrix_node_ = rclcpp::Node::make_shared("matrix_node");
    matreials_accord_ = 0;

    matrix_node_->declare_parameter<std::vector<int>>("front_collect", std::vector<int>{});
    matrix_node_->declare_parameter<std::vector<int>>("back_collect", std::vector<int>{});
    matrix_node_->declare_parameter<std::vector<int>>("construct_1", std::vector<int>{});
    matrix_node_->declare_parameter<std::vector<int>>("spin_construct_2", std::vector<int>{});
    matrix_node_->declare_parameter<std::vector<int>>("spin_construct_3", std::vector<int>{});
    matrix_node_->declare_parameter<std::vector<int>>("not_spin_construct_2", std::vector<int>{});
    matrix_node_->declare_parameter<std::vector<int>>("not_spin_construct_3", std::vector<int>{});
  }

  /* Node remapping function */
  static BT::PortsList providedPorts();

  /* Start and running function */
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;

  /* Halt function */
  void onHalted() override;

private:
  // step_results_-> 1: front_collect 2: back_collect 3: construct_1 4: construct_2 5: construct_3
  std::deque<int> step_results_;
  // obot_type_-> 0: SpinArm 1: NotSpinArm
  bool robot_type_;
  bool matreials_accord_;
  int success_levels_;
  int failed_levels_;
  int mission_progress_;
  int front_materials_;
  int back_materials_;

  std::deque<int> stage_info_;

  rclcpp::Node::SharedPtr matrix_node_;

  BT::Blackboard::Ptr blackboard_;
};