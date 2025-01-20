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
    // bool send_mission = true;
    int current_mission_type_ = 0;
    int current_mission_sub_type_ = 0;

    int mission_type_;
    int mission_sub_type_;
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

/***********************************/
/* Simple Node to activate SIMA */
/***********************************/
// class SIMAactivate : public BT::SyncActionNode {
// public:
//     SIMAactivate(const std::string& name, const BT::NodeConfig& config)
//         : BT::SyncActionNode(name, config) {}

//     /* Node remapping function */
//     static BT::PortsList providedPorts(){ return {}; }

//     /* Start and running function */
//     BT::NodeStatus tick() override;
// private:
// };


class CollectFinisher : public BT::StatefulActionNode
{
public:
  CollectFinisher(const std::string& name, const BT::NodeConfig& config)
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
  std::deque<int> step_results;
  bool has_raw_material_;
  bool has_one_level_;
  bool has_garbage_;
};

class ConstructFinisher : public BT::StatefulActionNode
{
public:
  ConstructFinisher(const std::string& name, const BT::NodeConfig& config)
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
  std::deque<int> step_results;
  int success_levels_;
  int failed_levels_;
  bool has_garbage_;
  int on_robot_materials_;

  std::deque<int> stage_info_;
};