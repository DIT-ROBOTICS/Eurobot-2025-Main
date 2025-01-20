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

// using std_srvs::SetBool;

using namespace BT;

namespace BT {
    template <> inline geometry_msgs::msg::TwistStamped convertFromString(StringView str);
    template <> inline int convertFromString(StringView str);
    template <> inline std::deque<int> convertFromString(StringView str);
}

/********************/
/* Navigation state */
/********************/
class Navigation : public BT::RosActionNode<btcpp_ros2_interfaces::action::NavMission> {

public:
    Navigation(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
        : RosActionNode<btcpp_ros2_interfaces::action::NavMission>(name, conf, params)
    {}

    /* Node remapping function */
    static PortsList providedPorts();
    bool setGoal(RosActionNode::Goal& goal) override;
    NodeStatus onResultReceived(const WrappedResult& wr) override;
    virtual NodeStatus onFailure(ActionNodeErrorCode error) override;
    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback);

private:

    bool nav_finished_ = false;
    bool nav_error_ = false;
    int nav_type_;
    geometry_msgs::msg::TwistStamped goal_;
};

/*****************/
/* Docking state */
/*****************/
class Docking : public BT::RosActionNode<btcpp_ros2_interfaces::action::NavMission> {

public:
    Docking(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
        : RosActionNode<btcpp_ros2_interfaces::action::NavMission>(name, conf, params)
    {}

    /* Node remapping function */
    static PortsList providedPorts();
    bool setGoal(RosActionNode::Goal& goal) override;
    NodeStatus onResultReceived(const WrappedResult& wr) override;
    virtual NodeStatus onFailure(ActionNodeErrorCode error) override;
    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback);

private:

    bool nav_finished_ = false;
    bool nav_error_ = false;
    geometry_msgs::msg::TwistStamped goal_;
};

class DynamicAdjustment : public BT::StatefulActionNode
{
public:
    DynamicAdjustment(const std::string& name, const BT::NodeConfig& config)
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
    geometry_msgs::msg::TwistStamped robot_pose_;
    geometry_msgs::msg::TwistStamped rival_pose_;
    geometry_msgs::msg::TwistStamped rival_predict_goal_;
    std::deque<int> stage_info_;
    std::deque<geometry_msgs::msg::TwistStamped> materials_info_;
    std::deque<geometry_msgs::msg::TwistStamped> garbage_points_;

    geometry_msgs::msg::TwistStamped goal_;
};