#pragma once

// Use C++ libraries
#include <filesystem>
#include <fstream>
#include <deque>
#include <bitset>
#include <chrono>
#include <cmath>
#include <string.h>

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
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

// tf2 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/impl/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

// Use action message
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "opennav_docking_msgs/action/dock_robot.hpp"

#define PI 3.1415926

using namespace BT;

namespace BT {
    template <> inline geometry_msgs::msg::PoseStamped convertFromString(StringView str);
    template <> inline int convertFromString(StringView str);
    template <> inline std::deque<int> convertFromString(StringView str);
}

/********************/
/* Navigation state */
/********************/
class Navigation : public BT::RosActionNode<nav2_msgs::action::NavigateToPose> {

public:
    Navigation(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
        : RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
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
    geometry_msgs::msg::PoseStamped goal_;
    geometry_msgs::msg::PoseStamped current_pose_;
};

/*****************/
/* Docking state */
/*****************/
class Docking : public BT::RosActionNode<opennav_docking_msgs::action::DockRobot> {

public:
    Docking(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
        : RosActionNode<opennav_docking_msgs::action::DockRobot>(name, conf, params), tf_buffer_(params.nh.lock()->get_clock()), listener_(tf_buffer_)
    {
        nav_finished_ = false;
        nav_error_ = false;
        isPureDocking_ = true;
    }

    /* Node remapping function */
    static PortsList providedPorts();
    bool setGoal(RosActionNode::Goal& goal) override;
    NodeStatus onResultReceived(const WrappedResult& wr) override;
    virtual NodeStatus onFailure(ActionNodeErrorCode error) override;
    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback);

private:
    bool UpdateRobotPose();

    bool nav_finished_;
    bool nav_error_;
    bool isPureDocking_;
    std::string dock_type_;
    double offset_;
    geometry_msgs::msg::PoseStamped goal_;
    geometry_msgs::msg::PoseStamped robot_pose_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener listener_;
};

class Rotation : public BT::RosActionNode<nav2_msgs::action::NavigateToPose> {

public:
    Rotation(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
        : RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
    {
        nav_finished_ = false;
        nav_error_ = false;
    }

    /* Node remapping function */
    static PortsList providedPorts();
    bool setGoal(RosActionNode::Goal& goal) override;
    NodeStatus onResultReceived(const WrappedResult& wr) override;
    virtual NodeStatus onFailure(ActionNodeErrorCode error) override;
    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback);

private:

    bool nav_finished_;
    bool nav_error_;
    geometry_msgs::msg::PoseStamped goal_;
    geometry_msgs::msg::PoseStamped current_pose_;
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
    geometry_msgs::msg::PoseStamped robot_pose_;
    geometry_msgs::msg::PoseStamped rival_pose_;
    geometry_msgs::msg::PoseStamped rival_predict_goal_;
    std::deque<int> stage_info_;
    std::deque<geometry_msgs::msg::PoseStamped> materials_info_;
    std::deque<geometry_msgs::msg::PoseStamped> garbage_points_;

    std::deque<geometry_msgs::msg::PoseStamped> goal_canditate_;

    geometry_msgs::msg::PoseStamped goal_;
};