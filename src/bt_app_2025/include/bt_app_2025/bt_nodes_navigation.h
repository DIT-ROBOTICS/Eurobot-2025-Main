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
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
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
// using namespace MATH;

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
    {
        node_ = params.nh.lock();
    }
    /* Node remapping function */
    static PortsList providedPorts();
    bool setGoal(RosActionNode::Goal& goal) override;
    NodeStatus onResultReceived(const WrappedResult& wr) override;
    virtual NodeStatus onFailure(ActionNodeErrorCode error) override;
    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback);
private:
    NodeStatus goalErrorDetect();
    std::shared_ptr<rclcpp::Node> node_;
    bool nav_finished_ = false;
    bool nav_error_ = false;
    int nav_recov_times_ = 0;
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
        node_ = params.nh.lock();
        node_->get_parameter("frame_id", frame_id_);
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
    std::shared_ptr<rclcpp::Node> node_;
    bool nav_finished_;
    bool nav_error_;
    bool isPureDocking_;
    int nav_recov_times_ = 0;
    std::string dock_type_;
    double offset_;
    double shift_ = 0;
    geometry_msgs::msg::PoseStamped goal_;
    geometry_msgs::msg::PoseStamped robot_pose_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener listener_;
    std::string frame_id_;
};

class Rotation : public BT::RosActionNode<nav2_msgs::action::NavigateToPose> {

public:
    Rotation(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
        : RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
    {
        node_ = params.nh.lock();
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
    NodeStatus goalErrorDetect();
    std::shared_ptr<rclcpp::Node> node_;
    bool nav_finished_;
    bool nav_error_;
    int nav_recov_times_ = 0;
    geometry_msgs::msg::PoseStamped goal_;
    geometry_msgs::msg::PoseStamped current_pose_;
};

class StopRobot : public BT::SyncActionNode
{
public:
    StopRobot(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node)
    : BT::SyncActionNode(name, config), node_(node)
    {
        publisher_ = node_->create_publisher<std_msgs::msg::Bool>("/stopRobot", rclcpp::QoS(10).reliable().transient_local());
    }
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
private:
    std::shared_ptr<rclcpp::Node> node_;
    BT::Blackboard::Ptr blackboard_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    std_msgs::msg::Bool stop_msg;
};

class VisionCheck : public BT::DecoratorNode
{
public:
    VisionCheck(const std::string &name, const BT::NodeConfig &config, std::shared_ptr<rclcpp::Node> node, BT::Blackboard::Ptr blackboard)
        : BT::DecoratorNode(name, config), node_(node), blackboard_(blackboard), tf_buffer_(node_->get_clock())
    {
        node_->get_parameter("frame_id", frame_id_);
    }
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
private:
    int findBestTarget();
    BT::Blackboard::Ptr blackboard_;
    rclcpp::Node::SharedPtr node_;
    // for tf listener
    tf2_ros::Buffer tf_buffer_;
    std::string frame_id_;
    geometry_msgs::msg::PoseStamped robot_pose_;
    geometry_msgs::msg::PoseStamped rival_pose_;
    // for input
    std_msgs::msg::Int32MultiArray materials_info_;
    geometry_msgs::msg::PoseStamped base_;
    std::vector<double> material_points_;
    std::vector<double> mission_points_;
    // for vision
    std::deque<int> candidate_;
    bool last_mission_failed_;
};