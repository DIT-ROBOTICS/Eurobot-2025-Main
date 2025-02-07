#pragma once

// Use C++ libraries
#include <filesystem>
#include <fstream>
#include <vector>
#include <deque>
#include <bitset>
#include <chrono>
#include <cmath>

// Use behavior tree
#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>

// Use ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

// Use ros message
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

// tf2 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/impl/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

// Use self define message
#include "nav2_msgs/action/navigate_to_pose.hpp"

#define PI 3.1415926

using namespace BT;

namespace BT {
    template <> inline geometry_msgs::msg::PoseStamped convertFromString(StringView str);
    template <> inline int convertFromString(StringView str);
    template <> inline std::deque<int> convertFromString(StringView str);
}

class PointProvider : public BT::SyncActionNode {

public:
    PointProvider(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config)
    {}

    /* Node remapping function */
    static BT::PortsList providedPorts();

    /* Start and running function */
    BT::NodeStatus tick() override;
private:
    geometry_msgs::msg::PoseStamped point;
};

class Navigation : public BT::StatefulActionNode {

public:

    Navigation(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node, rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr rival_pub)
        : BT::StatefulActionNode(name, config), node_(node), rival_pub_(rival_pub)
    {}

    /* Node remapping function */
    static BT::PortsList providedPorts();

    /* Start and running function */
    BT::NodeStatus onStart () override;
    BT::NodeStatus onRunning() override;

    /* Halt function - not implemented */
    void onHalted() override;

private:
    double calculateDistance(const geometry_msgs::msg::Pose &pose1, const geometry_msgs::msg::Pose &pose2);
    void broadcastTransform(const geometry_msgs::msg::Pose &pose);

    std::shared_ptr<rclcpp::Node> node_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr rival_pub_;
    // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr predict_goal_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    double move_x_;
    double move_y_;
    double rival_velocity = 0.01;

    bool finished_ = false;
    geometry_msgs::msg::PoseStamped goal_;
    geometry_msgs::msg::PoseWithCovarianceStamped current_pose_;
    geometry_msgs::msg::PoseStamped rival_predict_goal_;
};

/*****************/
/* Docking state */
/*****************/
class Docking : public BT::RosActionNode<nav2_msgs::action::NavigateToPose> {

public:
    Docking(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
        : RosActionNode<nav2_msgs::action::NavigateToPose>(name, conf, params)
    {
        nav_finished_ = false;
        nav_error_ = false;
        mission_type_ = 0;
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
    int mission_type_;
    geometry_msgs::msg::PoseStamped goal_;
    geometry_msgs::msg::PoseStamped current_pose_;
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

// class GeneratePathPoint : public BT::SyncActionNode {

// public: 
//     GeneratePathPoint(const std::string& name, const BT::NodeConfiguration& config, std::shared_ptr<rclcpp::Node> node) 
//         : BT::SyncActionNode(name, config), node_(node) {}

//     static BT::PortsList providedPorts();

//     BT::NodeStatus tick() override;

// private:
//     std::shared_ptr<rclcpp::Node> node_;
// };

class initPoints : public BT::SyncActionNode {

public: 
    initPoints(const std::string& name, const BT::NodeConfiguration& config, std::shared_ptr<rclcpp::Node> node) 
        : BT::SyncActionNode(name, config), node_(node) {}

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    std::shared_ptr<rclcpp::Node> node_;
};

class StateUpdater : public BT::StatefulActionNode
{
public:
    StateUpdater(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node)
        : BT::StatefulActionNode(name, config), node_(node)
    {}

    /* Node remapping function */
    static BT::PortsList providedPorts();

    /* Start and running function */
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;

    /* Halt function */
    void onHalted() override;

private:
    std::shared_ptr<rclcpp::Node> node_;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr materials_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr garbages_pub_;
    int pub_count = 0;
    
    geometry_msgs::msg::PoseArray materials_info_;
    geometry_msgs::msg::PoseArray garbage_points_;
    std::deque<geometry_msgs::msg::PoseStamped> materials_info_deque;
    std::deque<geometry_msgs::msg::PoseStamped> garbage_points_deque;
};