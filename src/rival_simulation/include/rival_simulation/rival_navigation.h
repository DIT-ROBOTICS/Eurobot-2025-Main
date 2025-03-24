#pragma once

// Use C++ libraries
#include <filesystem>
#include <fstream>
#include <vector>
#include <deque>
#include <cmath>
#include <thread>

// Use behavior tree
#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>

// Use ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

// Use ros message
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

// tf2 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/impl/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#define PI 3.1415926

using namespace BT;

namespace BT {
    template <> inline geometry_msgs::msg::PoseStamped convertFromString(StringView str);
    template <> inline int convertFromString(StringView str);
    template <> inline std::deque<int> convertFromString(StringView str);
}

class Navigation : public BT::StatefulActionNode {

public:

    Navigation(const std::string& name, const BT::NodeConfig& config, const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
        : BT::StatefulActionNode(name, config), node_(params.nh), blackboard_(blackboard)
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
    BT::Blackboard::Ptr blackboard_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr predict_goal_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    double move_x_;
    double move_y_;
    double rival_velocity = 0.02;

    bool finished_ = false;
    geometry_msgs::msg::PoseStamped goal_;
    geometry_msgs::msg::PoseStamped start_pose_;
    nav_msgs::msg::Odometry current_pose_;
    geometry_msgs::msg::PoseStamped rival_predict_goal_;
};

class initPoints : public BT::SyncActionNode {

public: 
    initPoints(const std::string& name, const BT::NodeConfiguration& config, std::shared_ptr<rclcpp::Node> node) 
        : BT::SyncActionNode(name, config), node_(node) 
    {}
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
private:
    std::shared_ptr<rclcpp::Node> node_;
};

class StateUpdater : public BT::StatefulActionNode
{
public:
    StateUpdater(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node, BT::Blackboard::Ptr blackboard)
        : BT::StatefulActionNode(name, config), node_(node), blackboard_(blackboard)
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
    BT::Blackboard::Ptr blackboard_;

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr materials_pub_;
    std_msgs::msg::Int32MultiArray materials_info_;
    std::vector<int> material_status_;
    int pub_count = 0;
};

class PublishPose : public BT::SyncActionNode
{
public:
    PublishPose(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node, BT::Blackboard::Ptr blackboard)
        : BT::SyncActionNode(name, config), node_(node), blackboard_(blackboard)
    {}

    /* Node remapping function */
    static BT::PortsList providedPorts();

    /* Start and running function */
    BT::NodeStatus tick() override;

private:
    void timer_publisher();

    std::shared_ptr<rclcpp::Node> node_;
    BT::Blackboard::Ptr blackboard_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr rival_pub_;
    nav_msgs::msg::Odometry current_pose_;
};