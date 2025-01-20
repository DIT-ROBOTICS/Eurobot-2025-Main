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

/******************************/
/* BTStarter - Start the tree */
/******************************/
class BTStarter : public BT::SyncActionNode {

public:
    BTStarter(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config), node_(node) {
        subscription_ = node_->create_subscription<std_msgs::msg::Float32>("/robot/startup/time", 10, std::bind(&BTStarter::topic_callback, this, std::placeholders::_1));
    }

    /* Node remapping function */
    static BT::PortsList providedPorts();

    /* Start and running function */
    BT::NodeStatus tick() override;
private:
    void topic_callback(const std_msgs::msg::Float32::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
    std::shared_ptr<rclcpp::Node> node_;

    float current_time_;
};

/* BTFinisher - Send mission type to kernel */
class BTFinisher : public BT::SyncActionNode {

public:
    BTFinisher(const std::string& name, const BT::NodeConfig& config, std::string file, int team, std::shared_ptr<rclcpp::Node> node)
        : BT::SyncActionNode(name, config), score_filepath(file), team_(team),  node_(node){}

    /* Node remapping function */
    static BT::PortsList providedPorts();

    /* Start and running function */
    BT::NodeStatus tick() override;

    std::string score_filepath;

    int team_;
    std::shared_ptr<rclcpp::Node> node_;
};

/*****************************************/
/* Comparator to check the state is safe */
/*****************************************/
class Comparator : public BT::ConditionNode {

public:
    Comparator(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node)
        : BT::ConditionNode(name, config), node_(node), tf_buffer_(node_->get_clock()), listener_(tf_buffer_)
    {}

    /* Node remapping function */
    static BT::PortsList providedPorts();

    /* Start and running function */
    BT::NodeStatus tick() override;

private:

    geometry_msgs::msg::TwistStamped UpdateRobotPose();
    geometry_msgs::msg::TwistStamped UpdateRivalPose();
    bool CheckRivalInThreshold(geometry_msgs::msg::TwistStamped pose, double threshold, double range);
    bool CheckRivalInRange(geometry_msgs::msg::TwistStamped pose, double range);
    double Distance(geometry_msgs::msg::TwistStamped pose1, geometry_msgs::msg::TwistStamped pose2);

    geometry_msgs::msg::TwistStamped mission_;

    std::shared_ptr<rclcpp::Node> node_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener listener_;
};

/****************************/
/* TimerChecker - Decorator */
/****************************/
class TimerChecker : public BT::DecoratorNode {

public:
    TimerChecker(const std::string& name, const NodeConfig& config)
        : BT::DecoratorNode(name, config) {}

    /* Node remapping function */
    static BT::PortsList providedPorts();

    /* Start and running function */
    BT::NodeStatus tick() override;

private:

    double timeout_ = 0.0;
    double start_time_ = 0.0;
    bool first_log_ = true;
};


// /****************************************************/
// /* Simple Node for finding the rival start position */
// /****************************************************/
// class RivalStart : public BT::SyncActionNode {

// public:
//     RivalStart(const std::string& name, const BT::NodeConfig& config, int team)
//         : BT::SyncActionNode(name, config), team_(team) {}

//     /* Node remapping function */
//     static BT::PortsList providedPorts();

//     /* Start and running function */
//     BT::NodeStatus tick() override;

// private:

//     // Private variables
//     int team_;
//     int check_start_point_;

//     // Kernel
//     // std::shared_ptr<Kernel> kernel_;
// };
