#pragma once

// Use C++ libraries
#include <filesystem>
#include <fstream>
#include <deque>
#include <bitset>
#include <math.h>

// Use behavior tree
#include "behaviortree_cpp/decorators/loop_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/blackboard.h"

// Use ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

// Use ros message
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

// tf2 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/impl/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

using namespace BT;

namespace BT {
    template <> inline geometry_msgs::msg::PoseStamped convertFromString(StringView str);
    template <> inline int convertFromString(StringView str);
    template <> inline std::deque<int> convertFromString(StringView str);
}

/******************************/
/* BTStarter - Start the tree */
/******************************/
// continuous update the current time
class BTStarter : public BT::SyncActionNode {

public:
    BTStarter(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<rclcpp::Node> node, BT::Blackboard::Ptr blackboard)
        : BT::SyncActionNode(name, config), node_(node), blackboard_(blackboard)
    {}

    /* Node remapping function */
    static BT::PortsList providedPorts();

    /* Start and running function */
    BT::NodeStatus tick() override;
private:
    void topic_callback(const std_msgs::msg::Float32::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
    std::shared_ptr<rclcpp::Node> node_;
    BT::Blackboard::Ptr blackboard_;

    float current_time_;
};

/******************************/
/* BTFinisher - Send mission type to kernel */
/******************************/
// No useage now
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
// No usage now
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
// No usage now
class TimerChecker : public BT::DecoratorNode {

public:
    TimerChecker(const std::string& name, const NodeConfig& config, BT::Blackboard::Ptr blackboard)
        : BT::DecoratorNode(name, config), blackboard_(blackboard) {}

    /* Node remapping function */
    static BT::PortsList providedPorts();

    /* Start and running function */
    BT::NodeStatus tick() override;

private:
    BT::Blackboard::Ptr blackboard_;

    double timeout_ = 0.0;
    double start_time_ = 0.0;
    double current_time_;
    bool first_log_;
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
