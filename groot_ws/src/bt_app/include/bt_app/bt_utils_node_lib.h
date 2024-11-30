#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/decorators/loop_node.h"
#include <deque>

// ROS kernel
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

class GeneratePathPoint : public BT::SyncActionNode {

public: 
    GeneratePathPoint(const std::string& name, const BT::NodeConfiguration& config) 
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:

};

class StartRace : public BT::SyncActionNode {

public: 
    StartRace(const std::string& name, const BT::NodeConfiguration& config) 
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    // std::shared_ptr<Kernel> kernel_;

};