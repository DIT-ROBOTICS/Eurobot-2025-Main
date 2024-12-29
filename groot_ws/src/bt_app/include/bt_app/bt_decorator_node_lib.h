#pragma once

// Basic Libraries
#include <filesystem>
#include <fstream>
#include <mutex>

// ROS related Libraries
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

// Behavior Tree Libraries
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/decorators/loop_node.h"


namespace BT {
    template <> inline int convertFromString(StringView str);
}

class TickFlow : public BT::DecoratorNode {

public:
    TickFlow(const std::string& name, const BT::NodeConfig& config)
        : BT::DecoratorNode(name, config) {}

    /* Node remapping function */
    static BT::PortsList providedPorts();

    /* Start and running function */
    BT::NodeStatus tick() override;

private:
    int max_tick_count;
    int tick_count = 0;
    std::mutex mutex_lock_;

};

class ElapseTimeCheck : public BT::DecoratorNode {

public:
    ElapseTimeCheck(const std::string& name, const BT::NodeConfig& config)
        : BT::DecoratorNode(name, config) {}

    /* Node remapping function */
    static BT::PortsList providedPorts();

    /* Start and running function */
    BT::NodeStatus tick() override;

private:
    bool time_started = false;
    double max_time_count;
    double start_time = -1;
    double current_time;
    std::mutex mutex_lock_;

};

class RaceTimeCheck : public BT::DecoratorNode {

public:
    RaceTimeCheck(const std::string& name, const BT::NodeConfig& config)
        : BT::DecoratorNode(name, config) {}

    /* Node remapping function */
    static BT::PortsList providedPorts();

    /* Start and running function */
    BT::NodeStatus tick() override;

private:
    double max_time_count;
    double elapsed_time;
    std::mutex mutex_lock_;

};