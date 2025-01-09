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

// Use self define message
#include "btcpp_ros2_interfaces/action/nav_mission.hpp"

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
    // bool mec_callback_;
    bool start_mission_ = false;
    // std::shared_ptr<Kernel> kernel_;
    geometry_msgs::msg::TwistStamped goal_;

    int mission_type_ = 0;
};

/*******************************/
/* Recovery mode using docking */
/*******************************/
class Recovery : public BT::RosActionNode<btcpp_ros2_interfaces::action::NavMission> {

public:
    // Recovery(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Kernel> kernel, bool use_docking, bool mec_callback)
    //     : BT::StatefulActionNode(name, config), kernel_(kernel), use_docking_(use_docking), mec_callback_(mec_callback) {}
    Recovery(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
        : RosActionNode<btcpp_ros2_interfaces::action::NavMission>(name, conf, params)
    {}

    /* Node remapping function */
    static PortsList providedPorts();
    bool setGoal(RosActionNode::Goal& goal) override;
    NodeStatus onResultReceived(const WrappedResult& wr) override;
    virtual NodeStatus onFailure(ActionNodeErrorCode error) override;
    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback);

private:

    double distance_ = 0.2;
    bool nav_finished_ = false;
    bool nav_error_ = false;
    int mission_type_ = 0;
    bool start_mission_ = false;
    // std::shared_ptr<Kernel> kernel_;
    geometry_msgs::msg::TwistStamped base_;
};

// /*****************************************/
// /* Comparator to check the state is safe */
// /*****************************************/
// class Comparator : public BT::ConditionNode {

// public:
//     // Comparator(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Kernel> kernel)
//     //     : BT::ConditionNode(name, config), kernel_(kernel) {}
//     Comparator(const std::string& name, const BT::NodeConfig& config)
//         : BT::ConditionNode(name, config) {}

//     /* Node remapping function */
//     static BT::PortsList providedPorts();

//     /* Start and running function */
//     BT::NodeStatus tick() override;

// private:

//     bool CheckRivalInThreshold(geometry_msgs::msg::TwistStamped pose, double threshold, double range);
//     bool CheckRivalInRange(geometry_msgs::msg::TwistStamped pose, double range);
//     double Distance(geometry_msgs::msg::TwistStamped pose1, geometry_msgs::msg::TwistStamped pose2);

//     // std::shared_ptr<Kernel> kernel_;
//     geometry_msgs::msg::TwistStamped mission_;
// };

// /*****************************************/
// /* Mission - Send mission type to kernel */
// /*****************************************/
// class BTMission : public RosActionNode<btcpp_ros2_interfaces::action::NavMission> {

// public:
//     // BTMission(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<Kernel> kernel, bool mec_callback)
//     //     : BT::StatefulActionNode(name, config), kernel_(kernel), mec_callback_(mec_callback) {}
//     BTMission(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
//         : RosActionNode<btcpp_ros2_interfaces::action::NavMission>(name, conf, params)
//     {}

//     /* Node remapping function */
//     static PortsList providedPorts();
//     bool setGoal(RosActionNode::Goal& goal) override;
//     NodeStatus onResultReceived(const WrappedResult& wr) override;
//     virtual NodeStatus onFailure(ActionNodeErrorCode error) override;
//     NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback);

// private:

//     void SendMissionByQueue();

//     bool mec_callback_ = false;
//     bool mission_finished_ = false;
//     bool send_mission = true;
//     int current_mission_type_ = 0;
//     int current_mission_sub_type_ = 0;
//     // std::shared_ptr<Kernel> kernel_;

//     std::deque<int> mission_type_;
//     std::deque<int> mission_sub_type_;
// };

/******************************/
/* BTStarter - Start the tree */
/******************************/
class BTStarter : public BT::SyncActionNode {

public:
    BTStarter(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config){}

    /* Node remapping function */
    static BT::PortsList providedPorts();

    /* Start and running function */
    BT::NodeStatus tick() override;
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
    // std::shared_ptr<Kernel> kernel_;
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

    // Kernel
    // std::shared_ptr<Kernel> kernel_;
};

/***********************************/
/* Simple Node to activate ladybug */
/***********************************/
// class LadybugActivate : public BT::SyncActionNode {

// public:
//     LadybugActivate(const std::string& name, const BT::NodeConfig& config)
//         : BT::SyncActionNode(name, config) {}

//     /* Node remapping function */
//     static BT::PortsList providedPorts(){ return {}; }

//     /* Start and running function */
//     BT::NodeStatus tick() override;

// private:

//     // Kernel
//     // std::shared_ptr<Kernel> kernel_;
// };

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