#pragma once

// Use C++ libraries
#include <filesystem>
#include <fstream>
#include <deque>
#include <string>
#include <bitset>
#include <math.h>

// Use behavior tree
#include "behaviortree_ros2/bt_action_node.hpp"
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
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
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

#define PI 3.1415926

using namespace BT;

namespace BT {
    template <> inline int convertFromString(StringView str);
    template <> inline std::deque<int> convertFromString(StringView str);
}

class GetLocation : public BT::SyncActionNode {

public:
    GetLocation(const std::string& name, const BT::NodeConfig& config, const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
        : BT::SyncActionNode(name, config), node_(params.nh.lock()), blackboard_(blackboard), tf_buffer_(node_->get_clock()), listener_(tf_buffer_)
    {
        node_->get_parameter("frame_id", frame_id_);
        tf_buffer_.setUsingDedicatedThread(true);
    }

    /* Node remapping function */
    static BT::PortsList providedPorts();

    /* Start and running function */
    BT::NodeStatus tick() override;
private:
    BT::Blackboard::Ptr blackboard_;
    rclcpp::Node::SharedPtr node_;
    // for tf listener
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener listener_;
    std::string frame_id_;
    geometry_msgs::msg::PoseStamped pose_;
};

class GetBlackboard : public BT::SyncActionNode {

public:
    GetBlackboard(const std::string& name, const BT::NodeConfig& config, const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
        : BT::SyncActionNode(name, config), node_(params.nh.lock()), blackboard_(blackboard)
    {}

    /* Node remapping function */
    static BT::PortsList providedPorts();

    /* Start and running function */
    BT::NodeStatus tick() override;
private:
    std::shared_ptr<rclcpp::Node> node_;
    BT::Blackboard::Ptr blackboard_;
};

class MySetBlackboard : public BT::SyncActionNode {

public:
    MySetBlackboard(const std::string& name, const BT::NodeConfig& config, const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
        : BT::SyncActionNode(name, config), node_(params.nh.lock()), blackboard_(blackboard)
    {}

    /* Node remapping function */
    static BT::PortsList providedPorts();

    /* Start and running function */
    BT::NodeStatus tick() override;
private:
    std::shared_ptr<rclcpp::Node> node_;
    BT::Blackboard::Ptr blackboard_;
};

/******************************/
/* BTStarter - Start the tree */
/******************************/
// continuous update the current time
class BTStarter : public BT::SyncActionNode {

public:
    BTStarter(const std::string& name, const BT::NodeConfig& config, const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
        : BT::SyncActionNode(name, config), node_(params.nh.lock()), blackboard_(blackboard)
    {}

    /* Node remapping function */
    static BT::PortsList providedPorts();

    /* Start and running function */
    BT::NodeStatus tick() override;
private:
    void topic_callback(const std_msgs::msg::Float32::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr keepout_zone_pub_;
    std::shared_ptr<rclcpp::Node> node_;
    BT::Blackboard::Ptr blackboard_;
    std_msgs::msg::String keepout_zone_;
    float current_time_;
    std::string team_;
};

/****************/
/* TimerChecker */
/****************/
class TimerChecker : public BT::ConditionNode {

public:
    TimerChecker(const std::string& name, const NodeConfig& config, BT::Blackboard::Ptr blackboard)
        : BT::ConditionNode(name, config), blackboard_(blackboard) {}
    /* Node remapping function */
    static BT::PortsList providedPorts();
    /* Start and running function */
    BT::NodeStatus tick() override;

private:
    BT::Blackboard::Ptr blackboard_;
    double current_time_;
};

// /****************************************************/
// /* Simple Node for finding the rival start position */
// /****************************************************/
// class RivalStart : public BT::SyncActionNode {

// public:
//     RivalStart(const std::string& name, const BT::NodeConfig& config, std::string team)
//         : BT::SyncActionNode(name, config), team_(team) {}

//     /* Node remapping function */
//     static BT::PortsList providedPorts();

//     /* Start and running function */
//     BT::NodeStatus tick() override;

// private:

//     // Private variables
//     std::string team_;
//     int check_start_point_;

//     // Kernel
//     // std::shared_ptr<Kernel> kernel_;
// };
