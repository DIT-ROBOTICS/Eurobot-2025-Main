#include "bt_app_2025/bt_nodes_navigation.h"

using namespace BT;
using namespace std;

template <> inline geometry_msgs::msg::TwistStamped BT::convertFromString(StringView str) {

    auto parts = splitString(str, ',');
    if (parts.size() != 3) {
        throw RuntimeError("invalid input)");
    }
    else {
        geometry_msgs::msg::TwistStamped output;
        output.twist.linear.x = convertFromString<double>(parts[0]);
        output.twist.linear.y = convertFromString<double>(parts[1]);
        output.twist.linear.z = convertFromString<double>(parts[2]);
        return output;
    }
}

template <> inline int BT::convertFromString(StringView str) {
    auto value = convertFromString<double>(str);
    return (int) value;
}

template <> inline std::deque<int> BT::convertFromString(StringView str) {

    auto parts = splitString(str, ',');
    std::deque<int> output;

    for (int i = 0; i < (int)parts.size(); i++) {
        output.push_back(convertFromString<int>(parts[i]));
    }

    return output;
}

BT::PortsList Navigation::providedPorts() {
    return { 
        BT::InputPort<geometry_msgs::msg::TwistStamped>("goal"),
        BT::InputPort<int>("type"),
        BT::OutputPort<geometry_msgs::msg::TwistStamped>("result")
    };
}

bool Navigation::setGoal(RosActionNode::Goal& goal) {
    goal_ = getInput<geometry_msgs::msg::TwistStamped>("goal").value();
    nav_type_ = getInput<int>("type").value();
    goal.nav_goal = goal_;
    goal.nav_goal.twist.angular.x = nav_type_;

    RCLCPP_INFO(logger(), "Sending Goal (%f, %f), Navigation Type: %.f", goal.nav_goal.twist.linear.x, goal.nav_goal.twist.linear.y, goal.nav_goal.twist.angular.x);
    nav_finished_ = false;

    return true;
}

NodeStatus Navigation::onFeedback(const std::shared_ptr<const Feedback> feedback) {
    auto feedback_ = feedback->progress;
    return NodeStatus::RUNNING;
}

NodeStatus Navigation::onResultReceived(const WrappedResult& wr) {
    auto result_ = wr.result->outcome;
    nav_finished_ = true;
    setOutput<geometry_msgs::msg::TwistStamped>("result", goal_);
    return NodeStatus::SUCCESS;
}

NodeStatus Navigation::onFailure(ActionNodeErrorCode error) {
    nav_error_ = true;
    setOutput<geometry_msgs::msg::TwistStamped>("result", goal_);
    RCLCPP_ERROR(logger(), "[BT]: Navigation error");
    return NodeStatus::FAILURE;
}

BT::PortsList Docking::providedPorts() {
    return { 
        BT::InputPort<geometry_msgs::msg::TwistStamped>("base"),
        BT::InputPort<geometry_msgs::msg::TwistStamped>("offset"),
        BT::OutputPort<geometry_msgs::msg::TwistStamped>("result"),
        BT::InputPort<int>("mission_type")
    };
}

bool Docking::setGoal(RosActionNode::Goal& goal) {
    auto m = getInput<geometry_msgs::msg::TwistStamped>("base");
    auto offset = getInput<geometry_msgs::msg::TwistStamped>("offset");
    mission_type_ = getInput<int>("mission_type").value();
    goal_.twist.linear = m.value().twist.linear;
    goal_.twist.linear.x += offset.value().twist.linear.x;
    goal_.twist.linear.y += offset.value().twist.linear.y;
    goal_.twist.linear.z += offset.value().twist.linear.z;
    goal.nav_goal = goal_;
    goal.nav_goal.twist.angular.x = mission_type_;

    nav_finished_ = false;
    nav_error_ = false;

    return true;
}

NodeStatus Docking::onFeedback(const std::shared_ptr<const Feedback> feedback) {
    auto feedback_ = feedback->progress;
    return NodeStatus::RUNNING;
}

NodeStatus Docking::onResultReceived(const WrappedResult& wr) {
    auto result_ = wr.result->outcome;
    nav_finished_ = true;
    setOutput<geometry_msgs::msg::TwistStamped>("result", goal_);
    return NodeStatus::SUCCESS;
}

NodeStatus Docking::onFailure(ActionNodeErrorCode error) {
    nav_error_ = true;
    setOutput<geometry_msgs::msg::TwistStamped>("result", goal_);
    RCLCPP_ERROR(logger(), "[BT]: Navigation error");
    return NodeStatus::FAILURE;
}

// BT::PortsList DynamicAdjustment::providedPorts() {
//     return {
//         // To Do: 
//     }
// }

// BT::NodeStatus DynamicAdjustment::onStart() {
//     // To Do: 
// }

// BT::NodeStatus DynamicAdjustment::onRunning() {
//     // To Do: 
// }

// void DynamicAdjustment::onHalted() {
//     // To Do: 
// }