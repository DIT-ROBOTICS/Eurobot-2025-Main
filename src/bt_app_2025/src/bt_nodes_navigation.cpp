#include "bt_app_2025/bt_nodes_navigation.h"

using namespace BT;
using namespace std;

template <> inline geometry_msgs::msg::PoseStamped BT::convertFromString(StringView str) {

    auto parts = splitString(str, ',');
    if (parts.size() != 3) {
        throw RuntimeError("invalid input)");
    }
    else {
        geometry_msgs::msg::PoseStamped output;
        output.pose.position.x = convertFromString<double>(parts[0]);
        output.pose.position.y = convertFromString<double>(parts[1]);
        output.pose.position.z = convertFromString<double>(parts[2]);
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
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
        BT::InputPort<int>("type"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("final_pose")
    };
}

bool Navigation::setGoal(RosActionNode::Goal& goal) {
    auto m = getInput<geometry_msgs::msg::PoseStamped>("goal");
    nav_type_ = getInput<int>("type").value();
    // goal_.pose.position = m.value().pose.position;
    rclcpp::Time now = this->now();
    goal_.header.stamp = now;
    goal_.header.frame_id = "map";
    goal_.pose.position.x = m.value().pose.position.x;
    goal_.pose.position.y = m.value().pose.position.y;
    goal_.pose.position.z = m.value().pose.position.z;
    tf2::Quaternion q;
    q.setRPY(0, 0, goal_.pose.position.z);
    goal_.pose.orientation.x = q.x();
    goal_.pose.orientation.y = q.y();
    goal_.pose.orientation.z = q.z();
    goal_.pose.orientation.w = q.w();
    // goal.nav_goal.pose.angular.x = nav_type_;
    goal.pose = goal_;

    RCLCPP_INFO(logger(), "Start Nav (%f, %f)", goal.pose.pose.position.x, goal.pose.pose.position.y);
    current_pose_ = goal_;
    nav_finished_ = false;

    return true;
}

NodeStatus Navigation::onFeedback(const std::shared_ptr<const Feedback> feedback) {
    current_pose_ = feedback->current_pose;
    // RCLCPP_INFO_STREAM(logger(), "current_pose: " << current_pose_.pose.position.x << ", " << current_pose_.pose.position.y);
    return NodeStatus::RUNNING;
}

NodeStatus Navigation::onResultReceived(const WrappedResult& wr) {
    // auto result_ = wr.result->outcome;
    nav_finished_ = true;
    setOutput<geometry_msgs::msg::PoseStamped>("final_pose", current_pose_);
    RCLCPP_INFO_STREAM(logger(), "final_pose: " << current_pose_.pose.position.x << ", " << current_pose_.pose.position.y);
    return NodeStatus::SUCCESS;
}

NodeStatus Navigation::onFailure(ActionNodeErrorCode error) {
    nav_error_ = true;
    setOutput<geometry_msgs::msg::PoseStamped>("final_pose", current_pose_);
    RCLCPP_ERROR(logger(), "[BT]: Navigation error");
    return NodeStatus::FAILURE;
}

BT::PortsList Docking::providedPorts() {
    return { 
        BT::InputPort<geometry_msgs::msg::PoseStamped>("base"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("offset"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("final_pose"),
        BT::InputPort<int>("mission_type")
    };
}

bool Docking::setGoal(RosActionNode::Goal& goal) {
    auto m = getInput<geometry_msgs::msg::PoseStamped>("base");
    auto offset = getInput<geometry_msgs::msg::PoseStamped>("offset");
    mission_type_ = getInput<int>("mission_type").value();

    goal_.pose.position = m.value().pose.position;
    rclcpp::Time now = this->now();
    goal_.header.stamp = now;
    goal_.header.frame_id = "map";
    goal_.pose.position.x += offset.value().pose.position.x;
    goal_.pose.position.y += offset.value().pose.position.y;
    goal_.pose.position.z += offset.value().pose.position.z;
    // goal.nav_goal.pose.angular.x = nav_type_;
    goal.pose = goal_;

    RCLCPP_INFO(logger(), "Start Docking (%f, %f)", goal.pose.pose.position.x, goal.pose.pose.position.y);

    current_pose_ = goal_;
    nav_finished_ = false;
    nav_error_ = false;

    return true;
}

NodeStatus Docking::onFeedback(const std::shared_ptr<const Feedback> feedback) {
    current_pose_ = feedback->current_pose;
    // RCLCPP_INFO_STREAM(logger(), "current_pose: " << current_pose_.pose.position.x << ", " << current_pose_.pose.position.y);
    return NodeStatus::RUNNING;
}

NodeStatus Docking::onResultReceived(const WrappedResult& wr) {
    // auto result_ = wr.result->outcome;
    nav_finished_ = true;
    setOutput<geometry_msgs::msg::PoseStamped>("final_pose", current_pose_);
    RCLCPP_INFO_STREAM(logger(), "final_pose: " << current_pose_.pose.position.x << ", " << current_pose_.pose.position.y);
    return NodeStatus::SUCCESS;
}

NodeStatus Docking::onFailure(ActionNodeErrorCode error) {
    nav_error_ = true;
    setOutput<geometry_msgs::msg::PoseStamped>("final_pose", current_pose_);
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