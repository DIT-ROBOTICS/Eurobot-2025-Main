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

double inline calculateDistance(const geometry_msgs::msg::Pose &pose1, const geometry_msgs::msg::Pose &pose2) {
    tf2::Vector3 position1(pose1.position.x, pose1.position.y, 0);
    tf2::Vector3 position2(pose2.position.x, pose2.position.y, 0);
    double dist = position1.distance(position2);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "distance: " << dist);
    return dist;
}

double inline calculateAngleDifference(const geometry_msgs::msg::Pose &pose1, const geometry_msgs::msg::Pose &pose2)
{
    tf2::Quaternion orientation1, orientation2;
    tf2::fromMsg(pose1.orientation, orientation1);
    tf2::fromMsg(pose2.orientation, orientation2);
    double yaw1 = tf2::impl::getYaw(orientation1);
    double yaw2 = tf2::impl::getYaw(orientation2);
    return std::fabs(yaw1 - yaw2);
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
    rclcpp::Time now = this->now();
    goal_.header.stamp = now;
    goal_.header.frame_id = "map";
    goal_.pose.position.x = m.value().pose.position.x;
    goal_.pose.position.y = m.value().pose.position.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, m.value().pose.position.z);
    goal_.pose.orientation.x = q.x();
    goal_.pose.orientation.y = q.y();
    goal_.pose.orientation.z = q.z();
    goal_.pose.orientation.w = q.w();
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
    // set output
    switch (wr.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was aborted");
        return NodeStatus::SUCCESS;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was canceled");
        return NodeStatus::SUCCESS;
    default:
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown result code");
        return NodeStatus::SUCCESS;
    }
    setOutput<geometry_msgs::msg::PoseStamped>("final_pose", current_pose_);
    // check if mission success
    if (calculateDistance(current_pose_.pose, goal_.pose) < 0.03 && calculateAngleDifference(current_pose_.pose, goal_.pose) < 0.4) {
        nav_finished_ = true;
        RCLCPP_INFO_STREAM(logger(), "success! final_pose: " << current_pose_.pose.position.x << ", " << current_pose_.pose.position.y);
        return NodeStatus::SUCCESS;
    } else {
        nav_error_ = true;
        RCLCPP_INFO_STREAM(logger(), "fail! final_pose: " << current_pose_.pose.position.x << ", " << current_pose_.pose.position.y);
        return NodeStatus::SUCCESS;
    }
}

NodeStatus Navigation::onFailure(ActionNodeErrorCode error) {
    nav_error_ = true;
    setOutput<geometry_msgs::msg::PoseStamped>("final_pose", current_pose_);
    RCLCPP_INFO_STREAM(logger(), "fail! final_pose: " << current_pose_.pose.position.x << ", " << current_pose_.pose.position.y);
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

    goal_.pose = m.value().pose;
    rclcpp::Time now = this->now();
    goal_.header.stamp = now;
    goal_.header.frame_id = "map";
    goal_.pose.position.x += offset.value().pose.position.x;
    goal_.pose.position.y += offset.value().pose.position.y;
    goal.pose = goal_;

    RCLCPP_INFO(logger(), "Start Docking (%f, %f)", goal.pose.pose.position.x, goal.pose.pose.position.y);

    current_pose_ = goal_;
    nav_finished_ = false;
    nav_error_ = false;

    return true;
}

NodeStatus Docking::onFeedback(const std::shared_ptr<const Feedback> feedback) {
    current_pose_ = feedback->current_pose;
    return NodeStatus::RUNNING;
}

NodeStatus Docking::onResultReceived(const WrappedResult& wr) {
    // set output
    switch (wr.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was aborted");
        return NodeStatus::SUCCESS;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was canceled");
        return NodeStatus::SUCCESS;
    default:
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown result code");
        return NodeStatus::SUCCESS;
    }
    setOutput<geometry_msgs::msg::PoseStamped>("final_pose", current_pose_);
    // check if mission success
    if (calculateDistance(current_pose_.pose, goal_.pose) < 0.03 && calculateAngleDifference(current_pose_.pose, goal_.pose) < 0.4) {
        nav_finished_ = true;
        RCLCPP_INFO_STREAM(logger(), "success! final_pose: " << current_pose_.pose.position.x << ", " << current_pose_.pose.position.y);
        return NodeStatus::SUCCESS;
    } else {
        nav_error_ = true;
        RCLCPP_INFO_STREAM(logger(), "fail! final_pose: " << current_pose_.pose.position.x << ", " << current_pose_.pose.position.y);
        return NodeStatus::SUCCESS;
    }
}

NodeStatus Docking::onFailure(ActionNodeErrorCode error) {
    nav_error_ = true;
    setOutput<geometry_msgs::msg::PoseStamped>("final_pose", current_pose_);
    RCLCPP_INFO_STREAM(logger(), "fail! final_pose: " << current_pose_.pose.position.x << ", " << current_pose_.pose.position.y);
    return NodeStatus::FAILURE;
}

BT::PortsList Rotation::providedPorts() {
    return { 
        BT::InputPort<geometry_msgs::msg::PoseStamped>("base"),
        BT::InputPort<double>("degree"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("final_pose")
    };
}

bool Rotation::setGoal(RosActionNode::Goal& goal) {
    auto m = getInput<geometry_msgs::msg::PoseStamped>("base");
    double rad = getInput<double>("degree").value() * PI / 180;
    double rad_base = acos(m.value().pose.orientation.w) * 2;
    rad += rad_base;

    rclcpp::Time now = this->now();
    goal_.header.stamp = now;
    goal_.header.frame_id = "map";

    goal_.pose = m.value().pose;
    tf2::Quaternion q;
    q.setRPY(0, 0, rad);
    goal_.pose.orientation.x = q.x();
    goal_.pose.orientation.y = q.y();
    goal_.pose.orientation.z = q.z();
    goal_.pose.orientation.w = q.w();
    goal.pose = goal_;

    RCLCPP_INFO(logger(), "Start Rotating (%f, %f)", goal_.pose.orientation.z, goal_.pose.orientation.w);

    current_pose_ = goal_;
    nav_finished_ = false;
    nav_error_ = false;

    return true;
}

NodeStatus Rotation::onFeedback(const std::shared_ptr<const Feedback> feedback) {
    current_pose_ = feedback->current_pose;
    return NodeStatus::RUNNING;
}

NodeStatus Rotation::onResultReceived(const WrappedResult& wr) {
    // set output
    setOutput<geometry_msgs::msg::PoseStamped>("final_pose", current_pose_);
    // check if mission success
    if (calculateDistance(current_pose_.pose, goal_.pose) < 0.03 && calculateAngleDifference(current_pose_.pose, goal_.pose) < 0.4) {
        nav_finished_ = true;
        RCLCPP_INFO(logger(), "success! final_direction: (%f, %f)", current_pose_.pose.orientation.z, current_pose_.pose.orientation.w);
        return NodeStatus::SUCCESS;
    } else {
        nav_error_ = true;
        RCLCPP_INFO(logger(), "fail! final_direction: (%f, %f)", current_pose_.pose.orientation.z, current_pose_.pose.orientation.w);
        return NodeStatus::SUCCESS;
    }
}

NodeStatus Rotation::onFailure(ActionNodeErrorCode error) {
    nav_error_ = true;
    setOutput<geometry_msgs::msg::PoseStamped>("final_pose", current_pose_);
    RCLCPP_ERROR(logger(), "[BT]: Navigation error");
    return NodeStatus::FAILURE;
}
BT::PortsList DynamicAdjustment::providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::TwistStamped>("robot_pose"),
        BT::InputPort<geometry_msgs::msg::TwistStamped>("rival_pose")
        // To Do: 
    };
}

BT::NodeStatus DynamicAdjustment::onStart() {
    // To Do: 
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DynamicAdjustment::onRunning() {
    // To Do: 
    return BT::NodeStatus::SUCCESS;
}

void DynamicAdjustment::onHalted() {
    // To Do: 
    return;
}