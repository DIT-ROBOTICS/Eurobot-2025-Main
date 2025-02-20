#include "bt_app_2025/bt_nodes_receiver.h"

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

PortsList LocReceiver::providedPorts() {
    return { 
        BT::OutputPort<geometry_msgs::msg::TwistStamped>("robot_pose"),
        BT::OutputPort<geometry_msgs::msg::TwistStamped>("rival_pose") 
    };
}

NodeStatus LocReceiver::tick() {
    if (UpdateRobotPose() && UpdateRivalPose()) {
        std::cout << "robot_pose_:(" << robot_pose_.twist.linear.x << ", " << robot_pose_.twist.linear.y << ")\n";
        // Set the output port
        setOutput("robot_pose", robot_pose_);
        setOutput("rival_pose", rival_pose_);
        return BT::NodeStatus::SUCCESS;
    } else {
        return BT::NodeStatus::FAILURE;
    }
}

bool LocReceiver::UpdateRobotPose() {
    geometry_msgs::msg::TransformStamped transformStamped;

    try {
        transformStamped = tf_buffer_.lookupTransform(
            // "robot/map" /* Parent frame - map */, 
            // "robot/base_footprint" /* Child frame - base */,
            "map", 
            "base_link",
            rclcpp::Time()
        );

        /* Extract (x, y, theta) from the transformed stamped */
        robot_pose_.twist.linear.x = transformStamped.transform.translation.x;
        robot_pose_.twist.linear.y = transformStamped.transform.translation.y;
        double theta;
        tf2::Quaternion q;
        tf2::fromMsg(transformStamped.transform.rotation, q);
        robot_pose_.twist.angular.z = tf2::impl::getYaw(q);

        return true;
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN_STREAM(node_->get_logger(), "[Kernel::UpdateRobotPose]: line " << __LINE__ << " " << ex.what());

        return false;
    }
}

bool LocReceiver::UpdateRivalPose() {
    geometry_msgs::msg::TransformStamped transformStamped;

    try {
        transformStamped = tf_buffer_.lookupTransform(
            "rival/map" /* Parent frame - map */, 
            "rival/base_footprint" /* Child frame - base */,
            rclcpp::Time()
        );

        /* Extract (x, y, theta) from the transformed stamped */
        rival_pose_.twist.linear.x = transformStamped.transform.translation.x;
        rival_pose_.twist.linear.y = transformStamped.transform.translation.y;
        double theta;
        tf2::Quaternion q;
        tf2::fromMsg(transformStamped.transform.rotation, q);
        rival_pose_.twist.angular.z = tf2::impl::getYaw(q);

        return true;
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN_STREAM(node_->get_logger(), "[Kernel::UpdateRobotPose]: line " << __LINE__ << " " << ex.what());

        return false;
    }
}

PortsList NavReceiver::providedPorts() {
    return {};
}

void NavReceiver::topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    rival_predict_goal_ = *msg;
    blackboard_->set<geometry_msgs::msg::PoseStamped>("rival_predict_goal", rival_predict_goal_);
}

BT::NodeStatus NavReceiver::tick() {
    RCLCPP_INFO(node_->get_logger(), "Node start");
    subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("rival/predict_goal", 10, std::bind(&NavReceiver::topic_callback, this, std::placeholders::_1));
    
    return BT::NodeStatus::SUCCESS;
}

PortsList CamReceiver::providedPorts() {
    return {};
}

void CamReceiver::materials_info_callback(const geometry_msgs::msg::PoseArray msg) {
    materials_info_ = *msg;
    // blackboard_->set<double>("global_info", materials_info_);
    // To Do: modify message type
}
void CamReceiver::banner_info_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    banner_info_ = *msg;
    // blackboard_->set<double>("banner_info", banner_info_);
    // To Do: modify message type
}
void CamReceiver::obstacles_info_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    obstacles_info_ = *msg;
    // blackboard_->set<double>("local_info", obstacles_info_);
    // To Do: modify message type
    // To Do: generate materials_info_ & garbage_points_
}

BT::NodeStatus CamReceiver::tick() {
    RCLCPP_INFO(node_->get_logger(), "Node start");
    sub_materials_info_ = node_->create_subscription<geometry_msgs::msg::PoseArray>("/robot/objects/materials_info", 10, std::bind(&CamReceiver::materials_info_callback, this, std::placeholders::_1));
    sub_banner_info_ = node_->create_subscription<std_msgs::msg::Int32>("/robot/objects/banner_info", 10, std::bind(&CamReceiver::banner_info_callback, this, std::placeholders::_1));
    sub_obstacles_info_ = node_->create_subscription<geometry_msgs::msg::PoseArray>("/robot/objects/obstacles_info", 10, std::bind(&CamReceiver::obstacles_info_callback, this, std::placeholders::_1));
    
    return BT::NodeStatus::SUCCESS;
}