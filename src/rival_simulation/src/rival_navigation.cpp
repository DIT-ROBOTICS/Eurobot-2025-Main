#include "rival_simulation/rival_navigation.h"

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
    return position1.distance(position2);
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
        BT::InputPort<geometry_msgs::msg::PoseStamped>("start_pose"), 
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
        BT::InputPort<int>("goal_index"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("final_pose")
    };
}

inline geometry_msgs::msg::PoseStamped Navigation::ExportPose(const int index) {
    node_->get_parameter("material_points", material_points_);
    geometry_msgs::msg::PoseStamped pose_;
    pose_.pose.position.x = material_points_[3 * index];
    pose_.pose.position.y = material_points_[3 * index + 1];
    pose_.pose.position.z = material_points_[3 * index + 2] * PI / 2;
    return pose_;
}

// Start function for the SimpleNavigation node
BT::NodeStatus Navigation::onStart() {
    auto g = &getInput<geometry_msgs::msg::PoseStamped>("goal");
    auto g_i = &getInput<int>("goal_index");
    goal = (g != NULL) ? g->value() : ExportPose(g_i->value());
    getInput<geometry_msgs::msg::PoseStamped>("start_pose", start_pose_);
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    predict_goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("rival/predict_goal", 20);

    current_pose_.pose.pose.position = start_pose_.pose.position;
    RCLCPP_INFO(node_->get_logger(), "Start Nav");
    return BT::NodeStatus::RUNNING;
}

double Navigation::calculateDistance(const geometry_msgs::msg::Pose &pose1, const geometry_msgs::msg::Pose &pose2) {
    tf2::Vector3 position1(pose1.position.x, pose1.position.y, 0);
    tf2::Vector3 position2(pose2.position.x, pose2.position.y, 0);
    return position1.distance(position2);
}

void Navigation::broadcastTransform(const geometry_msgs::msg::Pose &pose) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = node_->get_clock()->now();
    transform.header.frame_id = "rival/map";
    transform.child_frame_id = "rival/base_footprint";

    transform.transform.translation.x = pose.position.x;
    transform.transform.translation.y = pose.position.y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = pose.orientation.x;
    transform.transform.rotation.y = pose.orientation.y;
    transform.transform.rotation.z = pose.orientation.z;
    transform.transform.rotation.w = pose.orientation.w;

    broadcaster_->sendTransform(transform);
}

// Running function for the SimpleNavigation node
BT::NodeStatus Navigation::onRunning() {
    double distance = calculateDistance(current_pose_.pose.pose, goal_.pose);
    move_x_ = rival_velocity * (goal_.pose.position.x - current_pose_.pose.pose.position.x) / distance;
    move_y_ = rival_velocity * (goal_.pose.position.y - current_pose_.pose.pose.position.y) / distance;
    rclcpp::Rate rate(20);
    while (calculateDistance(current_pose_.pose.pose, goal_.pose) >= 0.03) {
        current_pose_.header.stamp = node_->now();
        current_pose_.header.frame_id = "map";
        current_pose_.pose.pose.position.x += move_x_;
        current_pose_.pose.pose.position.y += move_y_;
        current_pose_.twist.twist.linear.x = rival_velocity;
        current_pose_.twist.twist.linear.y = rival_velocity;
        predict_goal_pub_->publish(goal_);
        broadcastTransform(current_pose_.pose.pose);
        blackboard_->set<nav_msgs::msg::Odometry>("current_pose", current_pose_);
        RCLCPP_INFO(node_->get_logger(), "Navigating");
        rate.sleep();
    }
    geometry_msgs::msg::PoseStamped final_pose_;
    final_pose_.pose = current_pose_.pose.pose;
    setOutput<geometry_msgs::msg::PoseStamped>("final_pose", final_pose_);
    RCLCPP_INFO(node_->get_logger(), "Nav finish");
    return BT::NodeStatus::SUCCESS;
}

void Navigation::onHalted() {
    RCLCPP_INFO(node_->get_logger(), "Node halted");
    return;
}

// BT::PortsList initPoints::providedPorts() {
//     return {
//         BT::OutputPort<BT::SharedQueue<geometry_msgs::msg::PoseStamped>>("material_points")
//     };
// }

// BT::NodeStatus initPoints::tick() {
//     auto material_shared_queue = std::make_shared<std::deque<geometry_msgs::msg::PoseStamped>>();
//     std::vector<std::string> points_name_ = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9"};
//     node_->declare_parameter<std::vector<double>>(points_name_[0], {2.1, 1.7, 0.0, 0.0, 0.0, 0.707, 0.707});
//     node_->declare_parameter<std::vector<double>>(points_name_[1], {2.7, 1.3, 0.0, 0.0, 0.0, 0.0, 1.0});
//     node_->declare_parameter<std::vector<double>>(points_name_[2], {2.7, 0.4, 0.0, 0.0, 0.0, 0.0, 1.0});
//     node_->declare_parameter<std::vector<double>>(points_name_[3], {1.9, 0.8, 0.0, 0.0, 0.0, 0.707, 0.707});
//     node_->declare_parameter<std::vector<double>>(points_name_[4], {2.2, 0.4, 0.0, 0.0, 0.0, -0.707, 0.707});
//     node_->declare_parameter<std::vector<double>>(points_name_[5], {1.1, 0.8, 0.0, 0.0, 0.0, 0.707, 0.707});
//     node_->declare_parameter<std::vector<double>>(points_name_[6], {0.8, 0.4, 0.0, 0.0, 0.0, -0.707, 0.707});
//     node_->declare_parameter<std::vector<double>>(points_name_[7], {0.3, 1.3, 0.0, 0.0, 0.0, 1.0, 0.0});
//     node_->declare_parameter<std::vector<double>>(points_name_[8], {0.3, 0.4, 0.0, 0.0, 0.0, 1.0, 0.0});
//     node_->declare_parameter<std::vector<double>>(points_name_[9], {0.8, 1.7, 0.0, 0.0, 0.0, 0.707, 0.707});
//     for (const auto &point_name_ : points_name_) {
//         std::vector<double> param;
//         node_->get_parameter(point_name_, param);
//         geometry_msgs::msg::PoseStamped path_point;
//         path_point.pose.position.x = param[0];
//         path_point.pose.position.y = param[1];
//         path_point.pose.position.z = param[2];
//         path_point.pose.orientation.x = param[3];
//         path_point.pose.orientation.y = param[4];
//         path_point.pose.orientation.z = param[5];
//         path_point.pose.orientation.w = param[6];
//         // Store in shared_queue
//         material_shared_queue->push_back(path_point);
//     }
//     setOutput("material_points", material_shared_queue);

//     RCLCPP_INFO(node_->get_logger(), "Init points");

//     return BT::NodeStatus::SUCCESS;
// }

BT::PortsList StateUpdater::providedPorts() {
    return {
        BT::InputPort<int>("changed_point_code")
    };
}

BT::NodeStatus StateUpdater::onStart() {
    RCLCPP_INFO(node_->get_logger(), "Start update state");
    int pt_num_ = getInput<int>("changed_point_code").value();
    blackboard_->get<std::vector<int>>("material_status", material_status_);
    material_status_[pt_num_] = 0;
    blackboard_->set<std::vector<int>>("material_status", material_status_);
    materials_info_.data.clear();
    for (int item : material_status_) {
        materials_info_.data.push_back(item);
    }

    materials_pub_ = node_->create_publisher<std_msgs::msg::Int32MultiArray>("/detected/global_center_poses/has_material", 10);
    return NodeStatus::RUNNING;
}

BT::NodeStatus StateUpdater::onRunning() {
    pub_count++;
    materials_pub_->publish(materials_info_);
    if (pub_count == 100) {
        RCLCPP_INFO(node_->get_logger(), "Finish update state");
        return BT::NodeStatus::SUCCESS;
    }
    return NodeStatus::RUNNING;
}

void StateUpdater::onHalted() {
    RCLCPP_INFO_STREAM(node_->get_logger(), "Node halted");
    return;
}

PortsList PublishPose::providedPorts() {
    return {};
}

void PublishPose::timer_publisher() {
    rclcpp::Rate rate(100);
    while (rclcpp::ok()) { 
        blackboard_->get("current_pose", current_pose_);
        tf_.header.stamp = node_->get_clock()->now();
        tf_.header.frame_id = "map";
        tf_.child_frame_id = "rival/" + frame_id_;
        tf_.transform.translation.x = current_pose_.pose.pose.position.x;
        tf_.transform.translation.y = current_pose_.pose.pose.position.y;
        tf_broadcaster_->sendTransform(tf_);
        rival_pub_->publish(current_pose_);
        RCLCPP_INFO_STREAM(node_->get_logger(), current_pose_.pose.pose.position.x << ", " << current_pose_.pose.pose.position.y);
        rate.sleep();
    }
}

BT::NodeStatus PublishPose::tick() {
    rival_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/rival/final_pose", 20);
    current_pose_.pose.pose.position.x = 0;
    current_pose_.pose.pose.position.y = 0;
    current_pose_.twist.twist.linear.x = 0;
    current_pose_.twist.twist.linear.y = 0;
    blackboard_->set<nav_msgs::msg::Odometry>("current_pose", current_pose_);

    std::thread{std::bind(&PublishPose::timer_publisher, this)}.detach();
    
    return BT::NodeStatus::SUCCESS;
}