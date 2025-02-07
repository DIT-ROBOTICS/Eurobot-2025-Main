#include "rival_simulation/bt_nodes_navigation.h"

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

BT::PortsList PointProvider::providedPorts() {
    return { 
        BT::InputPort<geometry_msgs::msg::PoseStamped>("point_in"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("point_out1"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("point_out2") 
    };
}

BT::NodeStatus PointProvider::tick() {
    point = getInput<geometry_msgs::msg::PoseStamped>("point_in").value();
    setOutput<geometry_msgs::msg::PoseStamped>("point_out1", point);
    point.pose.position.x *= -1;
    point.pose.position.y *= -1;
    setOutput<geometry_msgs::msg::PoseStamped>("point_out2", point);

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList Navigation::providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("start_pose"), 
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("final_pose")
    };
}

// Start function for the SimpleNavigation node
BT::NodeStatus Navigation::onStart() {
    getInput<geometry_msgs::msg::PoseStamped>("goal", goal_);
    current_pose_.pose.pose = getInput<geometry_msgs::msg::PoseStamped>("start_pose").value().pose;
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    // rival_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/rival_pose", 20);
    // predict_goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("rival/predict_goal", 20);

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
    while (calculateDistance(current_pose_.pose.pose, goal_.pose) >= 0.3) {
        current_pose_.header.stamp = node_->now();
        current_pose_.header.frame_id = "map";
        current_pose_.pose.pose.position.x += move_x_;
        current_pose_.pose.pose.position.y += move_y_;
        // predict_goal_pub_->publish(goal_);
        broadcastTransform(current_pose_.pose.pose);
        rival_pub_->publish(current_pose_);
        RCLCPP_INFO(node_->get_logger(), "Navigating");
        rate.sleep();
    }
    // predict_goal_pub_.reset();
    rival_pub_.reset();
    RCLCPP_INFO(node_->get_logger(), "Nav finish");
    return BT::NodeStatus::SUCCESS;
}

// Halted function for the SimpleNavigation node
void Navigation::onHalted() {
    // ROS_INFO_STREAM("Node halted");
    return;
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
    setOutput<geometry_msgs::msg::PoseStamped>("final_pose", current_pose_);
    // check if mission success
    if (calculateDistance(current_pose_.pose, goal_.pose) < 0.03 && calculateAngleDifference(current_pose_.pose, goal_.pose) < 0.4) {
        nav_finished_ = true;
        RCLCPP_INFO_STREAM(logger(), "success! final_pose: " << current_pose_.pose.position.x << ", " << current_pose_.pose.position.y);
        return NodeStatus::SUCCESS;
    } else {
        nav_error_ = true;
        RCLCPP_INFO_STREAM(logger(), "fail! final_pose: " << current_pose_.pose.position.x << ", " << current_pose_.pose.position.y);
        return NodeStatus::FAILURE;
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
        return NodeStatus::FAILURE;
    }
}

NodeStatus Rotation::onFailure(ActionNodeErrorCode error) {
    nav_error_ = true;
    setOutput<geometry_msgs::msg::PoseStamped>("final_pose", current_pose_);
    RCLCPP_ERROR(logger(), "[BT]: Navigation error");
    return NodeStatus::FAILURE;
}

// BT::PortsList GeneratePathPoint::providedPorts() {
//     return {
//         BT::OutputPort<BT::SharedQueue<geometry_msgs::msg::PoseStamped>>("path_point")
//     };
// }

// BT::NodeStatus GeneratePathPoint::tick() {

//     auto shared_queue = std::make_shared<std::deque<geometry_msgs::msg::PoseStamped>>();
    
//     node_->declare_parameter<std::vector<double>>("waypoint1", {});
//     std::vector<std::vector<double>> params;
//     for (const auto &param : params) {

    // }
    // node_->get_parameter("waypoints", params);
    // for (const auto &param : params)
    // {
    //     geometry_msgs::msg::PoseStamped path_point;
    //     path_point.header.frame_id = "map";
    //     path_point.pose.position.x = param[0];
    //     path_point.pose.position.y = param[1];
    //     path_point.pose.position.z = param[2];

    //     path_point.pose.orientation.x = param[3];
    //     path_point.pose.orientation.y = param[4];
    //     path_point.pose.orientation.z = param[5];
    //     path_point.pose.orientation.w = param[6];

    //     // Store in shared_queue
    //     shared_queue->push_back(path_point);
    // }
    // Set the output port
//     setOutput("path_point", shared_queue);

//     return BT::NodeStatus::SUCCESS;
// }

BT::PortsList initPoints::providedPorts() {
    return {
        BT::OutputPort<BT::SharedQueue<geometry_msgs::msg::PoseStamped>>("garbage_points"),
        BT::OutputPort<BT::SharedQueue<geometry_msgs::msg::PoseStamped>>("material_points")
    };
}

BT::NodeStatus initPoints::tick() {
    auto garbage_shared_queue = std::make_shared<std::deque<geometry_msgs::msg::PoseStamped>>();
    auto material_shared_queue = std::make_shared<std::deque<geometry_msgs::msg::PoseStamped>>();
    std::vector<std::string> points_name_ = {"0", "1", "2", "3", "4", "5", "6", "7", "8", "9"};
    node_->declare_parameter<std::vector<double>>(points_name_[0], {2.1, 1.7, 0.0, 0.0, 0.0, 0.707, 0.707});
    node_->declare_parameter<std::vector<double>>(points_name_[1], {2.7, 1.3, 0.0, 0.0, 0.0, 0.0, 1.0});
    node_->declare_parameter<std::vector<double>>(points_name_[2], {2.7, 0.4, 0.0, 0.0, 0.0, 0.0, 1.0});
    node_->declare_parameter<std::vector<double>>(points_name_[3], {1.9, 0.8, 0.0, 0.0, 0.0, 0.707, 0.707});
    node_->declare_parameter<std::vector<double>>(points_name_[4], {2.2, 0.4, 0.0, 0.0, 0.0, -0.707, 0.707});
    node_->declare_parameter<std::vector<double>>(points_name_[5], {1.1, 0.8, 0.0, 0.0, 0.0, 0.707, 0.707});
    node_->declare_parameter<std::vector<double>>(points_name_[6], {0.8, 0.4, 0.0, 0.0, 0.0, -0.707, 0.707});
    node_->declare_parameter<std::vector<double>>(points_name_[7], {0.3, 1.3, 0.0, 0.0, 0.0, 1.0, 0.0});
    node_->declare_parameter<std::vector<double>>(points_name_[8], {0.3, 0.4, 0.0, 0.0, 0.0, 1.0, 0.0});
    node_->declare_parameter<std::vector<double>>(points_name_[9], {0.8, 1.7, 0.0, 0.0, 0.0, 0.707, 0.707});
    for (const auto &point_name_ : points_name_) {
        std::vector<double> param;
        node_->get_parameter(point_name_, param);
        geometry_msgs::msg::PoseStamped path_point;
        path_point.pose.position.x = param[0];
        path_point.pose.position.y = param[1];
        path_point.pose.position.z = param[2];
        path_point.pose.orientation.x = param[3];
        path_point.pose.orientation.y = param[4];
        path_point.pose.orientation.z = param[5];
        path_point.pose.orientation.w = param[6];
        // Store in shared_queue
        material_shared_queue->push_back(path_point);
    }
    setOutput("garbage_points", garbage_shared_queue);
    setOutput("material_points", material_shared_queue);

    RCLCPP_INFO(node_->get_logger(), "Init points");

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList StateUpdater::providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("changed_point_position"),
        BT::InputPort<int>("changed_point_code"),
        BT::InputPort<int>("point_status"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("new_garbage_position"),
        BT::InputPort<BT::SharedQueue<geometry_msgs::msg::PoseStamped>>("materials_in"),
        BT::OutputPort<BT::SharedQueue<geometry_msgs::msg::PoseStamped>>("materials_out"),
        BT::InputPort<BT::SharedQueue<geometry_msgs::msg::PoseStamped>>("garbages_in"),
        BT::OutputPort<BT::SharedQueue<geometry_msgs::msg::PoseStamped>>("garbages_out")
    };
}

BT::NodeStatus StateUpdater::onStart() {
    RCLCPP_INFO(node_->get_logger(), "Start update state");
    geometry_msgs::msg::PoseStamped position = getInput<geometry_msgs::msg::PoseStamped>("changed_point_position").value();
    int pt_num_ = getInput<int>("changed_point_code").value();
    int status_ = getInput<int>("point_status").value();
    materials_info_deque = *getInput<BT::SharedQueue<geometry_msgs::msg::PoseStamped>>("materials_in").value();
    garbage_points_deque = *getInput<BT::SharedQueue<geometry_msgs::msg::PoseStamped>>("garbages_in").value();

    materials_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>("/robot/objects/materials_info", 10);
    garbages_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>("/robot/objects/obstacles_info", 10);

    double rad = position.pose.position.z * PI / 180;
    tf2::Quaternion q;
    q.setRPY(0, 0, rad);
    materials_info_deque[pt_num_] = position;
    materials_info_deque[pt_num_].pose.orientation.x = q.x();
    materials_info_deque[pt_num_].pose.orientation.y = q.y();
    materials_info_deque[pt_num_].pose.orientation.z = q.z();
    materials_info_deque[pt_num_].pose.orientation.w = q.w();
    materials_info_deque[pt_num_].pose.position.z = (double)status_;

    if (getInput<geometry_msgs::msg::PoseStamped>("new_garbage_position").value().pose.position.z != -1) {
        geometry_msgs::msg::PoseStamped garbage = getInput<geometry_msgs::msg::PoseStamped>("new_garbage_position").value();
        garbage_points_deque.push_back(garbage);
    }
    for (const auto &param : materials_info_deque) 
        materials_info_.poses.push_back(param.pose);
    for (const auto &param : garbage_points_deque)
        garbage_points_.poses.push_back(param.pose);
    return NodeStatus::RUNNING;
}

BT::NodeStatus StateUpdater::onRunning() {
    pub_count++;
    materials_pub_->publish(materials_info_);
    garbages_pub_->publish(garbage_points_);
    if (pub_count == 100) {
        std::shared_ptr<std::deque<geometry_msgs::msg::PoseStamped>> shared_queue = std::make_shared<std::deque<geometry_msgs::msg::PoseStamped>>(materials_info_deque);
        setOutput("materials_out", shared_queue);
        shared_queue = std::make_shared<std::deque<geometry_msgs::msg::PoseStamped>>(garbage_points_deque);
        setOutput("garbages_out", shared_queue);

        RCLCPP_INFO(node_->get_logger(), "Finish update state");
        return BT::NodeStatus::SUCCESS;
    }
    return NodeStatus::RUNNING;
}

void StateUpdater::onHalted() {
    // ROS_INFO_STREAM("Node halted");
    return;
}