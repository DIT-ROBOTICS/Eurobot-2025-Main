#include "bt_app_2025/bt_nodes_navigation.h"
#include "bt_app_2025/bt_nodes_receiver.h"
// #include "bt_app_2025/bt_nodes_util.h"

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

double inline calculateAngleDifference(const geometry_msgs::msg::Pose &pose1, const geometry_msgs::msg::Pose &pose2) {
    tf2::Quaternion orientation1, orientation2;
    tf2::fromMsg(pose1.orientation, orientation1);
    tf2::fromMsg(pose2.orientation, orientation2);
    double yaw1 = tf2::impl::getYaw(orientation1);
    double yaw2 = tf2::impl::getYaw(orientation2);
    return std::fabs(yaw1 - yaw2);
}

geometry_msgs::msg::PoseStamped inline ConvertPoseFormat(geometry_msgs::msg::PoseStamped pose_) {
    tf2::Quaternion quaternion;
    tf2::fromMsg(pose_.pose.orientation, quaternion);
    pose_.pose.position.z = tf2::impl::getYaw(quaternion) * 2 / PI;
    return pose_;
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
    q.setRPY(0, 0, m.value().pose.position.z * PI / 2);
    goal_.pose.orientation.x = q.x();
    goal_.pose.orientation.y = q.y();
    goal_.pose.orientation.z = q.z();
    goal_.pose.orientation.w = q.w();
    goal.pose = goal_;

    RCLCPP_INFO(logger(), "Start Nav to (%f, %f)", goal.pose.pose.position.x, goal.pose.pose.position.y);
    return true;
}

NodeStatus Navigation::onFeedback(const std::shared_ptr<const Feedback> feedback) {
    current_pose_ = feedback->current_pose;
    nav_recov_times_ = feedback->number_of_recoveries;
    if (nav_recov_times_ > 2) {
        // check the correctness of the final pose
        return goalErrorDetect();
    }
    // RCLCPP_INFO_STREAM(logger(), "current_pose: " << current_pose_.pose.position.x << ", " << current_pose_.pose.position.y);
    return NodeStatus::RUNNING;
}

NodeStatus Navigation::goalErrorDetect() {
    double nav_dist_error_ = node_->get_parameter("nav_dist_error").as_double();
    double nav_ang_error_ = node_->get_parameter("nav_ang_error").as_double();

    // check the correctness of the final pose
    if (calculateDistance(current_pose_.pose, goal_.pose) < nav_dist_error_ && calculateAngleDifference(current_pose_.pose, goal_.pose) < nav_ang_error_) {
        RCLCPP_INFO_STREAM(logger(), "success! final_pose: " << current_pose_.pose.position.x << ", " << current_pose_.pose.position.y << ", " << ConvertPoseFormat(current_pose_).pose.position.z);
        RCLCPP_INFO_STREAM(logger(), "-----------------");
        setOutput<geometry_msgs::msg::PoseStamped>("final_pose", ConvertPoseFormat(current_pose_));
        return NodeStatus::SUCCESS;
    } else {
        nav_error_ = true;
        RCLCPP_INFO_STREAM(logger(), "fail! final_pose: " << current_pose_.pose.position.x << ", " << current_pose_.pose.position.y << ", " << ConvertPoseFormat(current_pose_).pose.position.z);
        RCLCPP_INFO_STREAM(logger(), "z" << ConvertPoseFormat(goal_).pose.position.z);
        RCLCPP_INFO_STREAM(logger(), "-----------------");
        setOutput<geometry_msgs::msg::PoseStamped>("final_pose", ConvertPoseFormat(goal_));
        return NodeStatus::SUCCESS;
    }
}

NodeStatus Navigation::onResultReceived(const WrappedResult& wr) {
    nav_finished_ = true;
    switch (wr.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
        break;
    case rclcpp_action::ResultCode::ABORTED:
        nav_error_ = true;
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was aborted");
    case rclcpp_action::ResultCode::CANCELED:
        nav_error_ = true;
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was canceled");
        return NodeStatus::FAILURE;
    default:
        nav_error_ = true;
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown result code");
        return NodeStatus::FAILURE;
    }
    return goalErrorDetect();
}

NodeStatus Navigation::onFailure(ActionNodeErrorCode error) {
    nav_error_ = true;
    nav_finished_ = true;
    RCLCPP_INFO_STREAM(logger(), "RETURN FAILURE! final_pose: " << current_pose_.pose.position.x << ", " << current_pose_.pose.position.y << ", " << current_pose_.pose.position.z);
    RCLCPP_INFO_STREAM(logger(), "z" << ConvertPoseFormat(goal_).pose.position.z);
    RCLCPP_INFO_STREAM(logger(), "-----------------");
    setOutput<geometry_msgs::msg::PoseStamped>("final_pose", ConvertPoseFormat(goal_));
    return NodeStatus::FAILURE;
}

BT::PortsList Docking::providedPorts() {
    return { 
        BT::InputPort<geometry_msgs::msg::PoseStamped>("base"),
        BT::InputPort<double>("offset"),
        BT::InputPort<double>("shift"),
        BT::InputPort<std::string>("dock_type"),
        BT::InputPort<bool>("isPureDocking"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("final_pose")
    };
}

bool Docking::setGoal(RosActionNode::Goal& goal) {
    auto m = getInput<geometry_msgs::msg::PoseStamped>("base");
    getInput<double>("offset", offset_);
    getInput<double>("shift", shift_);
    getInput<bool>("isPureDocking", isPureDocking_);
    getInput<std::string>("dock_type", dock_type_);

    rclcpp::Time now = this->now(); // get current time
    goal_.header.stamp = now; // set header time
    goal_.header.frame_id = "map";  // set header frame
    goal_.pose = m.value().pose; // calculate goal pose
    if (dock_type_ == "mission_dock_x") {
        goal_.pose.position.x += offset_; // set staging point
        goal_.pose.position.y += shift_;
    } else if (dock_type_ == "mission_dock_y") {
        goal_.pose.position.x += shift_;
        goal_.pose.position.y += offset_; // set staging point
    } else {
        RCLCPP_ERROR(logger(), "Invalid offset value");
        return false;
    }
    goal_.pose.position.z = offset_; // set offset distance
    tf2::Quaternion q; // declare Quaternion
    q.setRPY(0, 0, m.value().pose.position.z * PI / 2); // change degree-z into Quaternion
    goal_.pose.orientation.x = q.x();
    goal_.pose.orientation.y = q.y();
    goal_.pose.orientation.z = q.z();
    goal_.pose.orientation.w = q.w();
    goal.use_dock_id = false; // set use dock id
    goal.dock_pose = goal_; // send goal pose
    goal.dock_type = dock_type_;    // determine the docking direction (x or y)
    goal.max_staging_time = 1000.0; // set max staging time
    goal.navigate_to_staging_pose = !isPureDocking_;  // if it's pure docking, then don't need to navigate to staging pose

    RCLCPP_INFO(logger(), "Start Docking to (%f, %f)", goal.dock_pose.pose.position.x, goal.dock_pose.pose.position.y);
    return true;
}

NodeStatus Docking::onFeedback(const std::shared_ptr<const Feedback> feedback) {
    nav_recov_times_ = feedback->num_retries;
    if (nav_recov_times_ > 2) {
        LocReceiver::UpdateRobotPose(robot_pose_, tf_buffer_, frame_id_);
        RCLCPP_INFO_STREAM(logger(), "success! final_pose: " << robot_pose_.pose.position.x << ", " << robot_pose_.pose.position.y << ", " << ConvertPoseFormat(robot_pose_).pose.position.z);
        RCLCPP_INFO_STREAM(logger(), "-----------------");
        setOutput<geometry_msgs::msg::PoseStamped>("final_pose", ConvertPoseFormat(robot_pose_));
        return NodeStatus::SUCCESS;
    }
    return NodeStatus::RUNNING;
}

NodeStatus Docking::onResultReceived(const WrappedResult& wr) {
    nav_finished_ = true;
    switch (wr.result->success) {
        case true:
            break;
        case false:
            nav_error_ = true;
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal was aborted");
            return NodeStatus::FAILURE;
        default:
            nav_error_ = true;
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown result code");
            return NodeStatus::FAILURE;
    }
    LocReceiver::UpdateRobotPose(robot_pose_, tf_buffer_, frame_id_);
    RCLCPP_INFO_STREAM(logger(), "success! final_pose: " << robot_pose_.pose.position.x << ", " << robot_pose_.pose.position.y << ", " << ConvertPoseFormat(robot_pose_).pose.position.z);
    RCLCPP_INFO_STREAM(logger(), "-----------------");
    setOutput<geometry_msgs::msg::PoseStamped>("final_pose", ConvertPoseFormat(robot_pose_));
    return NodeStatus::SUCCESS;
}

NodeStatus Docking::onFailure(ActionNodeErrorCode error) {
    nav_error_ = true;
    nav_finished_ = true;
    LocReceiver::UpdateRobotPose(robot_pose_, tf_buffer_, frame_id_);
    setOutput<geometry_msgs::msg::PoseStamped>("final_pose", ConvertPoseFormat(robot_pose_));
    RCLCPP_INFO_STREAM(logger(), "RETURN FAILURE! final_pose: " << robot_pose_.pose.position.x << ", " << robot_pose_.pose.position.y << ", " << robot_pose_.pose.position.z);
    RCLCPP_INFO_STREAM(logger(), "-----------------");
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
    // double rad_base = acos(m.value().pose.orientation.w) * 2;
    double rad_base = m.value().pose.position.z * PI / 2;
    rad += rad_base;

    rclcpp::Time now = this->now();
    goal_.header.stamp = now;
    goal_.header.frame_id = "map";

    goal_.pose.position.x = m.value().pose.position.x;
    goal_.pose.position.y = m.value().pose.position.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, rad);
    goal_.pose.orientation.w = q.w();
    goal_.pose.orientation.x = q.x();
    goal_.pose.orientation.y = q.y();
    goal_.pose.orientation.z = q.z();
    goal.pose = goal_;

    RCLCPP_INFO(logger(), "Start Rotating (%f, %f)", goal_.pose.orientation.w, goal_.pose.orientation.z);
    return true;
}

NodeStatus Rotation::goalErrorDetect() {
    double rotate_dist_error_ = node_->get_parameter("rotate_dist_error").as_double();
    double rotate_ang_error_ = node_->get_parameter("rotate_ang_error").as_double();

    // check the correctness of the final pose
    if (calculateDistance(current_pose_.pose, goal_.pose) < rotate_dist_error_ && calculateAngleDifference(current_pose_.pose, goal_.pose) < rotate_ang_error_) {
        nav_finished_ = true;
        RCLCPP_INFO(logger(), "success! final_direction: (%f, %f)", current_pose_.pose.orientation.w, current_pose_.pose.orientation.z);
        RCLCPP_INFO_STREAM(logger(), "z" << ConvertPoseFormat(current_pose_).pose.position.z);
        RCLCPP_INFO_STREAM(logger(), "-----------------");
        setOutput<geometry_msgs::msg::PoseStamped>("final_pose", ConvertPoseFormat(current_pose_));
        return NodeStatus::SUCCESS;
    } else {
        nav_error_ = true;
        RCLCPP_INFO(logger(), "fail! final_direction: (%f, %f)", current_pose_.pose.orientation.w, current_pose_.pose.orientation.z);
        RCLCPP_INFO_STREAM(logger(), "z" << ConvertPoseFormat(goal_).pose.position.z);
        RCLCPP_INFO_STREAM(logger(), "-----------------");
        setOutput<geometry_msgs::msg::PoseStamped>("final_pose", ConvertPoseFormat(goal_));
        return NodeStatus::SUCCESS;
    }
}

NodeStatus Rotation::onFeedback(const std::shared_ptr<const Feedback> feedback) {
    current_pose_ = feedback->current_pose;
    nav_recov_times_ = feedback->number_of_recoveries;
    if (nav_recov_times_ > 2) {
        return goalErrorDetect();       // check the correctness of the final pose
    }
    return NodeStatus::RUNNING;
}

NodeStatus Rotation::onResultReceived(const WrappedResult& wr) {
    nav_finished_ = true;
    // check the correctness of the final pose
    return goalErrorDetect();
}

NodeStatus Rotation::onFailure(ActionNodeErrorCode error) {
    nav_error_ = true;
    nav_finished_ = true;
    RCLCPP_INFO(logger(), "RETURN FAILURE! final_direction: (%f, %f)", current_pose_.pose.orientation.w, current_pose_.pose.orientation.z);
    RCLCPP_INFO_STREAM(logger(), "z" << ConvertPoseFormat(goal_).pose.position.z);
    RCLCPP_INFO_STREAM(logger(), "-----------------");
    setOutput<geometry_msgs::msg::PoseStamped>("final_pose", ConvertPoseFormat(goal_));
    return NodeStatus::SUCCESS;
}

PortsList StopRobot::providedPorts() {
    return {};
}

BT::NodeStatus StopRobot::tick() {
    stop_msg.data = true;
    for (int i = 0; i < 10; i++)
        publisher_->publish(stop_msg);
    stop_msg.data = false;
    for (int i = 0; i < 10; i++)
        publisher_->publish(stop_msg);
    return BT::NodeStatus::SUCCESS;
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

PortsList VisionCheck::providedPorts() {
    return {
        BT::InputPort<int>("base_index"),           // if base index is -1, then plan new goal directly
        BT::InputPort<std::string>("dock_type"),
        BT::InputPort<std::string>("mission_type"), // front or back
        BT::InputPort<double>("offset"),
        BT::InputPort<double>("shift"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("remap_base"),
        BT::OutputPort<std::string>("remap_dock_type"),
        BT::OutputPort<double>("remap_offset"),
        BT::OutputPort<double>("remap_shift")
    };
}

int VisionCheck::findBestTarget() {
    double robotVelocity_, rivalVelocity;
    double robotMaterialDist, rivalMaterialDist;
    geometry_msgs::msg::PoseStamped rivalGoal;

    LocReceiver::UpdateRobotPose(robot_pose_, tf_buffer_, frame_id_);
    LocReceiver::UpdateRivalPose(rival_pose_, tf_buffer_, frame_id_);
    for (int i = 0; i < 10; i++)
        if (materials_info_.poses[i].position.z != -1)
            canditate_.push_back(i);
    for (int i = 0; i < canditate_.size(); i++) {
        if (calculateDistance(materials_info_.poses[canditate_.front()], rival_pose_.pose) >= calculateDistance(materials_info_.poses[canditate_.front()], robot_pose_.pose))
            canditate_.push_back(canditate_.front());
        canditate_.pop_front();
    }
    int min_index = canditate_.front();
    while (!canditate_.empty()) {
        canditate_.pop_front();
        if (min_index > canditate_.front())
            min_index = canditate_.front();
    }
    return min_index;
}

NodeStatus VisionCheck::tick() {
    // get input
    int baseIndex_ = getInput<double>("base_index").value();
    std::string dockType_ = getInput<std::string>("dock_type").value();
    std::string missionType_ = getInput<std::string>("mission_type").value();
    double offset_ = getInput<double>("offset").value();
    double shift_ = getInput<double>("shift").value();
    std::vector<double> materialPoints_;
    std::vector<double> missionPoints_;
    // get parameters
    node_->get_parameter("material_points", materialPoints_);
    node_->get_parameter("mission_points", missionPoints_);

    // use vision message to check the target
    blackboard_->get<geometry_msgs::msg::PoseArray>("materials_info", materials_info_);
    
    // If the target is not ok
    // use vision message to find the best new target `i` (new base)
    if (materials_info_.poses[baseIndex_].position.z == -1 || baseIndex_ == -1) {
        baseIndex_ = findBestTarget();
    }
    // get base & offset from map_points[i]
    base_.pose.position.x = materialPoints_[baseIndex_ * 4];
    base_.pose.position.y = materialPoints_[baseIndex_ * 4 + 1];
    base_.pose.position.z = materialPoints_[baseIndex_ * 4 + 2];
    offset_ = materialPoints_[baseIndex_ * 4 + 3];

    // derive the position.z & offset & shift according to mission_type & map_points[i]
    if (missionType_ == "front") {
        int dockTypeCode_;
        if (dockType_ == "mission_dock_y") // dock type code: 1 for y, -1 for x
            dockTypeCode_ = 1;
        else
            dockTypeCode_ = -1;
        shift_ *= offset_ / abs(offset_) * dockTypeCode_; // use dock type to determine the shift direction
    } else if (missionType_ == "back") {
        base_.pose.position.z = ((int)base_.pose.position.z / 2) ? base_.pose.position.z - 2 : base_.pose.position.z + 2;
        offset_ *= -1;
        shift_ = 0;
    } else {
        throw "error mission direction for choosing nav goal!";
    }

    // set output port
    setOutput<geometry_msgs::msg::PoseStamped>("remap_base", base_);
    setOutput<std::string>("remap_dock_type", dockType_);
    setOutput<double>("remap_offset", offset_);
    setOutput<double>("remap_shift", shift_);

    // Run the child node
    BT::NodeStatus child_status = child_node_->executeTick();
    return child_status;
}