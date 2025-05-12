#include "bt_app_2025/bt_nodes_navigation.h"
#include "bt_app_2025/bt_nodes_receiver.h"
// #include "bt_app_2025/bt_nodes_util.h"

using namespace BT;
// using namespace MATH;
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
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "distance: " << dist);
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
        BT::InputPort<geometry_msgs::msg::PoseStamped>("base"),
        BT::InputPort<double>("offset"),
        BT::InputPort<double>("shift"),
        BT::InputPort<std::string>("dock_type"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("final_pose")
    };
}

bool Navigation::setGoal(RosActionNode::Goal& goal) {
    auto m_0 = getInput<geometry_msgs::msg::PoseStamped>("goal");
    auto m_1 = getInput<geometry_msgs::msg::PoseStamped>("base");
    auto m = (m_0) ? m_0 : m_1;
    auto o = getInput<double>("offset");
    auto s = getInput<double>("shift");
    if (o) offset_ = o.value();
    if (s) shift_ = s.value();
    getInput<std::string>("dock_type", dock_type_);

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
    goal_.pose.position.z = 0;

    if (!m_0) {
        if (dock_type_ == "mission_dock_x" || dock_type_.substr(0, 6) == "dock_x") {
            goal_.pose.position.x += offset_; // set staging point
            goal_.pose.position.y += shift_;
        } else if (dock_type_ == "mission_dock_y" || dock_type_.substr(0, 6) == "dock_y") {
            goal_.pose.position.x += shift_;
            goal_.pose.position.y += offset_; // set staging point
        } else {
            RCLCPP_ERROR(logger(), "Invalid offset value");
            return false;
        }
        goal_.pose.position.z = offset_;
    }
    goal.use_dock_id = false; // set use dock id
    goal.dock_pose = goal_; // send goal pose
    goal.dock_type = dock_type_;    // determine the docking direction (x or y)
    goal.max_staging_time = 1000.0; // set max staging time
    goal.navigate_to_staging_pose = 1;  // if it's pure docking, then don't need to navigate to staging pose

    RCLCPP_INFO(logger(), "Start Nav to (%f, %f)", goal.dock_pose.pose.position.x, goal.dock_pose.pose.position.y);
    return true;
}

NodeStatus Navigation::onFeedback(const std::shared_ptr<const Feedback> feedback) {
    // nav_recov_times_ = feedback->number_of_recoveries;
    nav_recov_times_ = feedback->num_retries;
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
    LocReceiver::UpdateRobotPose(robot_pose_, tf_buffer_, frame_id_);
    if (calculateDistance(robot_pose_.pose, goal_.pose) < nav_dist_error_ && calculateAngleDifference(robot_pose_.pose, goal_.pose) < nav_ang_error_) {
        RCLCPP_INFO_STREAM(logger(), "success! final_pose: " << robot_pose_.pose.position.x << ", " << robot_pose_.pose.position.y << ", " << ConvertPoseFormat(robot_pose_).pose.position.z);
        RCLCPP_INFO_STREAM(logger(), "-----------------");
        setOutput<geometry_msgs::msg::PoseStamped>("final_pose", ConvertPoseFormat(robot_pose_));
        return NodeStatus::SUCCESS;
    } else {
        nav_error_ = true;
        RCLCPP_INFO_STREAM(logger(), "fail! final_pose: " << robot_pose_.pose.position.x << ", " << robot_pose_.pose.position.y << ", " << ConvertPoseFormat(robot_pose_).pose.position.z);
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
    RCLCPP_INFO_STREAM(logger(), "RETURN FAILURE! final_pose: " << robot_pose_.pose.position.x << ", " << robot_pose_.pose.position.y << ", " << robot_pose_.pose.position.z);
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
    blackboard_->set<bool>("Timeout", false);

    rclcpp::Time now = this->now(); // get current time
    goal_.header.stamp = now; // set header time
    goal_.header.frame_id = "map";  // set header frame
    goal_.pose = m.value().pose; // calculate goal pose
    if (dock_type_ == "mission_dock_x" || dock_type_.substr(0, 6) == "dock_x") {
        goal_.pose.position.x += offset_; // set staging point
        goal_.pose.position.y += shift_;
    } else if (dock_type_ == "mission_dock_y" || dock_type_.substr(0, 6) == "dock_y") {
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

NodeStatus Docking::goalErrorDetect() {
    double nav_dist_error_ = node_->get_parameter("nav_dist_error").as_double();
    double nav_ang_error_ = node_->get_parameter("nav_ang_error").as_double();
    blackboard_->set<bool>("enable_vision_check", true);

    // check the correctness of the final pose
    LocReceiver::UpdateRobotPose(robot_pose_, tf_buffer_, frame_id_);
    if (calculateDistance(robot_pose_.pose, goal_.pose) < nav_dist_error_ && calculateAngleDifference(robot_pose_.pose, goal_.pose) < nav_ang_error_) {
        RCLCPP_INFO_STREAM(logger(), "success! final_pose: " << robot_pose_.pose.position.x << ", " << robot_pose_.pose.position.y << ", " << ConvertPoseFormat(robot_pose_).pose.position.z);
        RCLCPP_INFO_STREAM(logger(), "-----------------");
        setOutput<geometry_msgs::msg::PoseStamped>("final_pose", ConvertPoseFormat(robot_pose_));
        return NodeStatus::SUCCESS;
    } else {
        nav_error_ = true;
        RCLCPP_INFO_STREAM(logger(), "fail! final_pose: " << robot_pose_.pose.position.x << ", " << robot_pose_.pose.position.y << ", " << ConvertPoseFormat(robot_pose_).pose.position.z);
        RCLCPP_INFO_STREAM(logger(), "z" << ConvertPoseFormat(goal_).pose.position.z);
        RCLCPP_INFO_STREAM(logger(), "-----------------");
        setOutput<geometry_msgs::msg::PoseStamped>("final_pose", ConvertPoseFormat(goal_));
        return NodeStatus::SUCCESS;
    }
}

NodeStatus Docking::onResultReceived(const WrappedResult& wr) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "get dock result");
    nav_finished_ = true;
    switch (wr.result->success) {
        case true:
            break;
        case false:
            nav_error_ = true;
            nav_finished_ = true;
            if (wr.result->error_code == 905)
                blackboard_->set<bool>("Timeout", true);
            blackboard_->set<bool>("enable_vision_check", true);
            LocReceiver::UpdateRobotPose(robot_pose_, tf_buffer_, frame_id_);
            setOutput<geometry_msgs::msg::PoseStamped>("final_pose", ConvertPoseFormat(robot_pose_));
            RCLCPP_INFO_STREAM(logger(), "error code: " << wr.result->error_code << " RETURN FAILURE! final_pose: " << robot_pose_.pose.position.x << ", " << robot_pose_.pose.position.y << ", " << robot_pose_.pose.position.z);
            RCLCPP_INFO_STREAM(logger(), "-----------------");
            return NodeStatus::FAILURE;
        default:
            nav_error_ = true;
            nav_finished_ = true;
            if (wr.result->error_code == 905)
                blackboard_->set<bool>("Timeout", true);
            blackboard_->set<bool>("enable_vision_check", true);
            LocReceiver::UpdateRobotPose(robot_pose_, tf_buffer_, frame_id_);
            setOutput<geometry_msgs::msg::PoseStamped>("final_pose", ConvertPoseFormat(robot_pose_));
            RCLCPP_INFO_STREAM(logger(), "error code: " << wr.result->error_code << " RETURN FAILURE! final_pose: " << robot_pose_.pose.position.x << ", " << robot_pose_.pose.position.y << ", " << robot_pose_.pose.position.z);
            RCLCPP_INFO_STREAM(logger(), "-----------------");
            return NodeStatus::FAILURE;
    }
    return goalErrorDetect();
}

NodeStatus Docking::onFailure(ActionNodeErrorCode error) {
    nav_error_ = true;
    nav_finished_ = true;

    blackboard_->set<bool>("enable_vision_check", true);
    LocReceiver::UpdateRobotPose(robot_pose_, tf_buffer_, frame_id_);
    setOutput<geometry_msgs::msg::PoseStamped>("final_pose", ConvertPoseFormat(robot_pose_));
    RCLCPP_INFO_STREAM(logger(), "error code: " << error << "RETURN FAILURE! final_pose: " << robot_pose_.pose.position.x << ", " << robot_pose_.pose.position.y << ", " << robot_pose_.pose.position.z);
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
    goal_.pose = m.value().pose; // calculate goal pose
    goal_.pose.position.z = 0; // set offset distance as 0
    tf2::Quaternion q; // declare Quaternion
    q.setRPY(0, 0, rad); // change degree-z into Quaternion
    goal_.pose.orientation.x = q.x();
    goal_.pose.orientation.y = q.y();
    goal_.pose.orientation.z = q.z();
    goal_.pose.orientation.w = q.w();
    goal.use_dock_id = false; // set use dock id
    goal.dock_pose = goal_; // send goal pose
    goal.dock_type = "dock_x_slow_loose";    // determine the docking direction (x or y)
    goal.max_staging_time = 1000.0; // set max staging time
    goal.navigate_to_staging_pose = 1;  // if it's pure docking, then don't need to navigate to staging pose

    RCLCPP_INFO(logger(), "Start Rotating (%f, %f)", goal_.pose.orientation.w, goal_.pose.orientation.z);
    return true;
}

NodeStatus Rotation::goalErrorDetect() {
    double rotate_dist_error_ = node_->get_parameter("rotate_dist_error").as_double();
    double rotate_ang_error_ = node_->get_parameter("rotate_ang_error").as_double();

    LocReceiver::UpdateRobotPose(robot_pose_, tf_buffer_, frame_id_);
    // check the correctness of the final pose
    if (calculateDistance(robot_pose_.pose, goal_.pose) < rotate_dist_error_ && calculateAngleDifference(robot_pose_.pose, goal_.pose) < rotate_ang_error_) {
        nav_finished_ = true;
        RCLCPP_INFO(logger(), "success! final_direction: (%f, %f)", robot_pose_.pose.orientation.w, robot_pose_.pose.orientation.z);
        // RCLCPP_INFO_STREAM(logger(), "z" << ConvertPoseFormat(robot_pose_).pose.position.z);
        RCLCPP_INFO_STREAM(logger(), "-----------------");
        setOutput<geometry_msgs::msg::PoseStamped>("final_pose", ConvertPoseFormat(robot_pose_));
        return NodeStatus::SUCCESS;
    } else {
        nav_error_ = true;
        RCLCPP_INFO(logger(), "fail! final_direction: (%f, %f)", robot_pose_.pose.orientation.w, robot_pose_.pose.orientation.z);
        // RCLCPP_INFO_STREAM(logger(), "z" << ConvertPoseFormat(goal_).pose.position.z);
        RCLCPP_INFO_STREAM(logger(), "-----------------");
        setOutput<geometry_msgs::msg::PoseStamped>("final_pose", ConvertPoseFormat(goal_));
        return NodeStatus::SUCCESS;
    }
}

NodeStatus Rotation::onFeedback(const std::shared_ptr<const Feedback> feedback) {
    nav_recov_times_ = feedback->num_retries;
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
    LocReceiver::UpdateRobotPose(robot_pose_, tf_buffer_, frame_id_);
    RCLCPP_INFO(logger(), "RETURN FAILURE! final_direction: (%f, %f)", robot_pose_.pose.orientation.w, robot_pose_.pose.orientation.z);
    // RCLCPP_INFO_STREAM(logger(), "z" << ConvertPoseFormat(goal_).pose.position.z);
    RCLCPP_INFO_STREAM(logger(), "-----------------");
    setOutput<geometry_msgs::msg::PoseStamped>("final_pose", ConvertPoseFormat(goal_));
    return NodeStatus::SUCCESS;
}

PortsList StopRobot::providedPorts() {
    return {};
}

BT::NodeStatus StopRobot::tick() {
    stop_msg.data = true;
    for (int i = 0; i < 10; i++) {
        publisher_->publish(stop_msg);
        rate_.sleep();
    }
    stop_msg.data = false;
    for (int i = 0; i < 10; i++) {
        publisher_->publish(stop_msg);
        rate_.sleep();
    }
    return BT::NodeStatus::SUCCESS;
}

PortsList MaterialChecker::providedPorts() {
    return {
        BT::InputPort<int>("base_index"),           // if base index is -1, then plan new goal directly
        BT::InputPort<std::string>("dock_type"),
        BT::InputPort<std::string>("mission_type"), // front grabber or back grabber
        BT::InputPort<double>("offset"),
        BT::InputPort<double>("shift"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("remap_base"),
        BT::OutputPort<std::string>("remap_dock_type"),
        BT::OutputPort<double>("remap_offset"),
        BT::OutputPort<double>("remap_shift"),
        BT::OutputPort<int>("remap_index"), 
    };
}

int MaterialChecker::findBestTarget() {
    double robotVelocity_, rivalVelocity;
    geometry_msgs::msg::PoseStamped rivalGoal;
    geometry_msgs::msg::Pose targetMaterialPose_;
    double safestDeltaDist_ = 5;
    int dist_, deltaDist_, minDist_ = 5;
    int safestPointIndex_, minDistIndex_;
    std::string team_;
    blackboard_->get<std::string>("team", team_);        // get team color

    LocReceiver::UpdateRobotPose(robot_pose_, tf_buffer_, frame_id_);
    LocReceiver::UpdateRivalPose(rival_pose_, tf_buffer_, frame_id_);
    for (int i = 1; i < 9; i++) {                 // delete empty materials point
        if (materials_info_.data[i])
            candidate_.push_back(i);
    }
    if (team_ == "b" && materials_info_.data[0])       // if it's blue team, then detect if the first point is empty
        candidate_.push_back(0);
    else if (team_ == "y" && materials_info_.data[9]) // if it's yellow team, then detect if the last point is empty
        candidate_.push_back(9);
    if (candidate_.empty()) {
        RCLCPP_INFO_STREAM(node_->get_logger(), "No material point detected");
        return -1;
    }
    
    safestPointIndex_ = candidate_.front();
    minDistIndex_ = candidate_.front();
    do {                                         // delete materials point that is too close to rival
        targetMaterialPose_.position.x = material_points_[candidate_.front() * 5];
        targetMaterialPose_.position.y = material_points_[candidate_.front() * 5 + 1];
        deltaDist_ = calculateDistance(targetMaterialPose_, rival_pose_.pose) - calculateDistance(targetMaterialPose_, robot_pose_.pose);
        dist_ = calculateDistance(targetMaterialPose_, robot_pose_.pose);
        if (safestDeltaDist_ < deltaDist_) {     // iterate to find the safest material point
            safestDeltaDist_ = deltaDist_;
            safestPointIndex_ = candidate_.front();
        }
        if (deltaDist_ >= 0 || deltaDist_ == safestDeltaDist_) {         // iterate to find the closest material point
            if (minDist_ > dist_) {
                minDistIndex_ = candidate_.front();
                minDist_ = dist_;
            }
        }
        candidate_.pop_front();
    } while (!candidate_.empty());
    if (last_mission_failed_) {
        blackboard_->set<bool>("last_mission_failed", false);  
        return safestPointIndex_;
    }
    else
        return minDistIndex_;
}

NodeStatus MaterialChecker::tick() {
    // get input
    int baseIndex_ = getInput<double>("base_index").value();
    std::string dockType_ = getInput<std::string>("dock_type").value();
    std::string missionType_ = getInput<std::string>("mission_type").value();
    double offset_ = getInput<double>("offset").value();
    double shift_ = getInput<double>("shift").value();

    // get parameters
    std::string mapPointsFile_, bot_;
    blackboard_->get<std::string>("bot", bot_); 
    mapPointsFile_ = "map_points_" + bot_;
    node_->get_parameter(mapPointsFile_, material_points_);
    node_->get_parameter("safety_dist", safety_dist_);

    blackboard_->get<std_msgs::msg::Int32MultiArray>("materials_info", materials_info_);  // use vision message to check the target
    blackboard_->get<bool>("last_mission_failed", last_mission_failed_);                  // see if last mission failed
    bool enable_vision_check_;
    blackboard_->get<bool>("enable_vision_check", enable_vision_check_);
    // If the target is not ok
    // use vision message to find the best new target `i` (new base)
    LocReceiver::UpdateRivalPose(rival_pose_, tf_buffer_, frame_id_);
    base_.pose.position.x = material_points_[baseIndex_ * 5];
    base_.pose.position.y = material_points_[baseIndex_ * 5 + 1];
    if (materials_info_.data[baseIndex_] == 0 || baseIndex_ == -1 || calculateDistance(base_.pose, rival_pose_.pose) < safety_dist_) {
        baseIndex_ = findBestTarget();
        /**********************************************************/
        /* Notice!! this part need to be consider again carefully */
        /* Return fail directly is very dangerous!                */
        /**********************************************************/
        if (baseIndex_ == -1) { 
            return NodeStatus::FAILURE;
        }
    }
    if (!enable_vision_check_) {
        blackboard_->get<int>("current_index", baseIndex_); 
    } else {
        blackboard_->set<bool>("enable_vision_check", false);
        RCLCPP_INFO_STREAM(node_->get_logger(), "index: " << baseIndex_ << " offset: " << material_points_[baseIndex_ * 5 + 3] << " missionType_: " << missionType_);
    }
    base_.pose.position.x = material_points_[baseIndex_ * 5];
    base_.pose.position.y = material_points_[baseIndex_ * 5 + 1];
    base_.pose.position.z = material_points_[baseIndex_ * 5 + 2];
    offset_ = material_points_[baseIndex_ * 5 + 3];
    dockType_ = (int(material_points_[baseIndex_ * 5 + 2]) % 2) ? ("dock_y_" + dockType_) : ("dock_x_" + dockType_);
    shift_ = 0;

    // derive the position.z & offset & shift according to mission_type & map_points[i]
    int offset_dir_ = (int)(1 - 2 * int(base_.pose.position.z) % 2);
    int offset_positivity_ = (int)(offset_ / abs(offset_));
    if (missionType_ == "front") {
        if (bot_ == "1")
            offset_ -= offset_ / abs(offset_) * 0.04;
        shift_ = material_points_[baseIndex_ * 5 + 4];
    } else if (missionType_ == "back") {
        base_.pose.position.z = ((int)base_.pose.position.z / 2) ? base_.pose.position.z - 2 : base_.pose.position.z + 2;
    } else {
        throw "error mission direction for choosing nav goal!";
    }

    // set output port
    blackboard_->set<int>("current_index", baseIndex_); 
    setOutput<geometry_msgs::msg::PoseStamped>("remap_base", base_);
    setOutput<std::string>("remap_dock_type", dockType_);
    setOutput<double>("remap_offset", offset_);
    setOutput<double>("remap_shift", shift_);
    setOutput<int>("remap_index", baseIndex_);

    // Run the child node
    BT::NodeStatus child_status = child_node_->executeTick();
    return child_status;
}

PortsList MissionChecker::providedPorts() {
    return {
        BT::InputPort<int>("base_index"),
        BT::InputPort<std::string>("mission_type"), // front grabber or back grabber
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("remap_base"),
        BT::OutputPort<std::string>("level")
    };
}

NodeStatus MissionChecker::tick() {
    // get input
    int baseIndex_ = getInput<double>("base_index").value();
    std::string missionType_ = getInput<std::string>("mission_type").value();
    double dist = 0;
    double offset = 0;
    double shift_ = 0;

    // get parameters
    std::string map_points;
    blackboard_->get<std::string>("bot", map_points); 
    map_points = "map_points_" + map_points;
    node_->get_parameter(map_points, material_points_);
    node_->get_parameter("safety_dist", safety_dist_);
    blackboard_->get<std_msgs::msg::Int32MultiArray>("mission_points_status", mission_points_status_);
    blackboard_->get<int>("front_materials", front_materials_);
    blackboard_->get<int>("back_materials", back_materials_);
    
    // get base & offset from map_points[i]
    base_.pose.position.x = material_points_[baseIndex_ * 5];
    base_.pose.position.y = material_points_[baseIndex_ * 5 + 1];
    base_.pose.position.z = material_points_[baseIndex_ * 5 + 2];
    offset = material_points_[baseIndex_ * 5 + 3];
    shift_ = material_points_[baseIndex_ * 5 + 4];

    LocReceiver::UpdateRivalPose(rival_pose_, tf_buffer_, frame_id_);
    dist = calculateDistance(base_.pose, rival_pose_.pose);

    if (missionType_ == "back" && back_materials_ != 0) {
        base_.pose.position.z = ((int)base_.pose.position.z / 2) ? base_.pose.position.z - 2 : base_.pose.position.z + 2;
    }
    if (base_.pose.position.z == 1.0 || base_.pose.position.z == 3.0) {
        if (mission_points_status_.data[baseIndex_ - 11] != 0) {   // check if this mission is already placed
            base_.pose.position.y -= offset * 1.5;    // if yes, the placement point need to be changed
            // RCLCPP_INFO_STREAM(node_->get_logger(), "step back 5 cm, and then place the materials");
        }
        if (dist < safety_dist_ && abs(base_.pose.position.y - rival_pose_.pose.position.y) < safety_dist_ * 2/3) {
            base_.pose.position.x += (base_.pose.position.x - rival_pose_.pose.position.x)/abs(base_.pose.position.x - rival_pose_.pose.position.x)*(safety_dist_ - abs(base_.pose.position.x - rival_pose_.pose.position.x));
            RCLCPP_INFO_STREAM(node_->get_logger(), "rival near the bot when placing the materials");
        }
    } else {
        if (mission_points_status_.data[baseIndex_ - 11] != 0) {  // check if this mission is already placed
            base_.pose.position.x -= offset * 1.5;   // if yes, the placement point need to e changed
            // RCLCPP_INFO_STREAM(node_->get_logger(), "step back 5 cm, and then place the materials");
        }
        if (dist < safety_dist_ && abs(base_.pose.position.x - rival_pose_.pose.position.x) < safety_dist_ * 2/3) {
            base_.pose.position.y += (base_.pose.position.y - rival_pose_.pose.position.y)/abs(base_.pose.position.y - rival_pose_.pose.position.y)*(safety_dist_ - abs(base_.pose.position.y - rival_pose_.pose.position.y));
            RCLCPP_INFO_STREAM(node_->get_logger(), "rival near the bot when placing the materials");
        }
    }
    if (back_materials_ != 0 && front_materials_ == 2) {
        setOutput<std::string>("level", "3");
    } else if (back_materials_ == 0 && front_materials_ == 2) {
        setOutput<std::string>("level", "2");
    } else if (back_materials_ != 0 && front_materials_ != 2) {
        setOutput<std::string>("level", "1");
    } else {
        setOutput<std::string>("level", "1");
    }

    // set output port
    setOutput<geometry_msgs::msg::PoseStamped>("remap_base", base_);

    // Run the child node
    BT::NodeStatus child_status = child_node_->executeTick();
    return child_status;
}