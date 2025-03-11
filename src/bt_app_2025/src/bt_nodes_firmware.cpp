#include "bt_app_2025/bt_nodes_firmware.h"

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

/********************************/
/* Simple Node to activate SIMA */
/********************************/
BT::NodeStatus SIMAactivate::tick() {
    timer_ = node_->create_wall_timer(1000ms, std::bind(&SIMAactivate::wakeUpSIMA, this));
    return NodeStatus::SUCCESS;
}
bool SIMAactivate::wakeUpSIMA() {
    if (current_time_ < 85)
        return false;

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client = node_->create_client<std_srvs::srv::SetBool>("/robot/objects/sima_activate");
    // setup request service
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true; // active SIMA

    while (!client->wait_for_service(1s))
        RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
    
    // keep sending request until mission success
    do {
        // send request to the server
        auto result = client->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
            if (result.get()->success) {
                RCLCPP_INFO_STREAM(node_->get_logger(), "SIMA active!"); 
                mission_finished_ = result.get()->success;
            }
            else
                RCLCPP_INFO_STREAM(node_->get_logger(), "SIMA error msg: " << result.get()->message);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to call SIMA");
        }
    } while (!mission_finished_);

    // close the timer function if mission success
    if (mission_finished_)
        timer_.reset();
    return true;
}
/*************/
/* Finisher */
/************/
PortsList Finisher::providedPorts() {
    return { 
        BT::InputPort<std::deque<int>>("step_results"),
        BT::InputPort<bool>("robot_type"),
        //construct:
        BT::OutputPort<int>("success_levels"), 
        BT::OutputPort<int>("failed_levels"), 
        BT::OutputPort<int>("on_robot_materials"),
        //collect:
        BT::OutputPort<bool>("has_raw_material"), 
        BT::OutputPort<bool>("has_one_level")
    };
}

BT::NodeStatus Finisher::onStart()
{
    getInput<std::deque<int>>("step_results", step_results_);
    getInput<bool>("robot_type", robot_type_);
    blackboard_->get<int>("mission_progress", mission_progress_);
    blackboard_->set<int>("mission_progress", 0);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Finisher::onRunning()
{
    switch(mission_progress_){

        case 1:
            break;
        case 2:
            break;
        case 3:
            break;
        case 4:
            if(robot_type_)
            {

            }
            else
            {
              
            }
            break;
        case 5:
            if(robot_type_)
            {

            }
            else
            {
                
            }
            break;
        default:
            break;
    }

    setOutput("success_levels", success_levels_); 
    setOutput("failed_levels", failed_levels_); 
    setOutput("on_robot_materials", on_robot_materials_);
    setOutput("has_raw_material", has_raw_material_); 
    setOutput("has_one_level", has_one_level_);
    
    return BT::NodeStatus::SUCCESS;
}

void Finisher::onHalted()
{
    // Reset the output port
    setOutput("success_levels", 0); 
    setOutput("failed_levels", 0); 
    setOutput("has_raw_material", false); 
    setOutput("has_one_level", false);
    setOutput("on_robot_materials", 0);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Testing Node halted");
    return;
}

/****************/
/* Topic Mission*/  
/****************/
PortsList FirmwareMission::providedPorts() {
    return { 
        BT::InputPort<int>("mission_type"),
        BT::OutputPort<int>("mission_status")
    };
}

BT::NodeStatus FirmwareMission::mission_callback(const std_msgs::msg::Int32::SharedPtr sub_msg) {
    mission_status_ = sub_msg->data;
    RCLCPP_INFO(node_->get_logger(), "I heard: '%d'", mission_status_);
    if (mission_status_ == 1) {
        RCLCPP_INFO(node_->get_logger(), "Mission success");
        blackboard_->set<int>("mission_progress", ++mission_progress_);
        setOutput<int>("mission_status", mission_status_);
        return BT::NodeStatus::SUCCESS;
    } else if (mission_status_ == 0) {
        RCLCPP_INFO(node_->get_logger(), "Mission running");
        return BT::NodeStatus::RUNNING;
    } else if (mission_status_ == 2) {
        RCLCPP_INFO(node_->get_logger(), "Mission received");
        mission_received_ = true;
        return BT::NodeStatus::RUNNING;
    } else {
        RCLCPP_INFO(node_->get_logger(), "Mission failed");
        setOutput<int>("mission_status", mission_status_);
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus FirmwareMission::onStart() {
    RCLCPP_INFO(node_->get_logger(), "Node start");
    getInput<int>("mission_type", mission_type_);
    blackboard_->get<int>("mission_progress", mission_progress_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FirmwareMission::onRunning() {
    RCLCPP_INFO(node_->get_logger(), "Testing Node running");
    pub_msg.data = mission_type_;
    if (!mission_received_)
        publisher_->publish(pub_msg);
    return BT::NodeStatus::RUNNING;
}

void FirmwareMission::onHalted() {
    RCLCPP_INFO(node_->get_logger(), "Testing Node halted");
    setOutput<int>("mission_status", -1);
    return;
}

/***************************/
/* Integrated Mission Node */
/***************************/
PortsList IntegratedMissionNode::providedPorts() {
    return { 
        BT::InputPort<std::string>("mission_set_name"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("base"),
        BT::OutputPort<int>("mission_status"),
        BT::OutputPort<int>("mission_progress")
    };
}
inline void IntegratedMissionNode::setOutputs() {
    setOutput<int>("mission_status", mission_status_);
    setOutput<int>("mission_progress", mission_progress_);
}

BT::NodeStatus IntegratedMissionNode::mission_callback(const std_msgs::msg::Int32::SharedPtr sub_msg) {
    mission_status_ = sub_msg->data;
    RCLCPP_INFO(node_->get_logger(), "I heard: '%d'", mission_status_);
    if (mission_status_ == MissionState::FINISH) {
        RCLCPP_INFO(node_->get_logger(), "Mission success");
        mission_progress_++;
        mission_finished_ = true;
    } else if (mission_status_ == MissionState::RUNNING1 || mission_status_ == MissionState::RUNNING2 || mission_status_ == MissionState::RUNNING3) {
        RCLCPP_INFO(node_->get_logger(), "Mission running");
    } else if (mission_status_ == MissionState::RECEIVED) {
        RCLCPP_INFO(node_->get_logger(), "Mission received");
        mission_received_ = true;
    } else {
        RCLCPP_INFO(node_->get_logger(), "Mission failed");
        mission_finished_ = true;
        setOutputs();
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus IntegratedMissionNode::onStart() {
    RCLCPP_INFO(node_->get_logger(), "Node start");
    getInput<std::string>("mission_set_name", mission_set_name_);
    getInput<geometry_msgs::msg::PoseStamped>("base", base_);
    node_->declare_parameter<std::vector<int64_t>>(mission_set_name_, {});
    node_->get_parameter(mission_set_name_, mission_queue_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus IntegratedMissionNode::onRunning() {
    RCLCPP_INFO(node_->get_logger(), "Testing Node running");

    while (!mission_queue_.empty()) {
        if (mission_finished_) {
            mission_type_ = (int)mission_queue_.back();
            mission_queue_.pop_back();
        }
        mission_finished_ = false;
        if (mission_type_ == 1) {
            // navigation
            double offset = base_.pose.position.z;
            auto nav_client = std::make_shared<NavigateClient>(base_.pose, offset);
            while (!nav_client->is_nav_finished()) {
              rclcpp::spin_some(nav_client);
            }
        } else if (!mission_received_) {
            pub_msg.data = mission_type_;
            publisher_->publish(pub_msg);
        }
    }
    setOutputs();
    if (mission_finished_)
        return BT::NodeStatus::SUCCESS;
    else
        return BT::NodeStatus::RUNNING;
}

bool IntegratedMissionNode::UpdateRobotPose() {
    geometry_msgs::msg::TransformStamped transformStamped;

    try {
        transformStamped = tf_buffer_.lookupTransform(
            "map", 
            "base_link",
            rclcpp::Time()
        );
        robot_pose_.pose.position.x = transformStamped.transform.translation.x;
        robot_pose_.pose.position.y = transformStamped.transform.translation.y;
        robot_pose_.pose.orientation = transformStamped.transform.rotation;

        return true;
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("docking"), "[Kernel::UpdateRobotPose]: line " << __LINE__ << " " << ex.what());

        return false;
    }
}

void IntegratedMissionNode::onHalted() {
    RCLCPP_INFO(node_->get_logger(), "Testing Node halted");
    setOutputs();
    return;
}

/******************/
/* ROS Navigation */
/******************/
double inline NavigateClient::calculateDistance(const geometry_msgs::msg::Pose &pose1, const geometry_msgs::msg::Pose &pose2) {
    tf2::Vector3 position1(pose1.position.x, pose1.position.y, 0);
    tf2::Vector3 position2(pose2.position.x, pose2.position.y, 0);
    double dist = position1.distance(position2);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "distance: " << dist);
    return dist;
}

double inline NavigateClient::calculateAngleDifference(const geometry_msgs::msg::Pose &pose1, const geometry_msgs::msg::Pose &pose2)
{
    tf2::Quaternion orientation1, orientation2;
    tf2::fromMsg(pose1.orientation, orientation1);
    tf2::fromMsg(pose2.orientation, orientation2);
    double yaw1 = tf2::impl::getYaw(orientation1);
    double yaw2 = tf2::impl::getYaw(orientation2);
    return std::fabs(yaw1 - yaw2);
}

bool NavigateClient::is_nav_finished() const
{
    return this->nav_finished_;
}
bool NavigateClient::is_nav_success() const
{
    return this->nav_error_;
}

void NavigateClient::send_goal()
{
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_) {
        RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
        nav_finished_ = true;
        nav_error_ = true;
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        nav_finished_ = true;
        nav_error_ = true;
        return;
    }

    rclcpp::Time now = this->now();
    goal_.header.stamp = now;
    goal_.header.frame_id = "map";
    if ((int)offset_ == 0) {
        goal_pose_.position.x += offset_ - (int)offset_;
    } else if ((int)offset_ == 1 || (int)offset_ == -1) {
        goal_pose_.position.y += offset_ - (int)offset_;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid offset value");
        nav_finished_ = true;
        nav_error_ = true;
        return;
    }
    goal_.pose = goal_pose_;

    RCLCPP_INFO(this->get_logger(), "Start Nav (%f, %f)", goal_.pose.position.x, goal_.pose.position.y);
    nav_finished_ = false;

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goal_;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&NavigateClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&NavigateClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&NavigateClient::result_callback, this, _1);
    auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void NavigateClient::goal_response_callback(GoalHandleNavigation::SharedPtr goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void NavigateClient::feedback_callback(
    GoalHandleNavigation::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    current_pose_ = feedback->current_pose;
}

void NavigateClient::result_callback(const GoalHandleNavigation::WrappedResult & result)
{
    nav_finished_ = true;
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Result received");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
    }
    // check if mission success
    if (calculateDistance(current_pose_.pose, goal_.pose) < 0.03 && calculateAngleDifference(current_pose_.pose, goal_.pose) < 0.4) {
        nav_error_ = false;
        RCLCPP_INFO_STREAM(this->get_logger(), "success! final_pose: " << current_pose_.pose.position.x << ", " << current_pose_.pose.position.y);
        return;
    } else {
        nav_error_ = true;
        RCLCPP_INFO_STREAM(this->get_logger(), "fail! final_pose: " << current_pose_.pose.position.x << ", " << current_pose_.pose.position.y);
        return;
    }
}