#include "bt_app_2025/bt_nodes_firmware.h"
#include "bt_app_2025/bt_nodes_receiver.h"

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

template <> inline std::deque<double> BT::convertFromString(StringView str) {

    auto parts = splitString(str, ',');
    std::deque<double> output;

    for (int i = 0; i < (int)parts.size(); i++) {
        output.push_back(convertFromString<double>(parts[i]));
    }

    return output;
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

/*****************/
/* Mission Start */
/*****************/
PortsList MissionStart::providedPorts() {
    return { 
        BT::InputPort<std::deque<double>>("BACK_IN"),
        BT::InputPort<std::deque<double>>("FORWARD_IN"),
        BT::InputPort<double>("SHIFT_IN"),
        BT::InputPort<int>("INDEX_IN"),
        BT::OutputPort<double>("BACK_L"),
        BT::OutputPort<double>("BACK_M"),
        BT::OutputPort<double>("BACK_S"),
        BT::OutputPort<double>("FORWARD_L"),
        BT::OutputPort<double>("FORWARD_M"),
        BT::OutputPort<double>("FORWARD_S"),
        BT::OutputPort<double>("SHIFT"),
        BT::OutputPort<std::string>("DOCK_DIR"),
    };
}

BT::NodeStatus MissionStart::tick() {
    auto back_ = getInput<std::deque<double>>("BACK_IN");
    auto forward_ = getInput<std::deque<double>>("FORWARD_IN");
    double shift_ = getInput<double>("SHIFT_IN").value();
    int index_ = getInput<int>("INDEX_IN").value();
 
    int offset_dir_;
    int offset_positivity_;
    std::string mapPointsFile_, bot_;
    std_msgs::msg::Int32MultiArray materials_info_;
    if (!blackboard_->get<std::string>("bot", bot_) ||
        !blackboard_->get<std_msgs::msg::Int32MultiArray>("materials_info", materials_info_)) {
        throw std::runtime_error("blackboard variable not found!");
    }
    mapPointsFile_ = "map_points_" + bot_;
    node_->get_parameter(mapPointsFile_, material_points_);
    RCLCPP_INFO_STREAM(node_->get_logger(), materials_info_.data[0] << materials_info_.data[1] << materials_info_.data[2] << materials_info_.data[3] << materials_info_.data[4] << materials_info_.data[5] << materials_info_.data[6] << materials_info_.data[7] << materials_info_.data[8] << materials_info_.data[9]);

    offset_dir_ = (int)(1 - 2 * ((int)(material_points_[index_ * 7 + 2]) % 2));
    if (material_points_[index_ * 7 + 3] != 0)
        offset_positivity_ = (int)(material_points_[index_ * 7 + 3] / abs(material_points_[index_ * 7 + 3]));
    else
        offset_positivity_ = 0;
    forward_.value()[0] = material_points_[index_ * 7 + 6];
    RCLCPP_INFO_STREAM(node_->get_logger(), offset_positivity_);
    shift_ *= offset_dir_ * offset_positivity_;

    if (offset_dir_ == 1)
        setOutput("DOCK_DIR", "dock_x_gentle_precise_linearBoost");
    else if (offset_dir_ == -1)
        setOutput("DOCK_DIR", "dock_y_gentle_precise_linearBoost");
    
    if (offset_positivity_ > 0) {
        back_.value()[0] *= -1;
        back_.value()[1] *= -1;
        back_.value()[2] *= -1;
    }
    else if (offset_positivity_ < 0) {
        forward_.value()[0] *= -1;
        forward_.value()[1] *= -1;
        forward_.value()[2] *= -1;
    }
    else if (material_points_[index_ * 7 + 2] == 0 || material_points_[index_ * 7 + 2] == 1){
        back_.value()[0] *= -1;
        back_.value()[1] *= -1;
        back_.value()[2] *= -1;
    }
    else {
        forward_.value()[0] *= -1;
        forward_.value()[1] *= -1;
        forward_.value()[2] *= -1;  
    }
    RCLCPP_INFO_STREAM(node_->get_logger(), back_.value()[0] << ", " << back_.value()[1] << ", " << back_.value()[2]);
    RCLCPP_INFO_STREAM(node_->get_logger(), forward_.value()[0] << ", " << forward_.value()[1] << ", " << forward_.value()[2]);

    setOutput("BACK_L", back_.value()[0]);
    setOutput("BACK_M", back_.value()[1]);
    setOutput("BACK_S", back_.value()[2]);
    setOutput("FORWARD_L", forward_.value()[0]);
    setOutput("FORWARD_M", forward_.value()[1]);
    setOutput("FORWARD_S", forward_.value()[2]);
    setOutput("SHIFT", shift_);

    return BT::NodeStatus::SUCCESS;
}

/******************/
/* MissionSuccess */
/******************/
PortsList MissionSuccess::providedPorts() {
    return { 
        BT::InputPort<int>("base_index"),
        BT::InputPort<int>("levels")
    };
}

BT::NodeStatus MissionSuccess::tick() {
    base_index_ = getInput<int>("base_index").value();
    int levels_ = getInput<int>("levels").value();
    bool lastMissionFailed_;

    lastMissionFailed_ = false;
    blackboard_->set<bool>("last_mission_failed", lastMissionFailed_);
    if (!blackboard_->get<int>("front_materials", front_materials_) ||
        !blackboard_->get<int>("back_materials", back_materials_) ||
        !blackboard_->get<int>("score_from_main", score_)) {
        throw std::runtime_error("blackboard variable not found!");
    }
    if (base_index_ > 10) {   // finish placement mission
        switch (levels_) {
            case 1:
                back_materials_ -= 1;
                break;
            case 2:
                front_materials_ -= 2;
                break;
            case 3:
                front_materials_ -= 2;
                back_materials_ -= 1;
                break;
            default:
                break;
        }
        // update mission_points_status_
        if (!blackboard_->get<std_msgs::msg::Int32MultiArray>("mission_points_status", mission_points_status_)) {
            throw std::runtime_error("blackboard variable not found!");
        }
        mission_points_status_.data[base_index_ - 11]++;
        blackboard_->set<std_msgs::msg::Int32MultiArray>("mission_points_status", mission_points_status_);
        // update score
        score_ += (pow(2, levels_) - 1) * 4;
        // send the finished area index to vision
        // publish score from main
        blackboard_->set<int>("score_from_main", score_);
        std::thread{std::bind(&MissionSuccess::timer_publisher, this)}.detach();
        RCLCPP_INFO_STREAM(node_->get_logger(), "place materials at point " << base_index_ << " successfully, data: " << mission_points_status_.data[base_index_ - 11]);
    } else if (base_index_ >= 0) {  // finish taking material
        switch (levels_) {
            case 1:
                back_materials_ += 2;
                break;
            case 2:
                front_materials_ += 2;
                break;
            default:
                break;
        }
        if (!blackboard_->get<std_msgs::msg::Int32MultiArray>("materials_info", materials_info_)) {
            throw std::runtime_error("blackboard variable not found!");
        }
        materials_info_.data[base_index_] = 0;
        blackboard_->set<std_msgs::msg::Int32MultiArray>("materials_info", materials_info_);
        RCLCPP_INFO_STREAM(node_->get_logger(), "take materials at point " << base_index_ << " successfully");
    } else if (base_index_ == -2) { // finish banner mission
        score_ += 20;
        blackboard_->set<int>("score_from_main", score_);
        std::thread{std::bind(&MissionSuccess::timer_publisher, this)}.detach();
    } else { // go home: -1
        score_ += 10;
        blackboard_->set<int>("score_from_main", score_);
        std::thread{std::bind(&MissionSuccess::timer_publisher, this)}.detach();
    }
    blackboard_->set<int>("front_materials", front_materials_);
    blackboard_->set<int>("back_materials", back_materials_);
    return BT::NodeStatus::SUCCESS;
}

void MissionSuccess::timer_publisher() {
    rclcpp::Rate rate(100);
    mission_finished.data = base_index_;
    ideal_score.data = score_;

    while (rclcpp::ok() && publish_count < publish_times) { 
        vision_pub_->publish(mission_finished);
        ideal_score_pub_->publish(ideal_score);
        publish_count++;
        rate.sleep();
    }
}

/*******************/
/* MissionFailure */
/*******************/
PortsList MissionFailure::providedPorts() {
    return { 
        BT::InputPort<std::deque<int>>("step_results"),
        BT::InputPort<bool>("robot_type"),
        BT::InputPort<int>("base_index"),
        BT::OutputPort<int>("success_levels"), 
        BT::OutputPort<int>("failed_levels")
    };
}

BT::NodeStatus MissionFailure::onStart()
{
    getInput<int>("base_index", base_index_);
    getInput<std::deque<int>>("step_results", step_results_);
    getInput<bool>("robot_type", robot_type_);
    if (!blackboard_->get<int>("mission_progress", mission_progress_) ||
        !blackboard_->get<int>("front_materials", front_materials_) ||
        !blackboard_->get<int>("back_materials", back_materials_)) {
        throw std::runtime_error("blackboard variable not found!");
    }
    blackboard_->set<int>("mission_progress", 0);
    
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "finisher get step_results: %d", step_results_.back());
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "finisher get robot_type_: %d", robot_type_);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MissionFailure::onRunning()
{
    auto front_collect_ = node_->get_parameter("front_collect").as_integer_array();
    auto back_collect_ = node_->get_parameter("back_collect").as_integer_array();
    auto construct_1_ = node_->get_parameter("construct_1").as_integer_array();
    auto construct_2_ = node_->get_parameter("construct_2").as_integer_array();
    auto construct_3_ = node_->get_parameter("construct_3").as_integer_array();
    
    switch(step_results_.back()){

        // front_collect
        case 1:
            // front_materials_ += front_collect_[1];
            // back_materials_ += front_collect_[2];
            success_levels_ = 0;
            failed_levels_ = 0;

            if (front_materials_ == 3)
                matreials_accord_ = 1;

            break;
        // back_collect
        case 2:
            // front_materials_ += back_collect_[1];
            // back_materials_ += back_collect_[2];
            success_levels_ = 0;
            failed_levels_ = 0;

            if (back_materials_ == 1)
                matreials_accord_ = 1;

            break;
        // construct_1
        case 3:
            // front_materials_ = construct_1_[1];
            // back_materials_ = construct_1_[2];
            success_levels_ = construct_1_[3];
            failed_levels_ = 1 - success_levels_;

            if (front_materials_ == 0)
                matreials_accord_ = 1;

            break;
        // construct_2
        case 4:
            // front_materials_ = construct_2_[mission_progress_ * 4 + 1];
            // back_materials_ = construct_2_[mission_progress_ * 4 + 2];
            success_levels_ = construct_2_[mission_progress_ * 4 + 3];
            failed_levels_ = 2 - success_levels_;

            if (front_materials_ == 0 && back_materials_ == 0)
                matreials_accord_ = 1;

            break;
        // construct_3
        case 5:
            // front_materials_ = construct_3_[mission_progress_ * 4 + 1];
            // back_materials_ = construct_3_[mission_progress_ * 4 + 2];
            
            if (construct_3_[mission_progress_ * 4 + 3] != 4)
            {
                success_levels_ = construct_3_[mission_progress_ * 4 + 3];
                failed_levels_ = 3 - success_levels_;
            }
            // 疊三層時，過程中兩層的成功與否受到建造區影響
            else
            {
                success_levels_ = 1;
                failed_levels_ = 2;
            }
        

            if (front_materials_ == 0 && back_materials_ == 0)
                matreials_accord_ = 1;
            
            break;
        default:
            break;
    }
    if (base_index_ > 10) {
        if (!blackboard_->get<std_msgs::msg::Int32MultiArray>("mission_points_status", mission_points_status_)) {
            throw std::runtime_error("blackboard variable not found!");
        }
        mission_points_status_.data[base_index_ - 11]++;
        blackboard_->set<std_msgs::msg::Int32MultiArray>("mission_points_status", mission_points_status_);
    }
    blackboard_->set<int>("front_materials", front_materials_);
    blackboard_->set<int>("back_materials", back_materials_);

    setOutput("success_levels", success_levels_); 
    setOutput("failed_levels", failed_levels_);

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "mission_progress: %d", mission_progress_);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "success_levels: %d, failed_levels: %d", success_levels_, failed_levels_);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "front_materials: %d, back_materials: %d", front_materials_, back_materials_);
    
    if (matreials_accord_)
        return BT::NodeStatus::SUCCESS;
    else
        return BT::NodeStatus::FAILURE;
}

void MissionFailure::onHalted()
{
    // Reset the output port
    setOutput("success_levels", 0); 
    setOutput("failed_levels", 0); 

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

BT::NodeStatus FirmwareMission::stopStep() {
    if (mission_status_ == 1 && mission_stamp_ == mission_type_) {
        // RCLCPP_INFO(node_->get_logger(), "Mission success");
        blackboard_->set<int>("mission_progress", ++mission_progress_);
        setOutput<int>("mission_status", mission_status_);
        // mission_received_ = false;
        mission_status_ = 0;
        subscription_.reset();
        return BT::NodeStatus::SUCCESS;
    } else if (mission_status_ == 0 || (mission_status_ == 1 && mission_stamp_ != mission_type_)) {
        return BT::NodeStatus::RUNNING;
    } else if (mission_status_ == -1) {
        // RCLCPP_INFO(node_->get_logger(), "Mission failed");
        setOutput<int>("mission_status", mission_status_);
        // mission_received_ = false;
        subscription_.reset();
        return BT::NodeStatus::FAILURE;
    } else {
        // RCLCPP_INFO(node_->get_logger(), "Unknown status code, Stop mission");
        setOutput<int>("mission_status", -1);
        subscription_.reset();
        return BT::NodeStatus::FAILURE;
    }
}
void FirmwareMission::mission_callback(const std_msgs::msg::Int32::SharedPtr sub_msg) {
    if (mission_type_)
    {
        // if (sub_msg->data == 0)
            // mission_received_ = true;
        int temp_ = sub_msg->data;
        mission_stamp_ = temp_ / 10;
        mission_status_ = temp_ % 10;
        // RCLCPP_INFO(node_->get_logger(), "mission type: %d, return: %d, status: %d", mission_type_, mission_stamp_, mission_status_);
    }
}

BT::NodeStatus FirmwareMission::onStart() {
    // RCLCPP_INFO(node_->get_logger(), "Node start");
    getInput<int>("mission_type", mission_type_);
    if (!blackboard_->get<int>("mission_progress", mission_progress_)) {
        throw std::runtime_error("blackboard variable not found!");
    }
    // RCLCPP_INFO(node_->get_logger(), "mission_type: %d", mission_type_);
    // RCLCPP_INFO(node_->get_logger(), "-----------------");
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FirmwareMission::onRunning() {
    pub_msg.data = mission_type_;
    publisher_->publish(pub_msg);
    // RCLCPP_INFO(node_->get_logger(), "mission_progress: %d", mission_progress_);
    // RCLCPP_INFO(node_->get_logger(), "mission_type: %d", mission_type_);
    rate_.sleep();
    return stopStep();
    // **failure test**
    // if (mission_progress_ != 5)
    //    blackboard_->set<int>("mission_progress", ++mission_progress_);
    // return BT::NodeStatus::SUCCESS;
}

void FirmwareMission::onHalted() {
    // RCLCPP_INFO(node_->get_logger(), "Testing Node halted");
    setOutput<int>("mission_status", -1);
    return;
}

BT::PortsList BannerChecker::providedPorts() {
    return {
        BT::InputPort<int>("banner_place"),
        BT::OutputPort<std::string>("remap_banner_place")
    };
}

void BannerChecker::onStart() {
    getInput<int>("banner_place", banner_place_);

    std::string mapPointsFile_, bot_;
    if (!blackboard_->get<std::string>("bot", bot_) ||
        !blackboard_->get<std::string>("team", team_) ||
        !blackboard_->get<std_msgs::msg::Int32MultiArray>("mission_points_status", mission_points_status_)) {
        throw std::runtime_error("blackboard variable not found!");
    }
    // get correspond map points according to bot name
    mapPointsFile_ = "map_points_" + bot_;
    node_->get_parameter(mapPointsFile_, material_points_);
    // node_->get_parameter("safety_dist", safety_dist_);                          // unused for now
    LocReceiver::UpdateRobotPose(robot_pose_, tf_buffer_, frame_id_);
    // LocReceiver::UpdateRivalPose(rival_pose_, tf_buffer_, frame_id_);           // unused for now
}

int BannerChecker::DecodeBannerIndex(const int index) {
    if (team_ == "yellow")
        return index + 12;
    else if (team_ == "blue")
        return index + 16;
    else
        throw("error team color");
}

BT::NodeStatus BannerChecker::tick() {
    geometry_msgs::msg::Pose ptPose_;
    int mapPoint_;
    int i = 0;

    onStart();
    mapPoint_ = DecodeBannerIndex(banner_place_);
    while (mission_points_status_.data[mapPoint_ - 11] && i < 3) {
        RCLCPP_INFO_STREAM(node_->get_logger(), "considering banner_place_: " << banner_place_ << ", status: " << mission_points_status_.data[mapPoint_ - 11]);
        banner_place_ = (banner_place_ + 1) % 3;
        mapPoint_ = DecodeBannerIndex(banner_place_);
        i++;
    }
    
    RCLCPP_INFO_STREAM(node_->get_logger(), "final decision: " << banner_place_);
    ptPose_.position.x = material_points_[mapPoint_ * 7];
    ptPose_.position.y = material_points_[mapPoint_ * 7 + 1];
    if (calculateDistance(ptPose_, robot_pose_.pose) < 0.05) {
        RCLCPP_INFO_STREAM(node_->get_logger(), "already at the banner place");
        setOutput("remap_banner_place", to_string(3));
        return BT::NodeStatus::SUCCESS;
    }
    setOutput("remap_banner_place", to_string(banner_place_));
    return BT::NodeStatus::SUCCESS;
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
            auto nav_client = std::make_shared<NavigateClient>(offset, base_.pose);
            while (!nav_client->is_nav_finished()) {
              rclcpp::spin_some(nav_client);
            }
        }
        pub_msg.data = mission_type_;
        publisher_->publish(pub_msg);
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