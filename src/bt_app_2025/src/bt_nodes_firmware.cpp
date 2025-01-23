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

/********************/
/* Firmware Mission */
/********************/
BT::PortsList BTMission::providedPorts() {
    return { 
        BT::InputPort<std::deque<int>>("mission_type"),
        BT::InputPort<std::deque<int>>("mission_sub_type"),
        BT::InputPort<std::string>("description"),
        BT::OutputPort<int>("result")
    };
}
bool BTMission::setGoal(RosActionNode::Goal& goal) {
    mission_type_ = getInput<std::deque<int>>("mission_type").value();
    mission_sub_type_ = getInput<std::deque<int>>("mission_sub_type").value();

    goal.mission_type = mission_type_.front();
    goal.mission_sub_type = mission_sub_type_.front();

    mission_finished_ = false;
    mission_failed_ = false;

    // blackboard()->set<std::string>("global_param", "Updated Value");
    // blackboard()->get("global_param", value);
    // std::cout << "Global parameter: " << value << std::endl;

    return true;
}
NodeStatus BTMission::onResultReceived(const WrappedResult& wr) {
    // result_ = wr.result->outcome.data;
    mission_finished_ = true;
    mission_failed_ = false;
    setOutput<int>("result", mission_type_.front());
    return NodeStatus::SUCCESS;
}
NodeStatus BTMission::onFailure(ActionNodeErrorCode error) {
    mission_failed_ = true;
    setOutput<int>("result", mission_type_.front());
    RCLCPP_ERROR(logger(), "[BT]: Navigation error");
    return NodeStatus::FAILURE;
}
NodeStatus BTMission::onFeedback(const std::shared_ptr<const Feedback> feedback) {
    progress_ = feedback->progress.data;
    return NodeStatus::RUNNING;
}

/******************/
/* Banner Mission */
/******************/

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
/*******************/
/* CollectFinisher */
/*******************/
PortsList CollectFinisher::providedPorts() {
    return { 
        BT::InputPort<std::deque<int>>("step_results"),
        BT::OutputPort<bool>("has_raw_material"), 
        BT::OutputPort<bool>("has_one_level"), 
        BT::OutputPort<bool>("has_garbage")
    };
}

BT::NodeStatus CollectFinisher::onStart()
{
    getInput<std::deque<int>>("step_results", step_results_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CollectFinisher::onRunning()
{
    // To Do: 
    setOutput("has_raw_material", has_raw_material_); 
    setOutput("has_one_level", has_one_level_); 
    setOutput("has_garbage", has_garbage_);
    return BT::NodeStatus::SUCCESS;
}

void CollectFinisher::onHalted()
{
    // Reset the output port
    setOutput("has_raw_material", false); 
    setOutput("has_one_level", false); 
    setOutput("has_garbage", false);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Testing Node halted");
    return;
}
/*********************/
/* ConstructFinisher */
/*********************/
PortsList ConstructFinisher::providedPorts() {
    return { 
        BT::InputPort<std::deque<int>>("step_results"),
        BT::OutputPort<int>("success_levels"), 
        BT::OutputPort<int>("failed_levels"), 
        BT::OutputPort<bool>("has_garbage"), 
        BT::OutputPort<int>("on_robot_materials")
    };
}

BT::NodeStatus ConstructFinisher::onStart()
{
    getInput<std::deque<int>>("step_results", step_results_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ConstructFinisher::onRunning()
{
    // To Do: 
    setOutput("success_levels", success_levels_); 
    setOutput("failed_levels", failed_levels_); 
    setOutput("has_garbage", has_garbage_);
    setOutput("on_robot_materials", on_robot_materials_);
    
    return BT::NodeStatus::SUCCESS;
}

void ConstructFinisher::onHalted()
{
    // Reset the output port
    setOutput("success_levels", 0); 
    setOutput("failed_levels", 0); 
    setOutput("has_garbage", false);
    setOutput("on_robot_materials", 0);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Testing Node halted");
    return;
}