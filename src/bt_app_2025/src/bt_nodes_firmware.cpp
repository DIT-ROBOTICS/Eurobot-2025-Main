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
/********************/
/* MissionFinisher */
/********************/
PortsList MissionFinisher::providedPorts() {
    return { 
        BT::InputPort<std::deque<int>>("step_results"),
        BT::InputPort<bool>("robot_type"),
        BT::OutputPort<int>("success_levels"), 
        BT::OutputPort<int>("failed_levels")
    };
}

BT::NodeStatus MissionFinisher::onStart()
{
    getInput<std::deque<int>>("step_results", step_results_);
    getInput<bool>("robot_type", robot_type_);
    blackboard_->get<int>("mission_progress", mission_progress_);
    blackboard_->get<int>("front_materials", front_materials_);
    blackboard_->get<int>("back_materials", back_materials_);
    blackboard_->set<int>("mission_progress", 0);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MissionFinisher::onRunning()
{
    auto front_collect_ = matrix_node_->get_parameter("front_collect").as_integer_array();
    auto back_collect_ = matrix_node_->get_parameter("back_collect").as_integer_array();
    auto construct_1_ = matrix_node_->get_parameter("construct_1").as_integer_array();
    auto not_spin_construct_2_ = matrix_node_->get_parameter("not_spin_construct_2").as_integer_array();
    auto spin_construct_2_ = matrix_node_->get_parameter("spin_construct_2").as_integer_array();
    auto not_spin_construct_3_ = matrix_node_->get_parameter("not_spin_construct_3").as_integer_array();
    auto spin_construct_3_ = matrix_node_->get_parameter("spin_construct_3").as_integer_array();
    switch(mission_progress_){

        // front_collect
        case 1:
            front_materials_ += front_collect_[1];
            back_materials_ += front_collect_[2];
            success_levels_ = 0;
            failed_levels_ = 0;

            if (front_materials_ == 3)
                matreials_accord_ = 1;

            break;
        // back_collect
        case 2:
            front_materials_ += back_collect_[1];
            back_materials_ += back_collect_[2];
            success_levels_ = 0;
            failed_levels_ = 0;

            if (back_materials_ == 1)
                matreials_accord_ = 1;

            break;
        // construct_1
        case 3:
            front_materials_ = construct_1_[1];
            back_materials_ = construct_1_[2];
            success_levels_ = construct_1_[3];
            failed_levels_ = 1 - success_levels_;

            if (front_materials_ == 0)
                matreials_accord_ = 1;

            break;
        // construct_2
        case 4:
            if(robot_type_)
            {
                front_materials_ = not_spin_construct_2_[mission_progress_ * 4 + 2];
                back_materials_ = not_spin_construct_2_[mission_progress_ * 4 + 3];
                success_levels_ = not_spin_construct_2_[mission_progress_ * 4 + 4];
                failed_levels_ = 2 - success_levels_;
            }
            else
            {
                front_materials_ = spin_construct_2_[mission_progress_ * 4 + 2];
                back_materials_ = spin_construct_2_[mission_progress_ * 4 + 3];
                success_levels_ = spin_construct_2_[mission_progress_ * 4 + 4];
                failed_levels_ = 2 - success_levels_;
            }

            if (front_materials_ == 0 && back_materials_ == 0)
                matreials_accord_ = 1;

            break;
        // construct_3
        case 5:
            if(robot_type_)
            {
                front_materials_ = not_spin_construct_3_[mission_progress_ * 4 + 2];
                back_materials_ = not_spin_construct_3_[mission_progress_ * 4 + 3];
                
                // 疊三層時，過程中兩層的成功與否受到建造區影響
                if (not_spin_construct_3_[mission_progress_ * 4 + 4] != 4)
                {
                    success_levels_ = not_spin_construct_3_[mission_progress_ * 4 + 4];
                    failed_levels_ = 3 - success_levels_;
                }
                else
                {
                    success_levels_ = 1;
                    failed_levels_ = 2;
                }
            }
            else
            {
                front_materials_ = spin_construct_3_[mission_progress_ * 4 + 2];
                back_materials_ = spin_construct_3_[mission_progress_ * 4 + 3];
                
                // 疊三層時，過程中兩層的成功與否受到建造區影響
                if (spin_construct_3_[mission_progress_ * 4 + 4] != 4)
                {
                    success_levels_ = spin_construct_3_[mission_progress_ * 4 + 4];
                    failed_levels_ = 3 - success_levels_;
                }
                else
                {
                    success_levels_ = 1;
                    failed_levels_ = 2;
                }
            }

            if (front_materials_ == 0 && back_materials_ == 0)
                matreials_accord_ = 1;
            
            break;
        default:
            break;
    }

    blackboard_->set<int>("front_materials", front_materials_);
    blackboard_->set<int>("back_materials", back_materials_);

    setOutput("success_levels", success_levels_); 
    setOutput("failed_levels", failed_levels_);
    
    if (matreials_accord_)
        return BT::NodeStatus::SUCCESS;
    else
        return BT::NodeStatus::FAILURE;
}

void MissionFinisher::onHalted()
{
    // Reset the output port
    setOutput("success_levels", 0); 
    setOutput("failed_levels", 0); 

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Testing Node halted");
    return;
}