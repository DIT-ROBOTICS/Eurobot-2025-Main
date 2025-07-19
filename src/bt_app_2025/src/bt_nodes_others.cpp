#include "bt_app_2025/bt_nodes_others.h"
#include "bt_app_2025/bt_nodes_receiver.h"

using namespace BT;
using namespace std;

template <> inline int BT::convertFromString(StringView str) {
    auto value = convertFromString<double>(str);
    return (int)value;
}

template <> inline std::deque<int> BT::convertFromString(StringView str) {

    auto parts = splitString(str, ',');
    std::deque<int> output;

    for (int i = 0; i < (int)parts.size(); i++) {
        output.push_back(convertFromString<int>(parts[i]));
    }

    return output;
}

geometry_msgs::msg::PoseStamped inline ConvertPoseFormat(geometry_msgs::msg::PoseStamped pose_) {
    tf2::Quaternion quaternion;
    tf2::fromMsg(pose_.pose.orientation, quaternion);
    pose_.pose.position.z = tf2::impl::getYaw(quaternion) * 2 / PI;
    return pose_;
}

BT::PortsList GetLocation::providedPorts() {
    return {
        BT::InputPort<std::string>("locaization_target"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose")
    };
}

BT::NodeStatus GetLocation::tick() {
    std::string locaizationTarget_;

    getInput<std::string>("locaization_target", locaizationTarget_);
    if (locaizationTarget_ == "robot") {
        LocReceiver::UpdateRobotPose(pose_, tf_buffer_, frame_id_);
    } else if (locaizationTarget_ == "rival") {
        LocReceiver::UpdateRivalPose(pose_, tf_buffer_, frame_id_);
    } else {
        return BT::NodeStatus::FAILURE;
    }
    setOutput<geometry_msgs::msg::PoseStamped>("pose", ConvertPoseFormat(pose_));

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList GetBlackboard::providedPorts() {
    return {
        BT::InputPort<std::string>("blackboard_key"),
        BT::InputPort<std::string>("blackboard_type")
    };
}

BT::NodeStatus GetBlackboard::tick() {
    std::string blackboard_key_, blackboard_type_;

    getInput<std::string>("blackboard_key", blackboard_key_);
    getInput<std::string>("blackboard_type", blackboard_type_);
    if (blackboard_type_ == "bool") {
        bool blackboard_value_;
        if (!blackboard_->get<bool>(blackboard_key_, blackboard_value_))
            return BT::NodeStatus::FAILURE;
        RCLCPP_INFO_STREAM(node_->get_logger(), "Get BlackBoard " << blackboard_key_ << ": " << blackboard_value_);
    }
    else if (blackboard_type_ == "int") {
        int blackboard_value_;
        if (!blackboard_->get<int>(blackboard_key_, blackboard_value_))
            return BT::NodeStatus::FAILURE;
        RCLCPP_INFO_STREAM(node_->get_logger(), "Get BlackBoard " << blackboard_key_ << ": " << blackboard_value_);
    }
    else if (blackboard_type_ == "float") {
        float blackboard_value_;
        if (!blackboard_->get<float>(blackboard_key_, blackboard_value_))
            return BT::NodeStatus::FAILURE;
        RCLCPP_INFO_STREAM(node_->get_logger(), "Get BlackBoard " << blackboard_key_ << ": " << blackboard_value_);
    }
    else if (blackboard_type_ == "double") {  
        double blackboard_value_;
        if (!blackboard_->get<double>(blackboard_key_, blackboard_value_))
            return BT::NodeStatus::FAILURE;
        RCLCPP_INFO_STREAM(node_->get_logger(), "Get BlackBoard " << blackboard_key_ << ": " << blackboard_value_);
    }
    else if (blackboard_type_ == "string") {
        std::string blackboard_value_;
        if (!blackboard_->get<std::string>(blackboard_key_, blackboard_value_))
            return BT::NodeStatus::FAILURE;
        RCLCPP_INFO_STREAM(node_->get_logger(), "Get BlackBoard " << blackboard_key_ << ": " << blackboard_value_);
    }

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList DemoSourcePoint::providedPorts() {
    return {
        BT::OutputPort<int>("material_pt_1"),
        BT::OutputPort<int>("material_pt_2"),
        BT::OutputPort<int>("mission_pt"),
        BT::OutputPort<int>("end_pt")
    };
}

BT::NodeStatus DemoSourcePoint::tick() {
    setOutput<int>("material_pt_1", plan_script_[1]);
    setOutput<int>("material_pt_2", plan_script_[2]);
    setOutput<int>("mission_pt", plan_script_[3]);
    setOutput<int>("end_pt", plan_script_[plan_script_.size() - 1]);
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList MySetBlackboard::providedPorts() {
    return {
        BT::InputPort<std::string>("blackboard_key"),
        BT::InputPort<bool>("blackboard_value"),
        BT::OutputPort<bool>("new_value")
    };
}

BT::NodeStatus MySetBlackboard::tick() {
    std::string blackboard_key_;
    bool blackboard_value_;
    getInput<std::string>("blackboard_key", blackboard_key_);
    getInput<bool>("blackboard_value", blackboard_value_);
    RCLCPP_INFO_STREAM(node_->get_logger(), blackboard_key_ << ": " << blackboard_value_);

    blackboard_->set<bool>(blackboard_key_, blackboard_value_);
    setOutput<bool>("new_value", blackboard_value_);
    return BT::NodeStatus::SUCCESS;
}

/*************/
/* BTStarter */
/*************/
BT::PortsList BTStarter::providedPorts() {
    return { BT::OutputPort<int>("result") };
}

BT::NodeStatus BTStarter::tick() {
    subscription_ = node_->create_subscription<std_msgs::msg::Float32>("/robot/startup/time", 10, std::bind(&BTStarter::topic_callback, this, std::placeholders::_1));
    keepout_zone_pub_ = node_->create_publisher<std_msgs::msg::String>("/keepout_zone", rclcpp::QoS(20).reliable().transient_local());
    if (!blackboard_->get<std::string>("team", team_)) {
        throw std::runtime_error("team not found in blackboard!");
    }
    if (team_ == "yellow") {
        keepout_zone_.data = "BCFGJ";
        RCLCPP_INFO_STREAM(node_->get_logger(), keepout_zone_.data);
    }
    else if (team_ == "blue") {
        keepout_zone_.data = "ADEHI";
        RCLCPP_INFO_STREAM(node_->get_logger(), keepout_zone_.data);
    }
    else {
        throw std::runtime_error("team variable not correct!");
    }
    keepout_zone_pub_->publish(keepout_zone_);

    int game_status = 0;
    setOutput<int>("result", game_status);
    return BT::NodeStatus::SUCCESS;
}

void BTStarter::topic_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    current_time_ = msg->data;
    blackboard_->set<double>("current_time", current_time_);
}

/****************/
/* TimerChecker */
/****************/
BT::PortsList TimerChecker::providedPorts() {
    return { 
        BT::InputPort<double>("target_time_sec")
    };
}

BT::NodeStatus TimerChecker::tick() {

    double targetTimeSec_ = getInput<double>("target_time_sec").value();

    if (!blackboard_->get<double>("current_time", current_time_))
        return BT::NodeStatus::FAILURE;

    if (current_time_ < targetTimeSec_)
        return BT::NodeStatus::SUCCESS;
    else
        return BT::NodeStatus::FAILURE;
}

// /****************************************************/
// /* Simple Node for finding the rival start position */
// /****************************************************/
// BT::PortsList RivalStart::providedPorts() {
//     return { 
//         BT::InputPort<int>("start_type")
//     };
// }

// BT::NodeStatus RivalStart::tick() {
//     // if (this->kernel_ == nullptr) {
//     //     RCLCPP_ERROR(logger(), "[RivalStart]: Kernel is null");
//     //     return BT::NodeStatus::FAILURE;
//     // }

//     check_start_point_ = getInput<int>("start_type").value();

//     // Get rival position from kernel
//     geometry_msgs::msg::TwistStamped rival_pose;
//     // kernel_->GetRivalPosePtr(rival_pose);

//     // Log the rival pose
//     RCLCPP_INFO(logger(), "[RivalStart]: Rival pose: " << rival_pose.twist.linear.x << " " << rival_pose.twist.linear.y);

//     switch (check_start_point_) {
//         case 0:
//             if (team_ == "b") {
//                 /* Checking  Rival points with central point (blue team) */
//                 if (rival_pose.twist.linear.x < 0.7) {
//                     RCLCPP_INFO(logger(), "[RivalStart]: Rival is in the blue team central start point");
//                     return BT::NodeStatus::SUCCESS;
//                 }
//                 else {
//                     RCLCPP_INFO(logger(), "[RivalStart]: Rival is not in the blue team central start point");
//                     return BT::NodeStatus::FAILURE;
//                 }
//             }
//             else {
//                 /* Checking rival points with central point (yellow team) */
//                 if (rival_pose.twist.linear.x > 2.3) {
//                     RCLCPP_INFO(logger(), "[RivalStart]: Rival is in the yellow team central start point");
//                     return BT::NodeStatus::SUCCESS;
//                 }
//                 else {
//                     RCLCPP_INFO(logger(), "[RivalStart]: Rival is not in the yellow team central start point");
//                     return BT::NodeStatus::FAILURE;
//                 }
//             }
//             break;
//         case 1:
//             /* Checking rival points with up point (blue and yellow team) */
//             if (rival_pose.twist.linear.y > 1.4) {
//                 RCLCPP_INFO(logger(), "[RivalStart]: Rival is in the up start point");
//                 return BT::NodeStatus::SUCCESS;
//             }
//             else {
//                 RCLCPP_INFO(logger(), "[RivalStart]: Rival is not in the up start point");
//                 return BT::NodeStatus::FAILURE;
//             }
//             break;
//         case 2:
//             /* Checking rival points with below point (blue and yellow team) */
//             if (rival_pose.twist.linear.y < 0.6) {
//                 RCLCPP_INFO(logger(), "[RivalStart]: Rival is in the below start point");
//                 return BT::NodeStatus::SUCCESS;
//             }
//             else {
//                 RCLCPP_INFO(logger(), "[RivalStart]: Rival is not in the below start point");
//                 return BT::NodeStatus::FAILURE;
//             }
//             break;
//         default:
//             return BT::NodeStatus::SUCCESS;
//     }
// }