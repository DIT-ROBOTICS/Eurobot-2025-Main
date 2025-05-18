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

bool LocReceiver::UpdateRobotPose(geometry_msgs::msg::PoseStamped &robot_pose_, tf2_ros::Buffer &tf_buffer_, std::string frame_id_) {
    geometry_msgs::msg::TransformStamped transformStamped;

    try {
        transformStamped = tf_buffer_.lookupTransform(
            "map", 
            frame_id_,
            rclcpp::Time()
        );
        robot_pose_.pose.position.x = transformStamped.transform.translation.x;
        robot_pose_.pose.position.y = transformStamped.transform.translation.y;
        robot_pose_.pose.position.z = 0;
        robot_pose_.pose.orientation = transformStamped.transform.rotation;
        return true;
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("localization"), "[UpdateRobotPose]: line " << __LINE__ << " " << ex.what());
        return false;
    }
}

bool LocReceiver::UpdateRivalPose(geometry_msgs::msg::PoseStamped &rival_pose_, tf2_ros::Buffer &tf_buffer_, std::string frame_id_) {
    geometry_msgs::msg::TransformStamped transformStamped;

    try {
        transformStamped = tf_buffer_.lookupTransform(
            "map" /* Parent frame - map */, 
            "rival/" + frame_id_ /* Child frame - base */,
            rclcpp::Time()
        );
        rival_pose_.pose.position.x = transformStamped.transform.translation.x;
        rival_pose_.pose.position.y = transformStamped.transform.translation.y;
        rival_pose_.pose.position.z = 0;
        rival_pose_.pose.orientation = transformStamped.transform.rotation;
        return true;
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("localization"), "[UpdateRivalPose]: line " << __LINE__ << " " << ex.what());
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

void CamReceiver::materials_info_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    materials_info_ = *msg;
    if (msg != NULL) {
        blackboard_->set<std_msgs::msg::Int32MultiArray>("materials_info", materials_info_);
    }
    // // print out the vision data for debug
    // int material_array = 0;
    // for (int i = 0; i < 10; i++)
    //     material_array += materials_info_.data[i] * pow(10, (9 - i));
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("cam receiver"), material_array);
}

void CamReceiver::mission_info_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    mission_info_ = msg->data;
    // blackboard_->set<std_msgs::msg::Int32>("mission_info", mission_info_);
}

void CamReceiver::score_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    score_vision_ = msg->data;
    blackboard_->get<int>("score_from_main", score_main_);
}

BT::NodeStatus CamReceiver::tick() {
    RCLCPP_INFO(node_->get_logger(), "Node start");
    sub_materials_info_ = node_->create_subscription<std_msgs::msg::Int32MultiArray>("/detected/global_center_poses/has_material", 10, std::bind(&CamReceiver::materials_info_callback, this, std::placeholders::_1));
    sub_mission_info_ = node_->create_subscription<std_msgs::msg::Int32>("/robot/objects/mission_info", 10, std::bind(&CamReceiver::mission_info_callback, this, std::placeholders::_1));
    sub_score_ = node_->create_subscription<std_msgs::msg::Int32>("/vision/score", 10, std::bind(&CamReceiver::score_callback, this, std::placeholders::_1));
    
    return BT::NodeStatus::SUCCESS;
}

PortsList TopicSubTest::providedPorts() {
    return { 
      BT::OutputPort<int>("sum") 
    };
}
  
BT::NodeStatus TopicSubTest::topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    number += msg->data;
    RCLCPP_INFO(node_->get_logger(), "I heard: '%d'", msg->data);
    if (number > 3) {
      subscription_.reset();
    }
    return BT::NodeStatus::SUCCESS;
}
  
BT::NodeStatus TopicSubTest::onStart() {
    RCLCPP_INFO(node_->get_logger(), "Node start");
    return BT::NodeStatus::RUNNING;
}
  
BT::NodeStatus TopicSubTest::onRunning() {
    setOutput<int>("sum", number);
    RCLCPP_INFO_STREAM(node_->get_logger(), number);
    if (number > 3) {
      subscription_.reset();
    }
    return BT::NodeStatus::SUCCESS;
}
  
void TopicSubTest::onHalted() {
    // Reset the output port
    subscription_.reset();
    setOutput<int>("sum", number);
    RCLCPP_INFO(node_->get_logger(), "Testing Node halted");
    return;
}