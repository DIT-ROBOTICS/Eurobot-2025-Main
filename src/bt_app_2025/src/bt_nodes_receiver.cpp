#include "bt_app_2025/bt_nodes.h"

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

PortsList LocReceiver::providedPorts() {
    return { 
        BT::OutputPort<geometry_msgs::msg::TwistStamped>("output") 
        BT::OutputPort<geometry_msgs::msg::TwistStamped>("output") 
    };
}

NodeStatus LocReceiver::onStart() {
    return BT::NodeStatus::RUNNING;
}

NodeStatus LocReceiver::onRunning() {
    UpdateRobotPose();
    std::cout << "robot_pose_:(" << robot_pose_.twist.linear.x << ", " << robot_pose_.twist.linear.y << ")\n";
    // Set the output port
    setOutput("output", robot_pose_);

    return BT::NodeStatus::SUCCESS;
}

void LocReceiver::UpdateRobotPose() {
    geometry_msgs::msg::TransformStamped transformStamped;

    try {
        transformStamped = tf_buffer_.lookupTransform(
        "robot/map" /* Parent frame - map */, 
        "robot/base_footprint" /* Child frame - base */,
        rclcpp::Time()
        );

        /* Extract (x, y, theta) from the transformed stamped */
        robot_pose_.twist.linear.x = transformStamped.transform.translation.x;
        robot_pose_.twist.linear.y = transformStamped.transform.translation.y;
        double theta;
        tf2::Quaternion q;
        tf2::fromMsg(transformStamped.transform.rotation, q);
        robot_pose_.twist.angular.z = tf2::impl::getYaw(q);
    }
    catch (tf2::TransformException &ex) {
        // RCLCPP_WARN_STREAM(node->get_logger(), "[Kernel::UpdateRobotPose]: line " << __LINE__ << " " << ex.what());
    }
}

void LocReceiver::UpdateRivalPose() {
    geometry_msgs::msg::TransformStamped transformStamped;

    try {
        transformStamped = tf_buffer_.lookupTransform(
        "robot/map" /* Parent frame - map */, 
        "robot/base_footprint" /* Child frame - base */,
        rclcpp::Time()
        );

        /* Extract (x, y, theta) from the transformed stamped */
        robot_pose_.twist.linear.x = transformStamped.transform.translation.x;
        robot_pose_.twist.linear.y = transformStamped.transform.translation.y;
        double theta;
        tf2::Quaternion q;
        tf2::fromMsg(transformStamped.transform.rotation, q);
        robot_pose_.twist.angular.z = tf2::impl::getYaw(q);
    }
    catch (tf2::TransformException &ex) {
        // RCLCPP_WARN_STREAM(node->get_logger(), "[Kernel::UpdateRobotPose]: line " << __LINE__ << " " << ex.what());
    }
}

void LocReceiver::onHalted() {
    // Reset the output port
    setOutput("output", robot_pose_);

    std::cout << "LocalizationTemp Node halted" << std::endl;

    return;
}

PortsList Docking::providedPorts() {
    return { 
        BT::InputPort<geometry_msgs::msg::TwistStamped>("base"),
        BT::InputPort<geometry_msgs::msg::TwistStamped>("offset"),
        BT::OutputPort<geometry_msgs::msg::TwistStamped>("result"),
        BT::InputPort<int>("mission_type")
    };
}

bool Docking::setGoal(RosActionNode::Goal& goal) {
    auto m = getInput<geometry_msgs::msg::TwistStamped>("base");
    auto offset = getInput<geometry_msgs::msg::TwistStamped>("offset");
    mission_type_ = getInput<int>("mission_type").value();
    goal_.twist.linear = m.value().twist.linear;
    goal_.twist.linear.x += offset.value().twist.linear.x;
    goal_.twist.linear.y += offset.value().twist.linear.y;
    goal_.twist.linear.z += offset.value().twist.linear.z;
    goal.nav_goal = goal_;
    goal.nav_goal.twist.angular.x = mission_type_;

    nav_finished_ = false;
    nav_error_ = false;
    start_mission_ = true;

    return true;
}

NodeStatus Docking::onFeedback(const std::shared_ptr<const Feedback> feedback) {
    auto feedback_ = feedback->progress;
    return NodeStatus::RUNNING;
}

NodeStatus Docking::onResultReceived(const WrappedResult& wr) {
    auto result_ = wr.result->outcome;
    nav_finished_ = true;
    start_mission_ = false;
    setOutput<geometry_msgs::msg::TwistStamped>("result", goal_);
    return NodeStatus::SUCCESS;
}

NodeStatus Docking::onFailure(ActionNodeErrorCode error) {
    nav_error_ = true;
    start_mission_ = false;
    setOutput<geometry_msgs::msg::TwistStamped>("result", goal_);
    RCLCPP_ERROR(logger(), "[BT]: Navigation error");
    return NodeStatus::FAILURE;
}

// BT::PortsList Recovery::providedPorts() {
//     return {
//         BT::InputPort<geometry_msgs::msg::TwistStamped>("base"),
//         BT::OutputPort<geometry_msgs::msg::TwistStamped>("result"),
//         BT::InputPort<int>("mission_type")
//     };
// }

// geometry_msgs::msg::TwistStamped Recovery::UpdateRobotPose() {

//     geometry_msgs::msg::TwistStamped robot_pose_;
//     geometry_msgs::msg::TransformStamped transformStamped;

//     try {
//         transformStamped = tf_buffer_.lookupTransform(
//         "robot/map" /* Parent frame - map */, 
//         "robot/base_footprint" /* Child frame - base */,
//         rclcpp::Time()
//         );

//         /* Extract (x, y, theta) from the transformed stamped */
//         robot_pose_.twist.linear.x = transformStamped.transform.translation.x;
//         robot_pose_.twist.linear.y = transformStamped.transform.translation.y;
//         double theta;
//         tf2::Quaternion q;
//         tf2::fromMsg(transformStamped.transform.rotation, q);
//         robot_pose_.twist.angular.z = tf2::impl::getYaw(q);
//     }
//     catch (tf2::TransformException &ex) {
//         // RCLCPP_WARN_STREAM(node->get_logger(), "[Kernel::UpdateRobotPose]: line " << __LINE__ << " " << ex.what());
//     }
//     return robot_pose_;
// }

// geometry_msgs::msg::TwistStamped Recovery::UpdateRivalPose() {

//     geometry_msgs::msg::TwistStamped rival_pose_;
//     geometry_msgs::msg::TransformStamped transformStamped;

//     try {
//         transformStamped = tf_buffer_.lookupTransform(
//         "rival/map" /* Parent frame - map */, 
//         "rival/base_footprint" /* Child frame - base */,
//         rclcpp::Time()
//         );

//         /* Extract (x, y, theta) from the transformed stamped */
//         rival_pose_.twist.linear.x = transformStamped.transform.translation.x;
//         rival_pose_.twist.linear.y = transformStamped.transform.translation.y;
//         double theta;
//         tf2::Quaternion q;
//         tf2::fromMsg(transformStamped.transform.rotation, q);
//         rival_pose_.twist.angular.z = tf2::impl::getYaw(q);
//     }
//     catch (tf2::TransformException &ex) {
//         // RCLCPP_WARN_STREAM(node->get_logger(), "[Kernel::UpdateRobotPose]: line " << __LINE__ << " " << ex.what());
//     }
//     return rival_pose_;
// }

// bool Recovery::setGoal(RosActionNode::Goal& goal) {

//     auto m = getInput<geometry_msgs::msg::TwistStamped>("base");
//     geometry_msgs::msg::TwistStamped robot_pose = UpdateRobotPose();
//     geometry_msgs::msg::TwistStamped rival_pose = UpdateRivalPose();
//     mission_type_ = getInput<int>("mission_type").value();
//     base_.twist.linear = m.value().twist.linear;

//     nav_finished_ = false;

//     // Get current vector by robot yaw
//     geometry_msgs::msg::TwistStamped normal_vector;
//     normal_vector.twist.linear.x = cos(robot_pose.twist.angular.z);
//     normal_vector.twist.linear.y = sin(robot_pose.twist.angular.z);
//     normal_vector.twist.linear.z = robot_pose.twist.angular.z;

//     // Transform rival pose to robot frame
//     geometry_msgs::msg::TwistStamped rival_pose_robot_frame;
//     rival_pose_robot_frame.twist.linear.x = rival_pose.twist.linear.x - robot_pose.twist.linear.x;
//     rival_pose_robot_frame.twist.linear.y = rival_pose.twist.linear.y - robot_pose.twist.linear.y;

//     // Dot product
//     double dot_product = robot_pose.twist.linear.x * rival_pose_robot_frame.twist.linear.x + \
//                          robot_pose.twist.linear.y * rival_pose_robot_frame.twist.linear.y;

//     // If the rival is in front of the robot -> backward
//     if (dot_product > 0) {
//         base_.twist.linear.x = robot_pose.twist.linear.x - distance_ * normal_vector.twist.linear.x;
//         base_.twist.linear.y = robot_pose.twist.linear.y - distance_ * normal_vector.twist.linear.y;
//         base_.twist.linear.z = robot_pose.twist.angular.z;
//     }
//     else {
//         base_.twist.linear.x = robot_pose.twist.linear.x + distance_ * normal_vector.twist.linear.x;
//         base_.twist.linear.y = robot_pose.twist.linear.y + distance_ * normal_vector.twist.linear.y;
//         base_.twist.linear.z = robot_pose.twist.angular.z;
//     }

//     // Log the normal vector and rival pose
//     RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "[Recovery]: Normal vector: " << normal_vector.twist.linear.x << " " << normal_vector.twist.linear.y << " " << normal_vector.twist.linear.z);
//     RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "[Recovery]: Robot pose: " << robot_pose.twist.linear.x << " " << robot_pose.twist.linear.y);
//     RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "[Recovery]: Rival pose: " << rival_pose.twist.linear.x << " " << rival_pose.twist.linear.y);
//     RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "[Recovery]: Dot product: " << dot_product);

//     // Clip x to [0.15, 2.85], y to [0.15, 1.85]
//     base_.twist.linear.x = std::max(0.15, std::min(2.85, base_.twist.linear.x));
//     base_.twist.linear.y = std::max(0.15, std::min(1.85, base_.twist.linear.y));

//     goal.nav_goal = base_;
//     goal.nav_goal.twist.angular.x = mission_type_;

//     // RCLCPP_INFO(logger(), "[Recovery]: Send base pose: " << base_.twist.linear.x << " " << base_.twist.linear.y);
//     cout << "[Recovery]: Send base pose: " << base_.twist.linear.x << " " << base_.twist.linear.y << endl;
//     return true;
// }

// NodeStatus Recovery::onFeedback(const std::shared_ptr<const Feedback> feedback) {
//     auto feedback_ = feedback->progress;
//     return NodeStatus::RUNNING;
// }

// NodeStatus Recovery::onResultReceived(const WrappedResult& wr) {
//     auto result_ = wr.result->outcome;
//     nav_finished_ = true;
//     start_mission_ = false;
//     setOutput<geometry_msgs::msg::TwistStamped>("result", base_);
//     return NodeStatus::SUCCESS;
// }

// NodeStatus Recovery::onFailure(ActionNodeErrorCode error) {
//     nav_error_ = true;
//     start_mission_ = false;
//     setOutput<geometry_msgs::msg::TwistStamped>("result", base_);
//     RCLCPP_ERROR(logger(), "[BT]: Navigation error");
//     return NodeStatus::FAILURE;
// }

BT::PortsList Comparator::providedPorts() {
    return { 
        BT::InputPort<geometry_msgs::msg::TwistStamped>("compare_point"),
        BT::InputPort<int>("mission_type"),
        BT::InputPort<int>("mission_sub_type"),
        BT::InputPort<int>("game_status")
    };
}

BT::NodeStatus Comparator::tick() {

    int mission_type = getInput<int>("mission_type").value();
    int mission_sub_type = getInput<int>("mission_sub_type").value();

    auto mi = getInput<geometry_msgs::msg::TwistStamped>("compare_point");
    mission_.twist.linear = mi.value().twist.linear;

    // Get current mission mask
    int mission_mask = 1 << mission_sub_type;

    // Check if the mission is finished
    int game_status = getInput<int>("game_status").value();
    if (mission_type == 0 && (game_status & mission_mask) != 0) {
        return BT::NodeStatus::FAILURE;
    }

    if (mission_type == 0 && CheckRivalInThreshold(mission_, 0.2, 0.6)) {
        return BT::NodeStatus::SUCCESS;
    }
    else if (mission_type != 0 && CheckRivalInRange(mission_, 0.5)) {
        return BT::NodeStatus::SUCCESS;
    }
    else {
        return BT::NodeStatus::FAILURE;
    }
}

geometry_msgs::msg::TwistStamped Comparator::UpdateRobotPose() {

    geometry_msgs::msg::TwistStamped robot_pose_;
    geometry_msgs::msg::TransformStamped transformStamped;

    try {
        transformStamped = tf_buffer_.lookupTransform(
        "robot/map" /* Parent frame - map */, 
        "robot/base_footprint" /* Child frame - base */,
        rclcpp::Time()
        );

        /* Extract (x, y, theta) from the transformed stamped */
        robot_pose_.twist.linear.x = transformStamped.transform.translation.x;
        robot_pose_.twist.linear.y = transformStamped.transform.translation.y;
        double theta;
        tf2::Quaternion q;
        tf2::fromMsg(transformStamped.transform.rotation, q);
        robot_pose_.twist.angular.z = tf2::impl::getYaw(q);
    }
    catch (tf2::TransformException &ex) {
        // RCLCPP_WARN_STREAM(node->get_logger(), "[Kernel::UpdateRobotPose]: line " << __LINE__ << " " << ex.what());
    }
    return robot_pose_;
}

geometry_msgs::msg::TwistStamped Comparator::UpdateRivalPose() {

    geometry_msgs::msg::TwistStamped rival_pose_;
    geometry_msgs::msg::TransformStamped transformStamped;

    try {
        transformStamped = tf_buffer_.lookupTransform(
        "rival/map" /* Parent frame - map */, 
        "rival/base_footprint" /* Child frame - base */,
        rclcpp::Time()
        );

        /* Extract (x, y, theta) from the transformed stamped */
        rival_pose_.twist.linear.x = transformStamped.transform.translation.x;
        rival_pose_.twist.linear.y = transformStamped.transform.translation.y;
        double theta;
        tf2::Quaternion q;
        tf2::fromMsg(transformStamped.transform.rotation, q);
        rival_pose_.twist.angular.z = tf2::impl::getYaw(q);
    }
    catch (tf2::TransformException &ex) {
        // RCLCPP_WARN_STREAM(node->get_logger(), "[Kernel::UpdateRobotPose]: line " << __LINE__ << " " << ex.what());
    }
    return rival_pose_;
}

bool Comparator::CheckRivalInThreshold(geometry_msgs::msg::TwistStamped pose, double threshold, double range) {

    geometry_msgs::msg::TwistStamped robot_pose = UpdateRobotPose();
    geometry_msgs::msg::TwistStamped rival_pose = UpdateRivalPose();

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Robot pose: " << robot_pose.twist.linear.x << " " << robot_pose.twist.linear.y);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Rival pose: " << rival_pose.twist.linear.x << " " << rival_pose.twist.linear.y);

    if (Distance(pose, robot_pose) < Distance(pose, rival_pose)) {
        // Log the distance
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Distance: " << Distance(pose, robot_pose) << " " << Distance(pose, rival_pose));

        return true;
    }
    else if (abs(Distance(pose, robot_pose) - Distance(pose, rival_pose)) < threshold && Distance(pose, rival_pose) > range) {

        // Log the distance
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Distance: " << Distance(pose, robot_pose) << " " << Distance(pose, rival_pose));

        return true;
    }
    else {
        return false;
    }
}

bool Comparator::CheckRivalInRange(geometry_msgs::msg::TwistStamped pose, double range) {

    geometry_msgs::msg::TwistStamped robot_pose = UpdateRobotPose();
    geometry_msgs::msg::TwistStamped rival_pose = UpdateRivalPose();

    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Robot pose: " << robot_pose.twist.linear.x << " " << robot_pose.twist.linear.y);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Rival pose: " << rival_pose.twist.linear.x << " " << rival_pose.twist.linear.y);

    if (Distance(pose, rival_pose) > range) {
        return true;
    }
    else {
        return false;
    }
}

double Comparator::Distance(geometry_msgs::msg::TwistStamped pose1, geometry_msgs::msg::TwistStamped pose2) {
    return sqrt(pow(pose1.twist.linear.x - pose2.twist.linear.x, 2) + pow(pose1.twist.linear.y - pose2.twist.linear.y, 2));
}

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
    auto result_ = wr.result->outcome;
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
    auto feedback_ = feedback->progress;
    return NodeStatus::RUNNING;
}

BT::PortsList BTStarter::providedPorts() {
    return { BT::OutputPort<int>("result") };
}

BT::NodeStatus BTStarter::tick() {

    int game_status = 0;
    setOutput<int>("result", game_status);

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList BTFinisher::providedPorts() {
    return { 
        BT::InputPort<int>("game_status"),
        BT::InputPort<int>("mission_type"),
        BT::InputPort<int>("mission_sub_type"),
        BT::OutputPort<int>("result") 
    };
}

BT::NodeStatus BTFinisher::tick() {
    
    int game_status = getInput<int>("game_status").value();
    int mission_type = getInput<int>("mission_type").value();
    int mission_sub_type = getInput<int>("mission_sub_type").value();

    if (mission_type == 0) {
        /* Plant mission */
        game_status += 1 << mission_sub_type;
    }
    else if (mission_type == 1) {
        /* Pot mission */
        game_status += 1 << (mission_sub_type + 12);
    }
    else if (mission_type == 2) {
        /* Planter mission */
        game_status += 1 << (mission_sub_type + 16);
    }
    else if (mission_type == 3) {
        if (mission_sub_type == 0) {
            /* Solar mission */
            game_status += 1 << 20;
        }
        else if (mission_sub_type == 1) {
            /* Solar mission */
            game_status += 1 << 21;
        }
    }
    else if (mission_type == 4) {
        /* Home */
        game_status += 1 << 22;
    }

    // Log game status
    // RCLCPP_INFO(logger(), "[BTFinisher]: Game status: " << game_status);
    cout << "[BTFinisher]: Game status: " << game_status << endl;

    int plant_number = 0; // Get by 2
    int pot_number = 0; // Get by 3
    int planter_number = 0; // Get by 5
    int solar_number_1 = 0; // Get by 7
    int solar_number_2 = 0; // Get by 8
    int got_home = 0; // Get by 9

    int tmp = game_status;

    std::string binary = bitset<32>(tmp).to_string();

    for (int i = 0; i < 32; i++) {
        if (tmp & 1) {

            // Append the binary representation
            binary[31 - i] = '1';

            if (i < 12) {
                plant_number++;
            }
            else if (i < 16) {
                pot_number++;
            }
            else if (i < 20) {
                planter_number++;
            }
            else if (i == 20) {
                solar_number_1++;
            }
            else if (i == 21) {
                solar_number_2++;
            }
            else if (i == 22) {
                got_home++;
            }
        }
        else {
            binary[31 - i] = '0';
        }
        tmp >>= 1;
    }
    double score = 0;

    // Log current plant number, pot number, planter number
    // RCLCPP_INFO(logger(), "[BTFinisher]: Plant number: " << plant_number << ", Pot number: " << pot_number << ", Planter number: " << planter_number);
    cout << "[BTFinisher]: Plant number: " << plant_number << ", Pot number: " << pot_number << ", Planter number: " << planter_number << endl;

    // Log the binary representation
    // RCLCPP_INFO(logger(), "[BTFinisher]: Binary representation: " << binary);
    cout << "[BTFinisher]: Binary representation: " << binary;

    if (pot_number == 1 && plant_number >= 1) {
        score += 12;
        plant_number--;
    }
    else if (pot_number == 2 && plant_number >= 1) {
        score += 11.5;
        plant_number -= 2;
    }
    else if (pot_number == 2 && plant_number >= 2) {
        score += 20 + 1;
        plant_number -= 2;
    }
    score += 4 * 3  * std::min(plant_number, planter_number);

    // Get solar degree from kernel
    std::vector<double> solar_degree;
    // kernel_->GetSolarDegree(solar_degree);

    // When we are blue:
    // 145~360
    // Total: 360-145=215

    // When we are yellow:
    // 180~360, 0~35
    // Total: 360-180+35=215

    // Test solar
    // score = 0;

    if (team_ == 0) {
        // Blue team
        RCLCPP_INFO(node_->get_logger(), "[BTFinisher]: Blue team");
        for (int i = 0; i < 3; i++) {
            if (solar_degree[i] >= 145) {
                score += 5;
            }
            else if (solar_degree[i] == -1 && solar_number_1 >= 1) {
                score += 5;
            }
        }
        for (int i = 3; i < (int)solar_degree.size(); i++) {
            if (solar_degree[i] >= 145) {
                score += 5;
            }
            else if (solar_degree[i] == -1 && solar_number_2 >= 1) {
                score += 5;
            }
        }
    }
    else {
        // Yellow team
        RCLCPP_INFO(node_->get_logger(), "[BTFinisher]: Yellow team");
        for (int i = 0; i < 3; i++) {
            if (solar_degree[i] != -1 && (solar_degree[i] >= 180 || solar_degree[i] <= 35)) {
                score += 5;
            }
            else if (solar_degree[i] == -1 && solar_number_1 >= 1) {
                score += 5;
            }
        }
        for (int i = 3; i < (int)solar_degree.size(); i++) {
            if (solar_degree[i] != -1 && (solar_degree[i] >= 180 || solar_degree[i] <= 35)) {
                score += 5;
            }
            else if (solar_degree[i] == -1 && solar_number_2 >= 1) {
                score += 5;
            }
        }
    }
    score += 20;

    if (got_home == 1) {
        score += 10;
    }

    // Log the score
    RCLCPP_INFO(node_->get_logger(), "[BTFinisher]: Current score: %f", score);

    // Log solar degree and solar number
    // RCLCPP_INFO(logger(), "[BTFinisher]: Solar degree: " << solar_degree[0] << " " << solar_degree[1] << " " << solar_degree[2] << " " << solar_degree[3] << " " << solar_degree[4]);
    // RCLCPP_INFO(logger(), "[BTFinisher]: Solar number: " << solar_number_1 << " " << solar_number_2);
    cout << "[BTFinisher]: Solar degree: " << solar_degree[0] << " " << solar_degree[1] << " " << solar_degree[2] << " " << solar_degree[3] << " " << solar_degree[4] << endl;
    cout << "[BTFinisher]: Solar number: " << solar_number_1 << " " << solar_number_2 << endl;

    // Degrade the score for mission plant
    // score *= 0.9;

    // Echo
    std::string cmd = "echo " + std::to_string(round(score)) + " > " + score_filepath;
    system(cmd.c_str());

    // Set output port
    setOutput<int>("result", game_status);

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList TimerChecker::providedPorts() {
    return { 
        BT::InputPort<int>("timer")
    };
}

BT::NodeStatus TimerChecker::tick() {

    int timer = getInput<int>("timer").value();

    double current_time;
    // kernel_->GetTime(current_time);

    // static first_log = true;

    // log
    // RCLCPP_INFO(logger(), "[TimerChecker]: Current time: " << current_time << ", Check Timeout: " << timer);
    if (first_log_) {
        // RCLCPP_INFO(logger(), "[TimerChecker]: Current time: " << current_time << ", Check Timeout: " << timer);
        cout << "[TimerChecker]: Current time: " << current_time << ", Check Timeout: " << timer << endl;
        first_log_ = false;
    }

    const BT::NodeStatus status = child_node_->executeTick();

    if (isStatusCompleted(status)) {
        resetChild();
    }

    if (current_time < timer) {
        
        switch (status) {
            case BT::NodeStatus::SUCCESS:
                return BT::NodeStatus::SUCCESS;
            case BT::NodeStatus::FAILURE:
                return BT::NodeStatus::FAILURE;
            case BT::NodeStatus::RUNNING:
                return BT::NodeStatus::RUNNING;
            default:
                return BT::NodeStatus::IDLE;
        }
    }
    else {

        // RCLCPP_ERROR(logger(), "[TimerChecker]: Timeout" << timer);
        cout << "[TimerChecker]: Timeout" << timer << endl;

        return BT::NodeStatus::FAILURE;
    }
}

// BT::NodeStatus LadybugActivate::tick() {
//     // if (this->kernel_ == nullptr) {
//     //     RCLCPP_ERROR(logger(), "[LadybugActivate]: Kernel is null");
//     //     return BT::NodeStatus::FAILURE;
//     // }

//     // kernel_->ActivateLadybug();

//     RCLCPP_ERROR(logger(), "[LadybugActivate]: Activate ladybug");

//     return BT::NodeStatus::SUCCESS;
// }

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
//             if (team_ == 0) {
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