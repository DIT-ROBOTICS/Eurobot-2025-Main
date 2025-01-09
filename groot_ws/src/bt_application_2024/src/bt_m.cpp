// BT 
#include "behaviortree_ros2/bt_action_node.hpp"

// #include "behaviortree_cpp/bt_factory.h"
// #include "behaviortree_cpp/decorators/loop_node.h"
#include "behaviortree_cpp/xml_parsing.h"
// #include "behaviortree_cpp/loggers/groot2_publisher.h"
// #include "behaviortree_cpp/decorators/loop_node.h"
// ROS
#include "rclcpp/rclcpp.hpp"
// #include "rclcpp/executors.hpp"
// ros message
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float32.hpp"
// BTaction nodes
#include "bt_application_2024/bt_nodes.h"
// C++
#include <memory>
#include <string>

using namespace BT;

// Function to send a boolean value to the service
bool sendBoolService(rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client, bool value, rclcpp::Node::SharedPtr node) {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = value;

    static bool success = false;

    if (success) return true;

    auto result_future = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
        auto result = result_future.get();
        success = true;
        return result->success;
    } else {
        RCLCPP_ERROR(node->get_logger(), "[BT Application]: Failed to call service");
        return false;
    }
}

double starting_time = 0.0;

void timeCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    starting_time = msg->data;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("bt_application_2024");

    // Parameters
    bool use_docking = false;
    bool mec_callback = true;
    std::string groot_xml_config_directory;
    std::string nav_action_name;
    std::string blue_filename, yellow_filename, score_filepath;
    double avoidance_distance;

    // Read parameters
    node->declare_parameter("groot_xml_config_directory", "/");
    node->declare_parameter("use_docking", false);
    node->declare_parameter("mec_callback", true);
    node->declare_parameter("nav_action_name", "/robot1/navigation_main");
    node->declare_parameter("groot_xml_blue_config_file", "bt_blue.xml");
    node->declare_parameter("groot_xml_yellow_config_file", "bt_yellow.xml");
    node->declare_parameter("score_filepath", "score.json");
    node->declare_parameter("avoidance_distance", 0.2);

    node->get_parameter("groot_xml_config_directory", groot_xml_config_directory);
    node->get_parameter("use_docking", use_docking);
    node->get_parameter("mec_callback", mec_callback);
    node->get_parameter("nav_action_name", nav_action_name);
    node->get_parameter("groot_xml_blue_config_file", blue_filename);
    node->get_parameter("groot_xml_yellow_config_file", yellow_filename);
    node->get_parameter("score_filepath", score_filepath);
    node->get_parameter("avoidance_distance", avoidance_distance);

    // Kernel
    // auto kernel = std::make_shared<Kernel>(node, nav_action_name);

    // Behavior Tree Factory
    BT::BehaviorTreeFactory factory;
    BT::RosNodeParams params;
    params.nh = node;
    // factory.registerNodeType<Comparator>("Comparator");
    factory.registerNodeType<Navigation>("Navigation", params);
    // factory.registerNodeType<BTMission>("BTMission", kernel, mec_callback);
    // factory.registerNodeType<Docking>("Docking", kernel, use_docking, mec_callback);
    // factory.registerNodeType<Recovery>("Recovery", kernel, use_docking, avoidance_distance);
    // factory.registerNodeType<BTMission>("BTMission");
    factory.registerNodeType<Docking>("Docking", params);
    factory.registerNodeType<Recovery>("Recovery", params);
    factory.registerNodeType<BTStarter>("BTStarter");
    factory.registerNodeType<TimerChecker>("TimerChecker");
    // factory.registerNodeType<LadybugActivate>("LadybugActivate");

    // Service Client
    auto client = node->create_client<std_srvs::srv::SetBool>("/robot/objects/ladybug_activate");
    // Subscriber
    auto time_sub = node->create_subscription<std_msgs::msg::Float32>("/robot/startup/time", 2, timeCallback);

    int team = 0;
    // To do: get team
    // factory.registerNodeType<RivalStart>("RivalStart", team);
    factory.registerNodeType<BTFinisher>("BTFinisher", score_filepath, team, node);

    std::string groot_filename;
    groot_xml_config_directory = "/home/user/groot_ws/groot_ws/src/bt_application_2024/bt_m_config/";
    // select tree
    if (team == 0) {
        groot_filename = groot_xml_config_directory + "/" + "bt_blue.xml";
        RCLCPP_INFO(node->get_logger(), "[BT Application]: Blue team is running!");
    } else {
        groot_filename = groot_xml_config_directory + "/" + "bt_yellow.xml";
        RCLCPP_INFO(node->get_logger(), "[BT Application]: Yellow team is running!");
    }

    // generate the tree in xml and safe the xml into a file
    std::string xml_models = BT::writeTreeNodesModelXML(factory);
    std::ofstream file("/home/user/groot_ws/groot_ws/src/bt_application_2024/bt_m_config/bt_tree_node_model.xml");
    file << xml_models;
    file.close();

    factory.registerBehaviorTreeFromFile(groot_filename);

    auto tree = factory.createTree("MainTree");

    BT::NodeStatus status = BT::NodeStatus::RUNNING;

    RCLCPP_INFO(node->get_logger(), "[BT Application]: Behavior Tree start running!");

    rclcpp::Rate rate(100);
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
        rclcpp::spin_some(node);
        rate.sleep();
        status = tree.rootNode()->executeTick();
    }

    rclcpp::shutdown();
    return 0;
}
