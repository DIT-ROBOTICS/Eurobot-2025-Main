// BT 
#include "behaviortree_ros2/bt_action_node.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/decorators/loop_node.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"
// ros message
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float32.hpp"
// BTaction nodes
#include "bt_app_2025/bt_nodes_firmware.h"
#include "bt_app_2025/bt_nodes_navigation.h"
#include "bt_app_2025/bt_nodes_others.h"
#include "bt_app_2025/bt_nodes_receiver.h"
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
    auto node = std::make_shared<rclcpp::Node>("bt_app_2025");
    rclcpp::executors::MultiThreadedExecutor executor;

    // Parameters
    std::string groot_xml_config_directory;
    std::string bt_tree_node_model;
    std::string blue_filename, yellow_filename, planA_filename, score_filepath;

    // Read parameters
    node->declare_parameter<std::string>("groot_xml_config_directory", "/home/user/Eurobot-2025-Main-ws/src/bt_app_2025/bt_m_config/");
    node->declare_parameter<std::string>("tree_node_model_config_file", "/home/user/Eurobot-2025-Main-ws/src/bt_app_2025/bt_m_config/bt_m_tree_node_model.xml");
    node->declare_parameter("groot_xml_blue_config_file", "bt_blue.xml");
    node->declare_parameter("groot_xml_yellow_config_file", "bt_yellow.xml");
    node->declare_parameter<std::string>("planA_config_file", "bt_plan_a.xml");
    node->declare_parameter("score_filepath", "score.json");

    node->get_parameter("groot_xml_config_directory", groot_xml_config_directory);
    node->get_parameter("tree_node_model_config_file", bt_tree_node_model);
    node->get_parameter("groot_xml_blue_config_file", blue_filename);
    node->get_parameter("groot_xml_yellow_config_file", yellow_filename);
    node->get_parameter("planA_config_file", planA_filename);
    node->get_parameter("score_filepath", score_filepath);

    // Create a shared blackboard
    auto blackboard = BT::Blackboard::create();
    // Store a persistent parameter
    blackboard->set<std::string>("global_param", "Persistent Value");

    // Behavior Tree Factory
    BT::BehaviorTreeFactory factory;
    BT::RosNodeParams params;
    int team = 0;
    params.nh = node;
    // action nodes
    /* receiver */
    factory.registerNodeType<LocReceiver>("LocReceiver", node);
    factory.registerNodeType<NavReceiver>("NavReceiver", node);
    factory.registerNodeType<CamReceiver>("CamReceiver", node);
    /* navigation */
    // factory.registerNodeType<DynamicAdjustment>("DynamicAdjustment");
    params.default_port_value = "navigate_to_pose";
    factory.registerNodeType<Navigation>("Navigation", params);
    factory.registerNodeType<Docking>("Docking", params);
    // /* firmware */
    params.default_port_value = "firmware_mission";
    factory.registerNodeType<BTMission>("BTMission", params);
    factory.registerNodeType<SIMAactivate>("SIMAactivate", node);
    // factory.registerNodeType<BannerMission>("BannerMission", params);
    factory.registerNodeType<ConstructFinisher>("ConstructFinisher");
    factory.registerNodeType<CollectFinisher>("CollectFinisher");
    /* others */
    factory.registerNodeType<BTStarter>("BTStarter", node);
    // factory.registerNodeType<BTFinisher>("BTFinisher", score_filepath, team, node);
    factory.registerNodeType<Comparator>("Comparator", node); // decorator
    factory.registerNodeType<TimerChecker>("TimerChecker"); // decorator
    // factory.registerNodeType<RivalStart>("RivalStart", team);
    factory.registerNodeType<PointProvider>("PointProvider");

    // Service Client
    auto client = node->create_client<std_srvs::srv::SetBool>("/robot/objects/ladybug_activate");
    // Subscriber
    auto time_sub = node->create_subscription<std_msgs::msg::Float32>("/robot/startup/time", 2, timeCallback);

    // generate the tree in xml and safe the xml into a file
    std::string xml_models = BT::writeTreeNodesModelXML(factory);
    std::ofstream file(bt_tree_node_model);
    // std::ofstream file("/home/user/groot_ws/groot_ws/src/bt_app_2025/bt_m_config/bt_tree_node_model.xml");
    file << xml_models;
    file.close();

    std::string groot_filename;
    // select tree
    // if (team == 0) {
    //     groot_filename = groot_xml_config_directory + "/" + "bt_blue.xml";
    //     RCLCPP_INFO(node->get_logger(), "[BT Application]: Blue team is running!");
    // } else {
    //     groot_filename = groot_xml_config_directory + "/" + "bt_yellow.xml";
    //     RCLCPP_INFO(node->get_logger(), "[BT Application]: Yellow team is running!");
    // }
    groot_filename = groot_xml_config_directory + "/" + planA_filename;
    factory.registerBehaviorTreeFromFile(groot_filename);

    auto tree = factory.createTree("MainTree");
    // auto tree = factory.createTree("NavTest");
    BT::Groot2Publisher publisher(tree, 2227);

    BT::NodeStatus status = BT::NodeStatus::RUNNING;

    RCLCPP_INFO(node->get_logger(), "[BT Application]: Behavior Tree start running!");
    
    rclcpp::Rate rate(100);
    executor.add_node(node);
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
        // rclcpp::spin_some(node);
        executor.spin_some();
        rate.sleep();
        status = tree.rootNode()->executeTick();
    }

    rclcpp::shutdown();
    return 0;
}
