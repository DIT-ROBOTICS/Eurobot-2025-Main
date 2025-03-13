// BT 
#include "behaviortree_ros2/bt_action_node.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/decorators/loop_node.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/blackboard.h"
// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/client.hpp"  // Service client
#include "rclcpp/service.hpp" // Service server
// ros message
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
// Include startup necessary service
#include "btcpp_ros2_interfaces/srv/example.hpp"
// BTaction nodes
#include "bt_app_2025/bt_nodes_firmware.h"
#include "bt_app_2025/bt_nodes_navigation.h"
#include "bt_app_2025/bt_nodes_others.h"
#include "bt_app_2025/bt_nodes_receiver.h"
#include "bt_app_2025/bt_nodes_util.h"
// C++
#include <memory>
#include <string>

using namespace BT;

// Function to send a boolean value to the service
// bool sendBoolService(rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client, bool value, rclcpp::Node::SharedPtr node) {
//     auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
//     request->data = value;

//     static bool success = false;

//     if (success) return true;

//     auto result_future = client->async_send_request(request);

//     if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
//         auto result = result_future.get();
//         success = true;
//         return result->success;
//     } else {
//         RCLCPP_ERROR(node->get_logger(), "[BT Application]: Failed to call service");
//         return false;
//     }
// }

double game_time = 0.0;
std::string team = "0";
char plans = 'A';
bool isReady = false;

void timeCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    game_time = msg->data;
}

void readyCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    if (isReady)
        return;
    team = msg->header.frame_id;
    switch((int)msg->point.z) {
        case 1:
            plans = 'A';
            break;
        case 2:
            plans = 'B';
            break;
        case 3:
            plans = 'C';
            break;
        case 4:
            plans = 'S';
            break;
        default:
            plans = 'A';
            break;
    };
    isReady = true;
}

int main(int argc, char** argv) {
    // initialize
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("bt_app_2025");
    rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::Rate rate(100);

    // ROS msg
    std_msgs::msg::Int32 ready_feedback;

    // Subscriber
    auto time_sub = node->create_subscription<std_msgs::msg::Float32>("/robot/startup/time", 2, timeCallback);
    auto sub = node->create_subscription<geometry_msgs::msg::PointStamped>("/robot/startup/ready_signal", 2, readyCallback);
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub = node->create_publisher<std_msgs::msg::Int32>("/robot/Start", 2);

    // Behavior Tree Factory
    BT::BehaviorTreeFactory factory;
    BT::RosNodeParams params;
    params.nh = node;

    // Parameters
    std::string groot_xml_config_directory;
    std::string bt_tree_node_model;
    std::string Yellow_A_file, Yellow_B_file, Yellow_C_file, Yellow_Special_file, Blue_A_file, Blue_B_file, Blue_C_file, Blue_Special_file;
    std::string tree_name;

    // Create a shared blackboard
    auto blackboard = BT::Blackboard::create();
    blackboard->set<double>("current_time", 0);
    blackboard->set<int>("mission_progress", 0);

    // Read parameters
    node->declare_parameter<std::string>("groot_xml_config_directory", "/home/user/Eurobot-2025-Main-ws/src/bt_app_2025/bt_m_config/");
    node->declare_parameter<std::string>("tree_node_model_config_file", "/home/user/Eurobot-2025-Main-ws/src/bt_app_2025/bt_m_config/bt_m_tree_node_model.xml");
    node->declare_parameter<std::string>("tree_name", "FuncTest");
    node->declare_parameter<std::string>("planA_Yellow_config", "bt_plan_a_Yellow.xml");
    node->declare_parameter<std::string>("planB_Yellow_config", "bt_plan_b_Yellow.xml");
    node->declare_parameter<std::string>("planC_Yellow_config", "bt_plan_c_Yellow.xml");
    node->declare_parameter<std::string>("Special_Yellow_config", "bt_plan_a_Yellow.xml");
    node->declare_parameter<std::string>("planA_Blue_config", "bt_plan_a_Blue.xml");
    node->declare_parameter<std::string>("planB_Blue_config", "bt_plan_b_Blue.xml");
    node->declare_parameter<std::string>("planC_Blue_config", "bt_plan_c_Blue.xml");
    node->declare_parameter<std::string>("Special_Blue_config", "bt_plan_a_Blue.xml");
    // get parameters
    node->get_parameter("groot_xml_config_directory", groot_xml_config_directory);
    node->get_parameter("tree_node_model_config_file", bt_tree_node_model);
    node->get_parameter("tree_name", tree_name);
    node->get_parameter("planA_Yellow_config", Yellow_A_file);
    node->get_parameter("planB_Yellow_config", Yellow_B_file);
    node->get_parameter("planC_Yellow_config", Yellow_C_file);
    node->get_parameter("Special_Yellow_config", Yellow_Special_file);
    node->get_parameter("planA_Blue_config", Blue_A_file);
    node->get_parameter("planB_Blue_config", Blue_B_file);
    node->get_parameter("planC_Blue_config", Blue_C_file);
    node->get_parameter("Special_Blue_config", Blue_Special_file);

    // action nodes
    /* receiver */
    factory.registerNodeType<NavReceiver>("NavReceiver", node, blackboard);
    factory.registerNodeType<CamReceiver>("CamReceiver", node, blackboard);
    /* navigation */
    factory.registerNodeType<DynamicAdjustment>("DynamicAdjustment");
    params.default_port_value = "navigate_to_pose";
    factory.registerNodeType<Navigation>("Navigation", params);
    factory.registerNodeType<Rotation>("Rotation", params);
    params.default_port_value = "dock_robot";
    factory.registerNodeType<Docking>("Docking", params);
    // /* firmware */
    params.default_port_value = "firmware_mission";
    factory.registerNodeType<FirmwareMission>("FirmwareMission", node, blackboard);
    factory.registerNodeType<IntegratedMissionNode>("IntegratedMissionNode", node, blackboard);
    factory.registerNodeType<SIMAactivate>("SIMAactivate", node);
    /* others */
    factory.registerNodeType<BTStarter>("BTStarter", node, blackboard);
    factory.registerNodeType<Comparator>("Comparator", node); // decorator
    factory.registerNodeType<TimerChecker>("TimerChecker", blackboard); // decorator

    // generate the tree in xml and safe the xml into a file
    std::string xml_models = BT::writeTreeNodesModelXML(factory);
    std::ofstream file(bt_tree_node_model);
    file << xml_models;
    file.close();

    // Receiving team number and plan code
    while (rclcpp::ok() && !isReady) {
        rclcpp::spin_some(node);
        rclcpp::Rate rate(100);
    }
    // Send feed back to startup
    for (int j = 0; j < 20; j++) {
        ready_feedback.data = 1;
        pub->publish(ready_feedback);
        rclcpp::Rate rate(100);
    }
    // select tree
    std::string groot_filename;
    if (team == "0") {
        switch (plans) {
            case 'A':
            groot_filename = groot_xml_config_directory + "/" + Yellow_A_file;
            RCLCPP_INFO(node->get_logger(), "[BT Application]: Yellow team is running plan A!");
            break;
            case 'B':
            groot_filename = groot_xml_config_directory + "/" + Yellow_B_file;
            RCLCPP_INFO(node->get_logger(), "[BT Application]: Yellow team is running plan B!");
            break;
            case 'C':
            groot_filename = groot_xml_config_directory + "/" + Yellow_C_file;
            RCLCPP_INFO(node->get_logger(), "[BT Application]: Yellow team is running plan C!");
            break;
            case 'S':
            groot_filename = groot_xml_config_directory + "/" + Yellow_Special_file;
            RCLCPP_INFO(node->get_logger(), "[BT Application]: Yellow team is running Special plan!");
            break;
            default:
            throw "False or Empty plan file";
        }
    } else {
        RCLCPP_INFO(node->get_logger(), "[BT Application]: Blue team is running!");
        switch (plans) {
            case 'A':
            groot_filename = groot_xml_config_directory + "/" + Blue_A_file;
            break;
            case 'B':
            groot_filename = groot_xml_config_directory + "/" + Blue_B_file;
            break;
            case 'C':
            groot_filename = groot_xml_config_directory + "/" + Blue_C_file;
            break;
            case 'S':
            groot_filename = groot_xml_config_directory + "/" + Blue_Special_file;
            break;
            default:
            throw "False or Empty plan file";
        }
    }
    factory.registerBehaviorTreeFromFile(groot_filename);

    auto tree = factory.createTree(tree_name, blackboard);
    BT::Groot2Publisher publisher(tree, 2227);

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    RCLCPP_INFO(node->get_logger(), "[BT Application]: Behavior Tree start running!");
    // executor.add_node(node);
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING /* && game_time <= 95 */) {
        // executor.spin_some();
        rclcpp::spin_some(node);
        rate.sleep();
        status = tree.rootNode()->executeTick();
    }

    rclcpp::shutdown();
    return 0;
}
