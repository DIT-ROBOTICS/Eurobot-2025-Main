// BT 
#include "behaviortree_ros2/bt_action_node.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/decorators/loop_node.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/utils/shared_library.h"
// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"
// BTaction node
#include "rival_simulation/rival_navigation.h"
// C++
#include <memory>
#include <string>
#include <vector>

using namespace BT;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("rival_simulation");
    rclcpp::executors::MultiThreadedExecutor executor;
    auto blackboard = BT::Blackboard::create();

    // Parameters
    std::string groot_xml_config_directory;
    std::string bt_tree_node_model;
    std::string script_file_directory, tree_name;
    std::vector<double> material_points;

    // Read parameters
    node->declare_parameter<std::string>("script_file_directory", "/home/user/Eurobot-2025-Main/src/rival_simulation/rival_main_config/bt_rival_script.xml");
    node->declare_parameter<std::string>("tree_node_model_config_file", "/home/user/Eurobot-2025-Main/src/rival_simulation/rival_main_config/bt_m_tree_node_model.xml");
    node->declare_parameter<std::string>("tree_name", "MainTree");
    node->declare_parameter<std::string>("frame_id", "base_link");
    node->declare_parameter<std::vector<double>>("material_points", std::vector<double>{});
    node->declare_parameter<std::vector<double>>("mission_points", std::vector<double>{});
    node->get_parameter("script_file_directory", script_file_directory);
    node->get_parameter("tree_node_model_config_file", bt_tree_node_model);
    node->get_parameter("tree_name", tree_name);
    node->get_parameter("material_points", material_points);

    blackboard->set<std::vector<int>>("material_status", std::vector<int>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1});

    // Behavior Tree Factory
    BT::BehaviorTreeFactory factory;
    BT::RosNodeParams params;
    params.nh = node;
    // action nodes
    params.default_port_value = "navigate_to_pose";
    factory.registerNodeType<Navigation>("Navigation", params, blackboard);
    factory.registerNodeType<initPoints>("initPoints", node);
    factory.registerNodeType<StateUpdater>("StateUpdater", node, blackboard);
    factory.registerNodeType<PublishPose>("PublishPose", node, blackboard);

    // generate the tree in xml and safe the xml into a file
    std::string xml_models = BT::writeTreeNodesModelXML(factory);
    std::ofstream file(bt_tree_node_model);
    file << xml_models;
    file.close();

    factory.registerBehaviorTreeFromFile(script_file_directory);
    auto tree = factory.createTree(tree_name);

    BT::NodeStatus status = BT::NodeStatus::RUNNING;

    RCLCPP_INFO(node->get_logger(), "Rival BT start running!");

    rclcpp::Rate rate(100);
    executor.add_node(node);
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {        
        executor.spin_some();
        rate.sleep();
        status = tree.rootNode()->executeTick();
    }

    rclcpp::shutdown();
    return 0;
}
