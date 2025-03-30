#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/decorators/loop_node.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/blackboard.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

#include "bt_app_test/bt_action_node_lib.h"
#include "bt_app_test/bt_decorator_node_lib.h"
#include "bt_app_test/bt_utils_node_lib.h"

using namespace BT;
using namespace std;

int main(int argc, char** argv)
{
  std::string bt_tree_node_model_, groot_xml_config_directory_, tree_name_;
  BehaviorTreeFactory factory;

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bt_node");

  node->declare_parameter<string>("bt_tree_node_model", "/home/ros/Eurobot-2025-Main/src/bt_app_test/bt_config/bt_tree_node_model.xml");
  node->declare_parameter<string>("groot_xml_config_directory", "/home/ros/Eurobot-2025-Main/src/bt_app_test/bt_config/");
  node->declare_parameter<std::string>("tree_name", "MainTree");
  node->get_parameter("bt_tree_node_model", bt_tree_node_model_);
  node->get_parameter("groot_xml_config_directory", groot_xml_config_directory_);
  node->get_parameter("tree_name", tree_name_);

  // Register the custom node
  // TODO: Add the custom node
  BT::RosNodeParams params;
  params.nh = node;
  factory.registerNodeType<Testing>("Testing");
  factory.registerNodeType<TopicSubTest>("TopicSubTest", node);
  factory.registerNodeType<TopicPubTest>("TopicPubTest", node);
  params.default_port_value = "number";
  factory.registerNodeType<StandardTopicPub>("StandardTopicPub", params);
  params.default_port_value = "number";
  factory.registerNodeType<StandardTopicSub>("StandardTopicSub", params);
  params.default_port_value = "testing_action";
  factory.registerNodeType<NavigationTemp>("NavigationTemp", params);
  factory.registerNodeType<LocalizationTemp>("LocalizationTemp", node);
  factory.registerNodeType<TickFlow>("TickFlow");
  factory.registerNodeType<GeneratePathPoint>("GeneratePathPoint");
  factory.registerNodeType<BT::LoopNode<geometry_msgs::msg::TwistStamped>>("LoopWayPoint");
  factory.registerNodeType<ElapseTimeCheck>("ElapseTimeCheck");
  factory.registerNodeType<StartRace>("StartRace");
  factory.registerNodeType<RaceTimeCheck>("RaceTimeCheck");
  params.default_port_value = "fibonacci";
  factory.registerNodeType<BTMission>("BTMission", params);
  factory.registerNodeType<NavAction>("NavAction", node);
  factory.registerNodeType<count_5>("count_5");
  factory.registerNodeType<count_10>("count_10");
  factory.registerNodeType<count_15>("count_15");
  factory.registerNodeType<Parallel_check>("Parallel_check");

  // Write tree nodes model
  std::string xml_tree_model = BT::writeTreeNodesModelXML(factory);
  std::ofstream file(bt_tree_node_model_);
  file << xml_tree_model;
  file.close();

  // Create the tree from the XML file
  for(auto const& entry : std::filesystem::directory_iterator(groot_xml_config_directory_))
  {
    if(entry.path().extension() == ".xml")
    {
      factory.registerBehaviorTreeFromFile(
          (groot_xml_config_directory_ + "/" + entry.path().filename().string()).c_str());
    }
  }

  // Create the tree
  auto tree = factory.createTree(tree_name_);

  BT::Groot2Publisher publisher(tree, 2227);

  rclcpp::Rate rate(100);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  RCLCPP_INFO(node->get_logger(), "[BT Application]: Behavior Tree start running!");

  while(rclcpp::ok() && status == BT::NodeStatus::RUNNING)
  {
    status = tree.rootNode()->executeTick();
    // Spinonce
    rclcpp::spin_some(node);
    rate.sleep();
  }
  return 0;
}