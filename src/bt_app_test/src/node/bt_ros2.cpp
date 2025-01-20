#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

#include "bt_app_test/bt_action_node_lib.h"
#include "bt_app_test/bt_decorator_node_lib.h"
#include "bt_app_test/bt_utils_node_lib.h"

using namespace BT;
using namespace std;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  BehaviorTreeFactory factory;
  auto node = std::make_shared<rclcpp::Node>("bt_node");

  node->declare_parameter<string>("bt_tree_node_model", "/home/user/Eurobot-2025-Main-ws/src/bt_app_test/bt_config/"
                     "bt_tree_node_model.xml");
  node->declare_parameter<string>("groot_xml_config_directory", "/home/user/Eurobot-2025-Main-ws/src/bt_app_test/"
                                           "bt_config/");

  // Register the custom node
  // TODO: Add the custom node
  BT::RosNodeParams params;
  params.nh = node;
  factory.registerNodeType<Testing>("Testing");
  factory.registerNodeType<TopicSubTest>("TopicSubTest", node);
  params.default_port_value = "testing_action";
  factory.registerNodeType<NavigationTemp>("NavigationTemp", params);
  factory.registerNodeType<LocalizationTemp>("LocalizationTemp", node);
  factory.registerNodeType<TickFlow>("TickFlow");
  factory.registerNodeType<GeneratePathPoint>("GeneratePathPoint");
  factory.registerNodeType<BT::LoopNode<geometry_msgs::msg::TwistStamped>>("LoopWayPoint");
  factory.registerNodeType<ElapseTimeCheck>("ElapseTimeCheck");
  factory.registerNodeType<StartRace>("StartRace");
  factory.registerNodeType<RaceTimeCheck>("RaceTimeCheck");

  // Write tree nodes model
  std::string xml_tree_model = BT::writeTreeNodesModelXML(factory);
  std::string bt_tree_node_model = node->get_parameter("bt_tree_node_model").as_string();
  std::ofstream file(bt_tree_node_model);
  // std::ofstream file("/home/user/Eurobot-2025-Main-ws/src/bt_app_test/bt_config/"
  //                    "bt_tree_node_model.xml");
  file << xml_tree_model;
  file.close();

  // Create the tree from the XML file
  std::string groot_xml_config_directory = node->get_parameter("groot_xml_config_directory").as_string();
  // std::string groot_xml_config_directory = "/home/user/Eurobot-2025-Main-ws/src/bt_app_test/"
  //                                          "bt_config/";
  for(auto const& entry : std::filesystem::directory_iterator(groot_xml_config_directory))
  {
    if(entry.path().extension() == ".xml")
    {
      factory.registerBehaviorTreeFromFile(
          (groot_xml_config_directory + "/" + entry.path().filename().string()).c_str());
    }
  }

  // Create the tree
  // auto tree = factory.createTree("Waypoint-Demo");
  auto tree = factory.createTree("MainTree");

  BT::Groot2Publisher publisher(tree, 2227);

  rclcpp::Rate rate(1);
  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  while(rclcpp::ok() && status == BT::NodeStatus::RUNNING)
  {
    status = tree.rootNode()->executeTick();

    // Spinonce
    rclcpp::spin_some(node);
    rate.sleep();
  }
  return 0;
}
