#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

#include "bt_app/bt_action_node_lib.h"
#include "bt_app/bt_decorator_node_lib.h"
#include "bt_app/bt_utils_node_lib.h"

using namespace BT;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  BehaviorTreeFactory factory;
  auto node = std::make_shared<rclcpp::Node>("bt_node");

  // Register the custom node
  // TODO: Add the custom node
  BT::RosNodeParams params;
  params.nh = node;
  params.default_port_value = "test";
  factory.registerNodeType<Testing>("Testing");
  factory.registerNodeType<SimpleNavigation>("SimpleNavigation", params);
  factory.registerNodeType<TickFlow>("TickFlow");
  factory.registerNodeType<GeneratePathPoint>("GeneratePathPoint");
  factory.registerNodeType<BT::LoopNode<geometry_msgs::msg::TwistStamped>>("LoopWayPoin"
                                                                           "t");
  factory.registerNodeType<ElapseTimeCheck>("ElapseTimeCheck");
  factory.registerNodeType<StartRace>("StartRace");
  factory.registerNodeType<RaceTimeCheck>("RaceTimeCheck");

  // Write tree nodes model
  std::string xml_tree_model = BT::writeTreeNodesModelXML(factory);
  std::ofstream file("/home/user/groot_ws/groot_ws/src/bt_app/bt_config/"
                     "bt_tree_node_model.xml");
  file << xml_tree_model;
  file.close();

  // Create the tree from the XML file
  std::string groot_xml_config_directory = "/home/user/groot_ws/groot_ws/src/bt_app/"
                                           "bt_config/";
  for(auto const& entry : std::filesystem::directory_iterator(groot_xml_config_directory))
  {
    if(entry.path().extension() == ".xml")
    {
      factory.registerBehaviorTreeFromFile(
          (groot_xml_config_directory + "/" + entry.path().filename().string()).c_str());
    }
  }

  // Create the tree
  auto tree = factory.createTree("Waypoint-Demo");

  // BT::Groot2Publisher publisher(tree, 2227);

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
