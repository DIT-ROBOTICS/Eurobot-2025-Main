#include "bt_app/bt_action_node_lib.h"

using namespace BT;

template <> inline geometry_msgs::Twist BT::convertFromString(StringView str) {

  auto parts = splitString(str, ',');
  if (parts.size() != 3) {
    throw RuntimeError("invalid input)");
  }
  else {
    // Testing function -> Only remain x and y
    geometry_msgs::msg::Twist output;
    output.linear.x = convertFromString<double>(parts[0]);
    output.linear.y = convertFromString<double>(parts[1]);
    return output;
  }
}

template <> inline int BT::convertFromString(StringView str) {
    auto value = convertFromString<double>(str);
    return (int) value;
}

BT::PortsList Testing::providedPorts()
{
  return { BT::InputPort<std::string>("input"), BT::OutputPort<std::string>("output") };
}

BT::NodeStatus Testing::onStart()
{
  if(!getInput<std::string>("input", input))
  {
    throw BT::RuntimeError("[Testing]: Missing required input: ", input);
  }

  std::cout << "Testing Node received input: " << input << std::endl;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Testing::onRunning()
{
  tick_count++;

  // Log the current count
  std::cout << "Testing: " << tick_count << std::endl;

  if(tick_count == 10)
  {
    tick_count = 0;

    // Set the output port
    setOutput("output", input);

    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void Testing::onHalted()
{
  tick_count = 0;

  // Reset the output port
  setOutput("output", "hey");

  std::cout << "Testing Node halted" << std::endl;

  return;
}

PortsList NavigationTemp::providedPorts()
{
  // return providedBasicPorts({ InputPort<unsigned>("order") });
  return { 
    BT::InputPort<geometry_msgs::msg::Twist>("goal")
  };
}

bool NavigationTemp::setGoal(RosActionNode::Goal& goal)
{
  getInput<geometry_msgs::msg::Twist>("goal", goal.goal);
  RCLCPP_INFO(logger(), "Sending Goal");
  return true;
}

NodeStatus NavigationTemp::onResultReceived(const WrappedResult& wr)
{
  RCLCPP_INFO(logger(), "Result received");
  return NodeStatus::SUCCESS;
}

NodeStatus NavigationTemp::onFailure(ActionNodeErrorCode error)
{
  return NodeStatus::FAILURE;
}

NodeStatus NavigationTemp::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  return NodeStatus::RUNNING;
}