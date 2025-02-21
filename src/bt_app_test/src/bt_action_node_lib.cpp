#include "bt_app_test/bt_action_node_lib.h"

using namespace BT;

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


PortsList Testing::providedPorts() {
  return { 
    BT::InputPort<std::string>("input"),
    BT::OutputPort<std::string>("output") 
  };
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
    setOutput<std::string>("output", input);

    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void Testing::onHalted()
{
  tick_count = 0;

  // Reset the output port
  setOutput<std::string>("output", "hey");

  std::cout << "Testing Node halted" << std::endl;

  return;
}

PortsList TopicPubTest::providedPorts() {
  return { 
    BT::InputPort<std::string>("topic_name"), 
    BT::InputPort<int>("base_number"),
    BT::InputPort<int>("add_number"),
    BT::OutputPort<int>("total_number") 
  };
}

BT::NodeStatus TopicPubTest::onStart() {
  RCLCPP_INFO(node_->get_logger(), "Node start");
  getInput<int>("base_number", a);
  getInput<int>("add_number", b);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TopicPubTest::onRunning() {
  RCLCPP_INFO(node_->get_logger(), "Testing Node running");
  number.data = a + b;
  std::cout << a << " + " << b << " = " << number.data << std::endl;
  setOutput<int>("total_number", number.data); 
  return BT::NodeStatus::SUCCESS;
}

void TopicPubTest::onHalted() {
  // Reset the output port
  setOutput<int>("total_number", 0);
  RCLCPP_INFO(node_->get_logger(), "Testing Node halted");
  return;
}

PortsList TopicSubTest::providedPorts() {
  return { 
    BT::OutputPort<int>("output") 
  };
}

void TopicSubTest::topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {
  number = msg->data;
  RCLCPP_INFO(node_->get_logger(), "I heard: '%d'", msg->data);
}

BT::NodeStatus TopicSubTest::onStart() {
  RCLCPP_INFO(node_->get_logger(), "Node start");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TopicSubTest::onRunning() {
  setOutput<int>("output", number);
  RCLCPP_INFO(node_->get_logger(), "Testing Node running");
  return BT::NodeStatus::SUCCESS;
}

void TopicSubTest::onHalted() {
  // Reset the output port
  setOutput<int>("output", 0);
  RCLCPP_INFO(node_->get_logger(), "Testing Node halted");
  return;
}

PortsList StandardTopicPub::providedPorts() {
  return {
    BT::InputPort<std::string>("topic_name"), 
    BT::InputPort<int>("base_number"),
    BT::InputPort<int>("add_number"),
    BT::OutputPort<int>("total_number") 
  };
}

bool StandardTopicPub::setMessage(std_msgs::msg::Int32& msg) {
  int a = getInput<int>("base_number").value();
  int b = getInput<int>("add_number").value();
  msg.data = a + b;
  std::cout << a << " + " << b << " = " << msg.data << std::endl;
  setOutput<int>("total_number", msg.data); 
  return true;
}

PortsList StandardTopicSub::providedPorts() {
  return {
    BT::InputPort<std::string>("topic_name"), 
    BT::OutputPort<int>("output") 
  };
}
bool StandardTopicSub::latchLastMessage() const {
  return true;
}
NodeStatus StandardTopicSub::onTick(const std::shared_ptr<std_msgs::msg::Int32>& last_msg) {
  if (last_msg) { // Check if the pointer is not null
    std::cout << "Received data: " << last_msg->data << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  else {
    std::cout << "No message received!" << std::endl;
    return BT::NodeStatus::FAILURE;
  }
}

PortsList LocalizationTemp::providedPorts() {
  return { 
    BT::InputPort<std::string>("input"), 
    BT::OutputPort<geometry_msgs::msg::TwistStamped>("output") 
  };
}

BT::NodeStatus LocalizationTemp::onStart() {
  if(!getInput<std::string>("input", input)) {
    throw BT::RuntimeError("[Testing]: Missing required input: ", input);
  }

  return BT::NodeStatus::RUNNING;
}

void LocalizationTemp::UpdateRobotPose() {

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

BT::NodeStatus LocalizationTemp::onRunning() {
  UpdateRobotPose();
  std::cout << "robot_pose_:(" << robot_pose_.twist.linear.x << ", " << robot_pose_.twist.linear.y << ")\n";
  // Set the output port
  setOutput<geometry_msgs::msg::TwistStamped>("output", robot_pose_);

  return BT::NodeStatus::SUCCESS;
}

void LocalizationTemp::onHalted() {
  tick_count = 0;

  // Reset the output port
  setOutput<geometry_msgs::msg::TwistStamped>("output", robot_pose_);

  std::cout << "LocalizationTemp Node halted" << std::endl;

  return;
}

BT::PortsList NavigationTemp::providedPorts() {
  // return providedBasicPorts({ InputPort<unsigned>("order") });
  return providedBasicPorts({BT::InputPort<geometry_msgs::msg::TwistStamped>("goal")});
}

bool NavigationTemp::setGoal(RosActionNode::Goal& goal) {
  getInput<geometry_msgs::msg::TwistStamped>("goal", goal.goal);
  RCLCPP_INFO(logger(), "Sending Goal (%f, %f)", goal.goal.twist.linear.x, goal.goal.twist.linear.y);
  return true;
}

BT::NodeStatus NavigationTemp::onResultReceived(const WrappedResult& wr) {
  RCLCPP_INFO(logger(), "Result received");
  return NodeStatus::SUCCESS;
}

BT::NodeStatus NavigationTemp::onFailure(ActionNodeErrorCode error) {
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus NavigationTemp::onFeedback(const std::shared_ptr<const Feedback> feedback) {
  return BT::NodeStatus::RUNNING;
}

BT::PortsList BTMission::providedPorts() {
  return { 
      BT::InputPort<int>("order"),
      BT::InputPort<std::string>("action_name"),
      BT::OutputPort<std::deque<int>>("sequence")
  };
}
bool BTMission::setGoal(RosActionNode::Goal& goal) {
  order_ = getInput<int>("order").value();

  goal.order = order_;
  return true;
}
NodeStatus BTMission::onResultReceived(const WrappedResult& wr) {
  // while(wr.result->sequence)
  // sequence_ = wr.result->sequence;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "\n result: ");
  for (int i = 0; i < wr.result->sequence.size(); i++) {
    sequence_.push_back(wr.result->sequence[i]);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), sequence_[i] << ", ");
  }
  setOutput<std::deque<int>>("result", sequence_);
  return NodeStatus::SUCCESS;
}
NodeStatus BTMission::onFailure(ActionNodeErrorCode error) {
  setOutput<std::deque<int>>("result", sequence_);
  RCLCPP_ERROR(logger(), "[BT]: Navigation error");
  return NodeStatus::FAILURE;
}
NodeStatus BTMission::onFeedback(const std::shared_ptr<const Feedback> feedback) {
  for (int i = 0; i < feedback->partial_sequence.size(); i++) {
    partial_sequence_.push_back(feedback->partial_sequence[i]);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), partial_sequence_[i] << ", ");
  }
  partial_sequence_.clear();
  return NodeStatus::RUNNING;
}