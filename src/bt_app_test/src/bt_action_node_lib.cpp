#include "bt_app_test/bt_action_node_lib.h"

using namespace BT;

template <> inline geometry_msgs::msg::PoseStamped BT::convertFromString(StringView str) {

  auto parts = splitString(str, ',');
  if (parts.size() != 3) {
      throw RuntimeError("invalid input)");
  }
  else {
      geometry_msgs::msg::PoseStamped output;
      output.pose.position.x = convertFromString<double>(parts[0]);
      output.pose.position.y = convertFromString<double>(parts[1]);
      output.pose.position.z = convertFromString<double>(parts[2]);
      return output;
  }
}

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

double inline calculateDistance(const geometry_msgs::msg::Pose &pose1, const geometry_msgs::msg::Pose &pose2) {
  tf2::Vector3 position1(pose1.position.x, pose1.position.y, 0);
  tf2::Vector3 position2(pose2.position.x, pose2.position.y, 0);
  double dist = position1.distance(position2);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "distance: " << dist);
  return dist;
}

double inline calculateAngleDifference(const geometry_msgs::msg::Pose &pose1, const geometry_msgs::msg::Pose &pose2)
{
  tf2::Quaternion orientation1, orientation2;
  tf2::fromMsg(pose1.orientation, orientation1);
  tf2::fromMsg(pose2.orientation, orientation2);
  double yaw1 = tf2::impl::getYaw(orientation1);
  double yaw2 = tf2::impl::getYaw(orientation2);
  return std::fabs(yaw1 - yaw2);
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
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "size: " << feedback->partial_sequence.size());
  for (int i = 0; i < feedback->partial_sequence.size(); i++) {
    partial_sequence_.push_back(feedback->partial_sequence[i]);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), partial_sequence_[i] << ", ");
  }
  partial_sequence_.clear();
  return NodeStatus::RUNNING;
}

PortsList NavAction::providedPorts() {
  return { 
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("final_pose")
  };
}

BT::NodeStatus NavAction::onStart() {
  this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
    node_->get_node_base_interface(),
    node_->get_node_graph_interface(),
    node_->get_node_logging_interface(),
    node_->get_node_waitables_interface(),
    "navigate_to_pose"
  );

  this->timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&NavAction::send_goal, this)
  );
  return BT::NodeStatus::RUNNING;
}

void NavAction::send_goal()
{
  using namespace std::placeholders;
  this->timer_->cancel();
  this->goal_done_ = false;

  if (!this->client_ptr_) {
    RCLCPP_ERROR(node_->get_logger(), "Action client not initialized");
  }
  if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
    this->goal_done_ = true;
    return;
  }

  auto m = getInput<geometry_msgs::msg::PoseStamped>("goal");
  rclcpp::Time now = node_->now();
  goal_.header.stamp = now;
  goal_.header.frame_id = "map";
  goal_.pose.position.x = m.value().pose.position.x;
  goal_.pose.position.y = m.value().pose.position.y;
  tf2::Quaternion q;
  q.setRPY(0, 0, m.value().pose.position.z);
  goal_.pose.orientation.x = q.x();
  goal_.pose.orientation.y = q.y();
  goal_.pose.orientation.z = q.z();
  goal_.pose.orientation.w = q.w();

  RCLCPP_INFO(node_->get_logger(), "Start Nav (%f, %f)", goal_.pose.position.x, goal_.pose.position.y);
  nav_finished_ = false;
  
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose = goal_;

  RCLCPP_INFO(node_->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&NavAction::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&NavAction::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&NavAction::result_callback, this, _1);
  auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void NavAction::goal_response_callback(GoalHandleNavigation::SharedPtr goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void NavAction::feedback_callback(
  GoalHandleNavigation::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  current_pose_ = feedback->current_pose;
}

BT::NodeStatus NavAction::result_callback(const GoalHandleNavigation::WrappedResult & result)
{
  RCLCPP_INFO_STREAM(node_->get_logger(), "current_pose: " << current_pose_.pose.position.x << ", " << current_pose_.pose.position.y);
  this->goal_done_ = true;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "Result received");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
      return BT::NodeStatus::FAILURE;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
      return BT::NodeStatus::FAILURE;
    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
      return BT::NodeStatus::FAILURE;
  }
  // check if mission success
  if (calculateDistance(current_pose_.pose, goal_.pose) < 0.03 && calculateAngleDifference(current_pose_.pose, goal_.pose) < 0.4) {
    nav_finished_ = true;
    RCLCPP_INFO_STREAM(node_->get_logger(), "success! final_pose: " << current_pose_.pose.position.x << ", " << current_pose_.pose.position.y);
    setOutput<geometry_msgs::msg::PoseStamped>("final_pose", current_pose_);
    return NodeStatus::SUCCESS;
  } else {
    nav_error_ = true;
    RCLCPP_INFO_STREAM(node_->get_logger(), "fail! final_pose: " << current_pose_.pose.position.x << ", " << current_pose_.pose.position.y);
    setOutput<geometry_msgs::msg::PoseStamped>("final_pose", goal_);
    return NodeStatus::SUCCESS;
  }
}

BT::NodeStatus NavAction::onRunning() {
  if (nav_finished_) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "SUCCESS");
    return NodeStatus::SUCCESS;
  } else if (nav_error_) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "FAILURE");
    return NodeStatus::SUCCESS;
  } else {
    // RCLCPP_INFO_STREAM(node_->get_logger(), "RUNNING");
    return NodeStatus::RUNNING;
  }
}

void NavAction::onHalted() {

  return;
}

PortsList count_5::providedPorts() {
  return { 
    BT::InputPort<std::string>("input"),
    BT::OutputPort<std::string>("output") 
  };
}

BT::NodeStatus count_5::onStart()
{
  if(!getInput<std::string>("input", input))
  {
    throw BT::RuntimeError("[count_5]: Missing required input: ", input);
  }

  std::cout << "count_5 Node received input: " << input << std::endl;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus count_5::onRunning()
{
  tick_count++;

  std::cout << "count_5: " << tick_count << std::endl;

  if(tick_count == 5)
  {
    tick_count = 0;

    return BT::NodeStatus::SUCCESS;
  }
  else
    return BT::NodeStatus::RUNNING;
}

void count_5::onHalted()
{
  tick_count = 0;

  std::cout << "count_5 Node halted" << std::endl;

  return;
}

PortsList count_10::providedPorts() {
  return { 
    BT::InputPort<std::string>("input"),
    BT::OutputPort<std::string>("output") 
  };
}

BT::NodeStatus count_10::onStart()
{
  if(!getInput<std::string>("input", input))
  {
    throw BT::RuntimeError("[count_10]: Missing required input: ", input);
  }

  std::cout << "count_10 Node received input: " << input << std::endl;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus count_10::onRunning()
{
  tick_count++;

  std::cout << "count_10: " << tick_count << std::endl;

  if(tick_count == 10)
  {
    tick_count = 0;

    return BT::NodeStatus::SUCCESS;
  }
  else if(tick_count >= 6)
  {
    return BT::NodeStatus::FAILURE;
  }
  else
    return BT::NodeStatus::RUNNING;
}

void count_10::onHalted()
{
  tick_count = 0;

  std::cout << "count_10 Node halted" << std::endl;

  return;
}

PortsList count_15::providedPorts() {
  return { 
    BT::InputPort<std::string>("input"),
    BT::OutputPort<std::string>("output") 
  };
}

BT::NodeStatus count_15::onStart()
{
  if(!getInput<std::string>("input", input))
  {
    throw BT::RuntimeError("[count_15]: Missing required input: ", input);
  }

  std::cout << "count_15 Node received input: " << input << std::endl;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus count_15::onRunning()
{
  tick_count++;

  std::cout << "count_15: " << tick_count << std::endl;

  if(tick_count == 15)
  {
    tick_count = 0;

    return BT::NodeStatus::SUCCESS;
  }
  // else if(tick_count >= 11)
  //   return BT::NodeStatus::FAILURE;
  else
    return BT::NodeStatus::RUNNING;
}

void count_15::onHalted()
{
  tick_count = 0;

  std::cout << "count_15 Node halted" << std::endl;

  return;
}

PortsList Parallel_check::providedPorts() {
  return { 
    BT::InputPort<std::string>("input"),
    BT::OutputPort<std::string>("output") 
  };
}

BT::NodeStatus Parallel_check::onStart()
{
  if(!getInput<std::string>("input", input))
  {
    throw BT::RuntimeError("[Parallel_check]: Missing required input: ", input);
  }

  std::cout << "Parallel_check Node received input: " << input << std::endl;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Parallel_check::onRunning()
{
  std::cout << "Parallel_check" << std::endl;

  return BT::NodeStatus::SUCCESS;
}

void Parallel_check::onHalted()
{
  std::cout << "Parallel_check Node halted" << std::endl;

  return;
}