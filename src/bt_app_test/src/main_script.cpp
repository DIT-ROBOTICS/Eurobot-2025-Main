#include "bt_app_test/main_script.h"

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

bool NavigateClient::is_goal_done() const
{
  return this->goal_done_;
}

void NavigateClient::send_goal()
{
  using namespace std::placeholders;

  this->timer_->cancel();

  this->goal_done_ = false;

  if (!this->client_ptr_) {
    RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
  }

  if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    this->goal_done_ = true;
    return;
  }

  rclcpp::Time now = this->now();
  goal_.header.stamp = now;
  goal_.header.frame_id = "map";
  goal_.pose = goal_pose_;

  RCLCPP_INFO(this->get_logger(), "Start Nav (%f, %f)", goal_.pose.position.x, goal_.pose.position.y);
  nav_finished_ = false;
  
  auto goal_msg = NavigateToPose::Goal();
  goal_msg.pose = goal_;

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&NavigateClient::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&NavigateClient::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&NavigateClient::result_callback, this, _1);
  auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void NavigateClient::goal_response_callback(GoalHandleNavigation::SharedPtr goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void NavigateClient::feedback_callback(
  GoalHandleNavigation::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  current_pose_ = feedback->current_pose;
}

void NavigateClient::result_callback(const GoalHandleNavigation::WrappedResult & result)
{
  this->goal_done_ = true;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Result received");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }
  // check if mission success
  if (calculateDistance(current_pose_.pose, goal_.pose) < 0.03 && calculateAngleDifference(current_pose_.pose, goal_.pose) < 0.4) {
    nav_finished_ = true;
    RCLCPP_INFO_STREAM(this->get_logger(), "success! final_pose: " << current_pose_.pose.position.x << ", " << current_pose_.pose.position.y);
    return;
  } else {
    nav_error_ = true;
    RCLCPP_INFO_STREAM(this->get_logger(), "fail! final_pose: " << current_pose_.pose.position.x << ", " << current_pose_.pose.position.y);
    return;
  }
}

std::deque<geometry_msgs::msg::Pose> script_pt_queue;

void initialize(std::shared_ptr<rclcpp::Node> node_) {
    std::vector<std::string> points_name_ = {"pt0", "pt1", "pt2", "pt3", "pt4", "pt5", "pt6", "pt7", "pt8", "pt9"};
    node_->declare_parameter<std::vector<double>>(points_name_[0], {2.5, 1.3, 0.0, 0.0, 0.0, 0.707, 0.707});
    node_->declare_parameter<std::vector<double>>(points_name_[1], {2.6, 1.3, 0.0, 0.0, 0.0, 0.707, 0.707});
    node_->declare_parameter<std::vector<double>>(points_name_[2], {2.5, 0.4, 0.0, 0.0, 0.0, 0.707, 0.707});
    node_->declare_parameter<std::vector<double>>(points_name_[3], {2.6, 0.4, 0.0, 0.0, 0.0, 0.707, 0.707});
    node_->declare_parameter<std::vector<double>>(points_name_[4], {2.5, 0.4, 0.0, 0.0, 0.0, -0.707, 0.707});
    node_->declare_parameter<std::vector<double>>(points_name_[5], {2.6, 0.4, 0.0, 0.0, 0.0, -0.707, 0.707});
    node_->declare_parameter<std::vector<double>>(points_name_[6], {2.5, 0.4, 0.0, 0.0, 0.0, -0.707, 0.707});
    node_->declare_parameter<std::vector<double>>(points_name_[7], {2.6, 0.9, 0.0, 0.0, 0.0, -0.707, 0.707});
    node_->declare_parameter<std::vector<double>>(points_name_[8], {2.5, 0.9, 0.0, 0.0, 0.0, -0.707, 0.707});
    node_->declare_parameter<std::vector<double>>(points_name_[9], {2.6, 0.9, 0.0, 0.0, 0.0, -0.707, 0.707});
    for (const auto &point_name_ : points_name_) {
        std::vector<double> param;
        node_->get_parameter(point_name_, param);
        geometry_msgs::msg::Pose path_point;
        path_point.position.x = param[0];
        path_point.position.y = param[1];
        path_point.position.z = param[2];
        path_point.orientation.x = param[3];
        path_point.orientation.y = param[4];
        path_point.orientation.z = param[5];
        path_point.orientation.w = param[6];
        // Store in shared_queue
        script_pt_queue.push_back(path_point);
    }
    RCLCPP_INFO(rclcpp::get_logger("main_script"), "Init points");
}
  
int main(int argc, char ** argv)
{
  rclcpp::Rate rate(100);

  rclcpp::init(argc, argv);

  auto node_ = rclcpp::Node::make_shared("main_script");

  initialize(node_);
  while (!script_pt_queue.empty()) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("main_script"), "script_pt_queue: " << script_pt_queue.front().position.x << ", " << script_pt_queue.front().position.y);
    auto nav_client = std::make_shared<NavigateClient>(script_pt_queue.front());
    script_pt_queue.pop_front();
    while (!nav_client->is_goal_done()) {
      rclcpp::spin_some(nav_client);
    }
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}