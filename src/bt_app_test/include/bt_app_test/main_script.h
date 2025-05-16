#include <chrono>
#include <cinttypes>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <vector>
#include <deque>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/impl/utils.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#define PI 3.1415926

class NavigateClient : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigation = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  NavigateClient(const geometry_msgs::msg::Pose& goal_pose)
  : Node("navigate_client"), goal_pose_(goal_pose), goal_done_(false)
  {
    this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "navigate_to_pose");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&NavigateClient::send_goal, this));
  }
  bool is_goal_done() const;
  void send_goal();

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;
  bool nav_finished_ = false;
  bool nav_error_ = false;
  geometry_msgs::msg::Pose goal_pose_;
  geometry_msgs::msg::PoseStamped goal_;
  geometry_msgs::msg::PoseStamped current_pose_;

  void goal_response_callback(GoalHandleNavigation::SharedPtr goal_handle);
  void feedback_callback(
    GoalHandleNavigation::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void result_callback(const GoalHandleNavigation::WrappedResult & result);

};  // class NavigateClient