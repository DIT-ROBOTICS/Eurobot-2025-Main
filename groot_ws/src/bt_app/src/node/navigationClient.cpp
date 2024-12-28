#pragma once

// Basic Libraries
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

// ROS related Libraries
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "btcpp_ros2_interfaces/action/navigation.hpp"

class TestNavClient : public rclcpp::Node
{
public:
  // Constructor
  explicit TestNavClient(const rclcpp::NodeOptions& options)
    : Node("test_nav_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<btcpp_ros2_interfaces::action::Navigation>(this, "nav_action");
    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&TestNavClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if(!this->client_ptr_->wait_for_action_server())
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = action::Navigation::Goal();
    goal_msg.goal.twist.linear.x = 1;
    goal_msg.goal.twist.linear.y = 1;
    goal_msg.goal.twist.angular.z = 0;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<btcpp_ros2_interfaces::action::Navigation>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&TestingActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&TestingActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&TestingActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<btcpp_ros2_interfaces::action::Navigation>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(const rclcpp_action::ClientGoalHandle<btcpp_ros2_interfaces::action::Navigation>::SharedPtr goal) {
    RCLCPP_INFO(this->get_logger(), "Goal accepted");
  }

  void feedback_callback(rclcpp_action::ClientGoalHandle<btcpp_ros2_interfaces::action::Navigation>::SharedPtr, const std::shared_ptr<const action::Navigation::Feedback> feedback) {

  }

  void result_callback(const rclcpp_action::ClientGoalHandle<btcpp_ros2_interfaces::action::Navigation>::WrappedResult & result) {
    RCLCPP_INFO(this->get_logger(), "Result received");
    rclcpp::shutdown();
  }
};

RCLCPP_COMPONENTS_REGISTER_NODE(TestNavClient)