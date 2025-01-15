#pragma once

// Basic Libraries
#include <filesystem>
#include <fstream>
#include <chrono>
#include <memory>
#include <string>

// ROS related Libraries
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "btcpp_ros2_interfaces/msg/localization.hpp"

// ROS Messages
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"

// TF2 Libraries
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/impl/utils.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"

class TestLocaClient : public rclcpp::Node
{
public:
  // Constructor
  explicit TestLocaClient(const rclcpp::NodeOptions& options)
    : Node("test_loca_client", options)
  {
    // Initialize ROS Publisher
    pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    // Initialize ROS Subscriber
    sub_loc_vel_ =
        this->create_subscriber("/odom", 1, &SimpleNavigation::locVelCallback, this);
    // Initialize ROS Timer (25HZ)
    timer_ = this->create_wall_timer(500ms, timerCallback);
  }

  // Destructor
  ~TestLocaClient()
  {
    // Do nothing
  }

  void setGoal(geometry_msgs::TwistStamped goal_position);

  void onHalted();

  std::vector<std::function<void()>> finish_callbacks_;

private:
  /* ROS Publisher */
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;

  /* ROS Subscriber */
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_loc_vel_;

  /* ROS Timer */
  rclcpp::TimerBase::SharedPtr timer_;

  /* ROS Callbacks */
  void locVelCallback(const nav_msgs::msg::Odometry::ConstPtr& msg);
  void timerCallback(const rclcpp::TimerEvent& event);

  /* ROS Items */
  geometry_msgs::msg::Twist cmd_vel_;
  nav_msgs::msg::Odometry loc_;
  geometry_msgs::msg::TwistStamped goal_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(TestLocaClient)