#pragma once

// Use C++ libraries
#include <filesystem>
#include <fstream>
#include <deque>
#include <bitset>
#include <vector>

// Use behavior tree
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>

// Use ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

// Use ros message
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// tf2 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/impl/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

using namespace BT;
using namespace std;

namespace BT {
    template <> inline geometry_msgs::msg::TwistStamped convertFromString(StringView str);
    template <> inline int convertFromString(StringView str);
    template <> inline std::deque<int> convertFromString(StringView str);
}

namespace MATH {
    double inline calculateDistance(const geometry_msgs::msg::Pose &pose1, const geometry_msgs::msg::Pose &pose2);
    double inline calculateAngleDifference(const geometry_msgs::msg::Pose &pose1, const geometry_msgs::msg::Pose &pose2);
    geometry_msgs::msg::PoseStamped inline ConvertPoseFormat(geometry_msgs::msg::PoseStamped &pose_);
}