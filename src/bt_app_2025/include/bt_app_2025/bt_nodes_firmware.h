#pragma once

// Use C++ libraries
#include <filesystem>
#include <fstream>
#include <deque>
#include <bitset>
#include <vector>
#include <math.h> 

// Use behavior tree
#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>

// Use ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

// Use ros message
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

// tf2 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

// Use action message
#include "btcpp_ros2_interfaces/action/firmware_mission.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace BT;

namespace BT {
    template <> inline geometry_msgs::msg::PoseStamped convertFromString(StringView str);
    template <> inline int convertFromString(StringView str);
    template <> inline std::deque<int> convertFromString(StringView str);
    template <> inline std::deque<double> convertFromString(StringView str);
}

/********************************/
/* Simple Node to activate SIMA */
/********************************/
class SIMAactivate : public BT::SyncActionNode {
public:
  SIMAactivate(const std::string& name, const BT::NodeConfig& config, const RosNodeParams& params)
    : BT::SyncActionNode(name, config), node_(params.nh.lock())
  {}

  /* Node remapping function */
  static BT::PortsList providedPorts() {
    return {}; 
  }

  /* Start and running function */
  BT::NodeStatus tick() override;
private:
  bool wakeUpSIMA();
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::TimerBase::SharedPtr timer_;
  float current_time_;
  bool mission_finished_ = false;
};

class MissionStart : public BT::SyncActionNode
{
public:

  MissionStart(const std::string& name, const BT::NodeConfig& config, const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
    : BT::SyncActionNode(name, config), node_(params.nh.lock()), blackboard_(blackboard)
  {}
  /* Node remapping function */
  static BT::PortsList providedPorts();
  /* Start and running function */
  BT::NodeStatus tick() override;

private:
  BT::Blackboard::Ptr blackboard_;
  rclcpp::Node::SharedPtr node_;
  std::vector<double> material_points_;
};

class MissionSuccess : public BT::SyncActionNode
{
public:

  MissionSuccess(const std::string& name, const BT::NodeConfig& config, const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
    : BT::SyncActionNode(name, config), node_(params.nh.lock()), blackboard_(blackboard)
  {
    publish_times = 100;
    publish_count = 0;
    vision_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/main/mission/success", rclcpp::QoS(100).reliable().transient_local());
  }
  /* Node remapping function */
  static BT::PortsList providedPorts();
  /* Start and running function */
  BT::NodeStatus tick() override;
  void timer_publisher();

private:
  BT::Blackboard::Ptr blackboard_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr vision_pub_;
  std_msgs::msg::Bool is_mission_finished;
  std_msgs::msg::Int32 mission_finished;
  std_msgs::msg::Int32MultiArray mission_points_status_;
  std_msgs::msg::Int32MultiArray materials_info_;
  int publish_times;
  int publish_count;
  int front_materials_;
  int back_materials_;
};

class MissionFailure : public BT::StatefulActionNode
{
public:

  MissionFailure(const std::string& name, const BT::NodeConfig& config, const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
    : BT::StatefulActionNode(name, config), node_(params.nh.lock()), blackboard_(blackboard)
  {
    matreials_accord_ = 0;
  }

  /* Node remapping function */
  static BT::PortsList providedPorts();

  /* Start and running function */
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;

  /* Halt function */
  void onHalted() override;

private:
  BT::Blackboard::Ptr blackboard_;
  // step_results_-> 1: front_collect 2: back_collect 3: construct_1 4: construct_2 5: construct_3
  std::deque<int> step_results_;
  // obot_type_-> 0: SpinArm 1: NotSpinArm
  std_msgs::msg::Int32MultiArray mission_points_status_;
  bool robot_type_;
  bool matreials_accord_;
  int success_levels_;
  int failed_levels_;
  int mission_progress_;
  int front_materials_;
  int back_materials_;
  int base_index_;

  std::deque<int> stage_info_;

  rclcpp::Node::SharedPtr node_;
};

/****************/
/* Topic Mission*/  
/****************/

class FirmwareMission : public BT::StatefulActionNode
{
public:
  FirmwareMission(const std::string& name, const BT::NodeConfig& config, const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
    : BT::StatefulActionNode(name, config), node_(params.nh.lock()), blackboard_(blackboard), rate_(30) {
    publisher_ = node_->create_publisher<std_msgs::msg::Int32>("mission_type", 10);
    subscription_ = node_->create_subscription<std_msgs::msg::Int32>("mission_status", 10, std::bind(&FirmwareMission::mission_callback, this, std::placeholders::_1));
  }

  /* Node remapping function */
  static BT::PortsList providedPorts();

  /* Start and running function */
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;

  void mission_callback(const std_msgs::msg::Int32::SharedPtr sub_msg);
  BT::NodeStatus stopStep();
  /* Halt function */
  void onHalted() override;

private:
  std::shared_ptr<rclcpp::Node> node_;
  BT::Blackboard::Ptr blackboard_;
  rclcpp::Rate rate_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;

  std_msgs::msg::Int32 pub_msg;
  int mission_progress_ = 0;
  int mission_type_ = 0;
  int mission_status_ = 0;
  int mission_stamp_ = 0;
  bool mission_received_ = false;
  bool mission_finished_ = false;
};

class BannerChecker : public BT::SyncActionNode
{
public:
    BannerChecker(const std::string &name, const BT::NodeConfig &config, const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
        : BT::SyncActionNode(name, config), node_(params.nh.lock()), blackboard_(blackboard), tf_buffer_(params.nh.lock()->get_clock()), listener_(tf_buffer_)
    {
        node_->get_parameter("frame_id", frame_id_);
        tf_buffer_.setUsingDedicatedThread(true);
    }
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
private:
    void onStart();
    int DecodeBannerIndex(const int index);
    BT::Blackboard::Ptr blackboard_;
    rclcpp::Node::SharedPtr node_;
    // for tf listener
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener listener_;
    std::string frame_id_;
    geometry_msgs::msg::PoseStamped robot_pose_;
    geometry_msgs::msg::PoseStamped rival_pose_;
    // for input
    std_msgs::msg::Int32MultiArray mission_points_status_;
    std::vector<double> material_points_;
    std::string team_;
    int banner_place_;
    double safety_dist_;
};

/***************************/
/* Integrated Mission Node */
/***************************/
typedef enum {
  IDLE,
  RECEIVED,
  RUNNING1,
  RUNNING2,
  RUNNING3,
  FINISH,
  FAILED
} MissionState;

class IntegratedMissionNode : public BT::StatefulActionNode
{
public:
  IntegratedMissionNode(const std::string& name, const BT::NodeConfig& config, const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
    : BT::StatefulActionNode(name, config), node_(params.nh.lock()), blackboard_(blackboard), tf_buffer_(params.nh.lock()->get_clock()), listener_(tf_buffer_) {
    publisher_ = node_->create_publisher<std_msgs::msg::Int32>("mission_type", 10);
    subscription_ = node_->create_subscription<std_msgs::msg::Int32>("mission_status", 10, std::bind(&IntegratedMissionNode::mission_callback, this, std::placeholders::_1));
  }

  /* Node remapping function */
  // get a list of integer to represend a mission. e.g. 132321 for taking one level
  // get a code for specifying mission point
  // get a posestemped for specifying start pose
  static BT::PortsList providedPorts(); 

  /* Start and running function */
  BT::NodeStatus onStart() override; // 
  // use while loop to execute each mission 
  // when encountering the nav mission. construct the nav object. and don't do other firmware misson
  // if need to do nav & firmware at the same time. need to input spetial code
  BT::NodeStatus onRunning() override; 
  // determine weather the firmware mission running.
  // counting finished mission 
  BT::NodeStatus mission_callback(const std_msgs::msg::Int32::SharedPtr sub_msg);
  
  /* Halt function */
  void onHalted() override;
  // Output:
  // output finished mission count
  // use BT to connect a LocReceiver to get current pose

private:
  std::shared_ptr<rclcpp::Node> node_;
  BT::Blackboard::Ptr blackboard_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;

  bool UpdateRobotPose();
  inline void setOutputs();

  geometry_msgs::msg::PoseStamped base_;
  geometry_msgs::msg::PoseStamped robot_pose_;
  std::vector<int64_t> mission_queue_;
  std::string mission_set_name_ = "NONE";
  std_msgs::msg::Int32 pub_msg;
  int mission_progress_ = 0;
  int mission_type_ = 0;
  int mission_status_ = IDLE;
  bool mission_received_ = false;
  bool mission_finished_ = false;
};

/**************/
/* ROS Client */
/**************/
class NavigateClient : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigation = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  NavigateClient(const geometry_msgs::msg::Pose& goal_pose, const double offset)
  : Node("navigate_client"), goal_pose_(goal_pose), offset_(offset) {
    nav_finished_ = false;
    nav_error_ = false;
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
  bool is_nav_finished() const;
  bool is_nav_success() const;
  void send_goal();

private:
  void goal_response_callback(GoalHandleNavigation::SharedPtr goal_handle);
  void feedback_callback(
    GoalHandleNavigation::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void result_callback(const GoalHandleNavigation::WrappedResult & result);

  double inline calculateDistance(const geometry_msgs::msg::Pose &pose1, const geometry_msgs::msg::Pose &pose2);
  double inline calculateAngleDifference(const geometry_msgs::msg::Pose &pose1, const geometry_msgs::msg::Pose &pose2);

  rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool nav_finished_;
  bool nav_error_;
  double offset_;
  geometry_msgs::msg::Pose goal_pose_;
  geometry_msgs::msg::PoseStamped goal_;
  geometry_msgs::msg::PoseStamped current_pose_;
};