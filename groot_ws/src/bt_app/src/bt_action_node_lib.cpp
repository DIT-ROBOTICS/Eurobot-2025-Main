#include "bt_app/bt_action_node_lib.h"

// template <>
// inline geometry_msgs::msg::TwistStamped convertFromString(BT::StringView str)
// {
//   auto parts = BT::splitString(str, ',');
//   if(parts.size() != 3)
//   {
//     throw BT::RuntimeError("invalid input)");
//   }
//   else
//   {
//     // Testing function -> Only remain x and y
//     geometry_msgs::msg::TwistStamped output;
//     output.twist.linear.x = BT::convertFromString<double>(parts[0]);
//     output.twist.linear.y = BT::convertFromString<double>(parts[1]);
//     return output;
//   }
// }

// template <>
// inline int BT::convertFromString(StringView str)
// {
//   auto value = convertFromString<double>(str);
//   return (int)value;
// }

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

// Provide the ports for the SimpleNavigation node
BT::PortsList SimpleNavigation::providedPorts()
{
  return { BT::InputPort<geometry_msgs::msg::TwistStamped>("goal") };
}

// Start function for the SimpleNavigation node
BT::NodeStatus SimpleNavigation::onStart()
{
  // Get the goal
  RCLCPP_INFO(node_->get_logger(), "test2");
  geometry_msgs::msg::TwistStamped goal;
  if(!getInput<geometry_msgs::msg::TwistStamped>("goal", goal))
  {
    throw BT::RuntimeError("[SimpleNavigation]: Missing required input [goal]");
  }
  // timer_ = node_->create_wall_timer(std::chrono::milliseconds(500), std::bind(&SimpleNavigation::timerCallback, this));

  // Set the goal
  // kernel_->setGoal(goal);
  setGoal(goal);

  // Set the finished flag
  finished_ = false;

  // Register the goal function
  // kernel_->registerFinishCallback(std::bind(&SimpleNavigation::setFinished, this));

  return BT::NodeStatus::RUNNING;
}

// Running function for the SimpleNavigation node
BT::NodeStatus SimpleNavigation::onRunning()
{
  RCLCPP_INFO(node_->get_logger(), "test3");
  if(finished_)
  {
    // kernel_->clearFinishCallbacks();
    return BT::NodeStatus::SUCCESS;
  }

  // Do nothing
  timerCallback();
  return BT::NodeStatus::RUNNING;
}

// Finished function for the SimpleNavigation node
void SimpleNavigation::setFinished()
{
  finished_ = true;
}

// Halted function for the SimpleNavigation node
void SimpleNavigation::onHalted()
{
  // kernel_->onHalted();
  // ******************* simple navigaiton
  // Zero the velocity
  cmd_vel_.linear.x = 0.0;
  cmd_vel_.angular.z = 0.0;
  pub_cmd_vel_->publish(cmd_vel_);

  // Set the trigger to false
  is_trigger_ = false;
  // **********************
  // kernel_->clearFinishCallbacks();

  std::cout << "[SimpleNavigation]: Simple Navigation Node halted\n";

  return;
}

// **************** simple navigation
// Callback function for /loc_vel subscriber
void SimpleNavigation::locVelCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Store the message
  loc_ = *msg;

  RCLCPP_INFO(node_->get_logger(), "Current Position: %f %f", loc_.pose.pose.position.x,
              loc_.pose.pose.position.y);
}
// Callback function for timer
void SimpleNavigation::timerCallback(/*const rclcpp::TimerEvent& event*/)
{
  printf("test4");
  RCLCPP_INFO(node_->get_logger(), "is_trigger: %d", is_trigger_);
  if(is_trigger_)
  {
    // The goal position is set
    double dx = goal_.twist.linear.x - loc_.pose.pose.position.x;
    double dy = goal_.twist.linear.y - loc_.pose.pose.position.y;
    double distance = sqrt(dx * dx + dy * dy);

    // If the distance is less than 0.1, the task is finished
    if(distance < 0.1)
    {
      is_trigger_ = false;

      // Stop the robot
      cmd_vel_.linear.x = 0.0;
      cmd_vel_.angular.z = 0.0;
      pub_cmd_vel_->publish(cmd_vel_);

      for(auto& callback : finish_callbacks_)
      {
        callback();
      }
    }
    else
    {
      // Calculate the velocity (very simple local planner in nav2)

      // Convert orientation to angle
      tf2::Quaternion q;
      tf2::fromMsg(loc_.pose.pose.orientation, q);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      double angle = atan2(dy, dx);
      double dangle = angle - yaw;

      if(/* Check if we have to turn the robot */ fabs(dangle) > 0.1)
      {
        cmd_vel_.linear.x = 0.0;
        cmd_vel_.angular.z = 0.3 * dangle;
      }
      else
      {
        cmd_vel_.linear.x = 0.3;
        cmd_vel_.angular.z = 0.0;
      }

      pub_cmd_vel_->publish(cmd_vel_);
    }
  }
}
// Set the goal position
void SimpleNavigation::setGoal(geometry_msgs::msg::TwistStamped goal_position)
{
  goal_ = goal_position;
  is_trigger_ = true;

  // Log the twist stamped message
  RCLCPP_INFO(node_->get_logger(), "Goal Position: %f %f ", goal_.twist.linear.x,
              goal_.twist.linear.y);
}
void finished_function()
{
  printf("Finished!");
}