#include <functional>
#include <memory>
#include <thread>

// ros2
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
// messages
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
// tf2
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/impl/utils.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2/exceptions.hpp"
// custom action
#include "btcpp_ros2_interfaces/action/navigation.hpp"

class TestNavServer : public rclcpp::Node {

public:
    explicit TestNavServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("test_nav_server", options) 
    {
        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<btcpp_ros2_interfaces::action::Navigation>(
            this,
            "testing_action",
            std::bind(&TestNavServer::handle_goal, this, _1, _2),
            std::bind(&TestNavServer::handle_cancel, this, _1),
            std::bind(&TestNavServer::handle_accepted, this, _1)
        );
    }

private:
    rclcpp_action::Server<btcpp_ros2_interfaces::action::Navigation>::SharedPtr action_server_;
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const btcpp_ros2_interfaces::action::Navigation::Goal> goal
    ) 
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->goal);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<btcpp_ros2_interfaces::action::Navigation>> goal_handle) 
    {

        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");

        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<btcpp_ros2_interfaces::action::Navigation>> goal_handle) 
    {

        using namespace std::placeholders;

        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&TestNavServer::execute, this, _1), goal_handle}.detach();
    }

    void botPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        loc_ = *msg;
        RCLCPP_INFO(node_->get_logger(), "Current Position: %f %f", loc_.pose.pose.position.x, loc_.pose.pose.position.y);
    }

    void moveTo(auto goal_handle)
    {
        feedback_ = std::make_shared<btcpp_ros2_interfaces::action::Navigation::Feedback>();
        is_arrived = false;
        rclcpp::Rate loop_rate(1 / deltaTime); // 100 Hz loop rate
        
        while(!is_arrived)
        {
            rclcpp::spin_some(this->botPoseCallback());
            
            double dx = goal_.goal.twist.linear.x - loc_.pose.pose.position.x;
            double dy = goal_.goal.twist.linear.y - loc_.pose.pose.position.y;
            double distance = sqrt(dx * dx + dy * dy);

            // Calculate the velocity (very simple local planner in nav2)
            // Convert orientation to angle
            tf2::Quaternion q;
            tf2::fromMsg(loc_.pose.pose.orientation, q);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            double angle = atan2(dy, dx);
            double dangle = angle - yaw;

            /* Check if we have to turn the robot */ 
            if(fabs(dangle) > 0.1)
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

            // If the distance is less than 0.1, the task is finished
            if(distance < 0.1)
            {
                // Stop the robot
                cmd_vel_.linear.x = 0.0;
                cmd_vel_.angular.z = 0.0;
                pub_cmd_vel_->publish(cmd_vel_);
                is_arrived = true;
            }
            feedback_->feedback.twist.linear.x = loc_.pose.pose.position.x;
            feedback_->feedback.twist.linear.y = loc_.pose.pose.position.y;
            feedback_->feedback.twist.angular.z = yaw;
            goal_handle->publish_feedback(feedback_);
            RCLCPP_INFO(this->get_logger(), "Publish feedback");
        }
    }

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<btcpp_ros2_interfaces::action::Navigation>> goal_handle) 
    {
        goal_ = goal_handle->get_goal();
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        // Initialize ROS Publisher
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
        // Initialize ROS Subscriber
        sub_loc_vel_ = this->create_subscriber("/odom", 1, &TestNavServer::botPoseCallback, this);
        moveTo(goal_handle);
        // Initialize ROS Timer (25HZ)
        // timer_ = this->create_wall_timer(500ms, moveTo);

        auto result = std::make_shared<btcpp_ros2_interfaces::action::Navigation::Result>();

        if (rclcpp::ok()) 
        {
            result->result = 0;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }
    /* ROS Publisher */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    /* ROS Subscriber */
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_loc_vel_;
    /* ROS Timer */
    // rclcpp::TimerBase::SharedPtr timer_;

    /* ROS Items */
    geometry_msgs::Twist cmd_vel_;
    nav_msgs::Odometry loc_;
    geometry_msgs::Twist goal_;
    auto feedback;

    /* nav params*/
    bool is_arrived;
    float deltaTime = 0.01;
};

RCLCPP_COMPONENTS_REGISTER_NODE(TestNavServer)