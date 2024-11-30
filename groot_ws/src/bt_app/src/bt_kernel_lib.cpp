// #include "bt_app/bt_kernel_lib.h"

// Constructor
navigationMain::navigationMain(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local) {

    // Initialize ROS Publisher
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // Initialize ROS Subscriber
    sub_loc_vel_ = nh_.subscribe("/odom", 1, &navigationMain::locVelCallback, this);

    // Initialize ROS Timer (25HZ)
    timer_ = nh_.createTimer(ros::Duration(0.04), &navigationMain::timerCallback, this);

    // Start the timer
    timer_.start();
}

// Destructor
navigationMain::~navigationMain() {
    // Do nothing
}

// Callback function for /loc_vel subscriber
void navigationMain::locVelCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Store the message
    loc_ = *msg;

    // ROS_INFO_STREAM("Current Position: " << loc_.pose.pose.position.x << ", " << loc_.pose.pose.position.y);
}

void navigationMain::getCurrentTimeInSecond(double& time) {
    time = ros::Time::now().toSec();
}

// Callback function for timer
void navigationMain::timerCallback(const ros::TimerEvent& event) {

    if (is_trigger_) {

        // The goal position is set
        double dx = goal_.twist.linear.x - loc_.pose.pose.position.x;
        double dy = goal_.twist.linear.y - loc_.pose.pose.position.y;
        double distance = sqrt(dx * dx + dy * dy);

        // If the distance is less than 0.1, the task is finished
        if (distance < 0.1) {
            
            is_trigger_ = false;

            // Stop the robot
            cmd_vel_.linear.x = 0.0;
            cmd_vel_.angular.z = 0.0;
            pub_cmd_vel_.publish(cmd_vel_);

            for (auto& callback : finish_callbacks_) {
                callback();
            }
        } 
        else {

            // Calculate the velocity (very simple local planner in nav2)

            // Convert orientation to angle
            tf2::Quaternion q;
            tf2::fromMsg(loc_.pose.pose.orientation, q);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);

            double angle = atan2(dy, dx);
            double dangle = angle - yaw;

            if (/* Check if we have to turn the robot */ fabs(dangle) > 0.1) {
                cmd_vel_.linear.x = 0.0;
                cmd_vel_.angular.z = 0.3 * dangle;
            } 
            else {
                cmd_vel_.linear.x = 0.3;
                cmd_vel_.angular.z = 0.0;
            }

            pub_cmd_vel_.publish(cmd_vel_);
        }

    }
}

void navigationMain::onHalted() {

    // Zero the velocity
    cmd_vel_.linear.x = 0.0;
    cmd_vel_.angular.z = 0.0;
    pub_cmd_vel_.publish(cmd_vel_);

    // Set the trigger to false
    is_trigger_ = false;
}

// Set the goal position
void navigationMain::setGoal(geometry_msgs::TwistStamped goal_position) {

    goal_ = goal_position;
    is_trigger_ = true;

    // Log the twist stamped message
    ROS_INFO_STREAM("Goal Position: " << goal_.twist.linear.x << ", " << goal_.twist.linear.y);
}

// Register a callback function to be called when the task is finished
void navigationMain::registerFinishCallback(std::function<void()> callback) {
    finish_callbacks_.push_back(callback);
}

// Clear all the registered callback functions
void navigationMain::clearFinishCallbacks() {
    finish_callbacks_.clear();
}

// Set the race time
void navigationMain::setRaceTime() {
    race_elapsed_time_ = ros::Time::now().toSec();
}

// Get the race elapsed time
void navigationMain::getRaceElapsedTime(double& time) {
    time = ros::Time::now().toSec() - race_elapsed_time_;
}