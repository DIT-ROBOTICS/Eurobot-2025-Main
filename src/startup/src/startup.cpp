/* Simple rclcpp publisher */
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

// tf2 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/impl/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include <jsoncpp/json/json.h>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <stdio.h>
#include <string>

#include "btcpp_ros2_interfaces/msg/obstacles.hpp"
#include "btcpp_ros2_interfaces/msg/circle_obstacle.hpp"
#include "btcpp_ros2_interfaces/msg/segment_obstacle.hpp"

#define PI 3.1415926

typedef enum StartUpState {
    INIT = 0,
    READY,
    ERROR,
    START
} StartUpState;

class StartUp : public rclcpp::Node {
public:
    StartUp() : Node("startup_node") {
        initial_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 2);
        ready_pub = this->create_publisher<std_msgs::msg::String>("/robot/startup/ready_signal", 2);
        start_pub = this->create_publisher<std_msgs::msg::Bool>("/robot/startup/start_signal", 2);
        time_pub = this->create_publisher<std_msgs::msg::Float32>("/robot/startup/time", 2);
        start_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/Start", 2, std::bind(&StartUp::StartCallback, this, std::placeholders::_1));
        vision_sub = this->create_subscription<std_msgs::msg::Int32>("/robot/startup/vision_ready_signal", 2, std::bind(&StartUp::VisionCallback, this, std::placeholders::_1));
        localization_sub = this->create_subscription<std_msgs::msg::Int32>("/robot/startup/localization_ready_signal", 2, std::bind(&StartUp::LocalizationCallback, this, std::placeholders::_1));
        navigation_sub = this->create_subscription<std_msgs::msg::Int32>("/robot/startup/navigation_ready_signal", 2, std::bind(&StartUp::NavigationCallback, this, std::placeholders::_1));
        // srv = this->create_service<btcpp_ros2_interfaces::srv::StartUpSrv>("/robot/startup/ready_signal_feedback",  &StartUp::ReadyFeedback);
        obstacles_pub_ = this->create_publisher<btcpp_ros2_interfaces::msg::Obstacles>("ball_obstacles", 10);
        
        this->declare_parameter<std::string>("Robot_name", "Tongue");
        this->declare_parameter<std::vector<double>>("material_points", std::vector<double>{});
        this->declare_parameter<std::vector<double>>("number_of_plans", std::vector<double>{});
        this->declare_parameter<std::vector<double>>("start_points_bot1_yellow", std::vector<double>{});
        this->declare_parameter<std::vector<double>>("start_points_bot1_blue", std::vector<double>{});
        this->declare_parameter<std::vector<double>>("start_points_bot2_yellow", std::vector<double>{});
        this->declare_parameter<std::vector<double>>("start_points_bot2_blue", std::vector<double>{});
        this->declare_parameter<std::string>("Bot1_name", "");
        this->declare_parameter<std::string>("Bot2_name", "");
        for (int i = 0; i < 26; i++) {
            std::string param_name_1 = "Bot1_Yellow";
            std::string param_name_2 = "Bot1_Blue";
            std::string param_name_3 = "Bot2_Yellow";
            std::string param_name_4 = "Bot2_Blue";
            param_name_1 += char(65 + i);
            param_name_2 += char(65 + i);
            param_name_3 += char(65 + i);
            param_name_4 += char(65 + i);
            param_name_1 += "_config";
            param_name_2 += "_config";
            param_name_3 += "_config";
            param_name_4 += "_config";
            this->declare_parameter<std::string>(param_name_1, "nan");
            this->declare_parameter<std::string>(param_name_2, "nan");
            this->declare_parameter<std::string>(param_name_3, "nan");
            this->declare_parameter<std::string>(param_name_4, "nan");
        }
        this->declare_parameter<std::string>("Bot1_YellowSpetial_config", "nan");
        this->declare_parameter<std::string>("Bot1_BlueSpetial_config", "nan");
        this->declare_parameter<std::string>("Bot2_YellowSpetial_config", "nan");
        this->declare_parameter<std::string>("Bot2_BlueSpetial_config", "nan");

        this->get_parameter("Robot_name", Robot_name_);
        this->get_parameter("material_points", material_points_);
        this->get_parameter("number_of_plans", number_of_plans_double_);
        this->get_parameter("Bot1_name", Bot1_name_);
        this->get_parameter("Bot2_name", Bot2_name_);
        this->get_parameter("start_points_bot1_yellow", start_points_bot1_yellow_);
        this->get_parameter("start_points_bot1_blue", start_points_bot1_blue_);
        this->get_parameter("start_points_bot2_yellow", start_points_bot2_yellow_);
        this->get_parameter("start_points_bot2_blue", start_points_bot2_blue_);
        for (int i = 0; i < 4; i++) {
            number_of_plans_[i] = int(number_of_plans_double_[i]);
        }
        number_of_plans_[0] = 6;
        number_of_plans_[1] = 6;
        number_of_plans_[2] = 4;
        number_of_plans_[3] = 4;
        name_of_bot1_yellow_plans = new std::string[number_of_plans_[0]];
        name_of_bot1_blue_plans = new std::string[number_of_plans_[1]];
        name_of_bot2_yellow_plans = new std::string[number_of_plans_[2]];
        name_of_bot2_blue_plans = new std::string[number_of_plans_[3]];
        for (int i = 0; i < number_of_plans_[0] - 1; i++) {
            std::string param_name = "Bot1_Yellow";
            param_name += char(65 + i);
            param_name += "_config";
            this->get_parameter(param_name, name_of_bot1_yellow_plans[i]);
        }
        this->get_parameter("Bot1_YellowSpetial_config", name_of_bot1_yellow_plans[number_of_plans_[0] - 1]);
        for (int i = 0; i < number_of_plans_[1] - 1; i++) {
            std::string param_name = "Bot1_Blue";
            param_name += char(65 + i);
            param_name += "_config";
            this->get_parameter(param_name, name_of_bot1_blue_plans[i]);
        }
        this->get_parameter("Bot1_BlueSpetial_config", name_of_bot1_blue_plans[number_of_plans_[1] - 1]);
        for (int i = 0; i < number_of_plans_[2] - 1; i++) {
            std::string param_name = "Bot2_Yellow";
            param_name += char(65 + i);
            param_name += "_config";
            this->get_parameter(param_name, name_of_bot2_yellow_plans[i]);
        }
        this->get_parameter("Bot2_YellowSpetial_config", name_of_bot2_yellow_plans[number_of_plans_[2] - 1]);
        for (int i = 0; i < number_of_plans_[3] - 1; i++) {
            std::string param_name = "Bot2_Blue";
            param_name += char(65 + i);
            param_name += "_config";
            this->get_parameter(param_name, name_of_bot2_blue_plans[i]);
        }
        this->get_parameter("Bot2_BlueSpetial_config", name_of_bot2_blue_plans[number_of_plans_[3] - 1]);

        start_up_state = INIT;
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(100),
            std::bind(&StartUp::StateMachine, this)
        );
    }

    void StateMachine() {
        switch (start_up_state) {

        case INIT:

            // ReadJsonFile(file_path);
            /* temp: will be delete and get the `plan_code_` from web pannel */ 
            this->declare_parameter<int>("plan_code", 0);  // ten: plan, one: color
            this->get_parameter("plan_code", plan_code_);
            /*****************************************************************/
            RCLCPP_INFO_STREAM(rclcpp::get_logger("startup"), "plan_code: " << plan_code_);
            team_colcor_ = (plan_code_ - plan_code_ / 10 * 10);
            UpdateTeamAndPoint(plan_code_);

            /* Press ready signal and get robot init position */
            if (plan_code_) {
                start_up_state = READY;
                RCLCPP_INFO(this->get_logger(), "[StartUp Program]: INIT -> READY");
            }
            break;
        case READY:
            PublishReadySignal(ready_pub, initial_pub);  // To Do: will change it into service
            if (ready_feedback[0] == ready_feedback[1] == ready_feedback[2] == START) {
                if (ready == false) {
                    RCLCPP_INFO(this->get_logger(), "[StartUp Program]: All of the programs are ready!");
                }
                ready = true;
                // might need to return ACK back to vision, localization & navigation
            }
            /* Plug out the plug */
            if ((ready_feedback[0] == ready_feedback[1] == ready_feedback[2] == START) && start) {
                start_up_state = START;
                RCLCPP_INFO(this->get_logger(), "[StartUp Program]: READY -> START");
                /* Publish start signal */
                PublishStartSignal(start_pub);
                /* Start the time */
                starting_time = this->get_clock()->now().seconds();
            }
            break;
        case START:
            PublishTime(time_pub);
            break;
        default:
            RCLCPP_INFO(this->get_logger(), "[StartUp Program]: UNKNOWN");
            break;
        }
    }

    void PublishReadySignal(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub, rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pub) {
        start_position.header.stamp = this->get_clock()->now();
        // initial_pub->publish(start_position);
        pub->publish(start_plan);
    }

    void PublishStartSignal(rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub) {
        start_signal.data = start;
        pub->publish(start_signal);
    }

    void PublishTime(rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub) {

        // Get current time by second
        double current_time = this->get_clock()->now().seconds();

        std_msgs::msg::Float32 msg;
        msg.data = current_time - starting_time;
        pub->publish(msg);

        c.center.x = 1.5;
        c.center.y = 1.0;
        c.velocity.x = 0;
        c.velocity.y = 0;
        c.radius = 0.1;
        
        auto obstacles_msg = btcpp_ros2_interfaces::msg::Obstacles();
        obstacles_msg.circles.push_back(c);
        obstacles_msg.header.frame_id = "map";
        obstacles_msg.header.stamp = this->get_clock()->now();
        obstacles_pub_->publish(obstacles_msg);

        static bool is_published = false;
        static bool time_check = false;

        if (msg.data >= 85 && !is_published) {
            // RCLCPP_INFO(this->get_logger(), "[StartUp Program]: Send the ladybug!");

            // Use system call the ladybug script
            // system("/home/main_ws/scripts/ladybug.sh");

            is_published = true;
        }

        if (msg.data >= 100 && !time_check) {
            RCLCPP_INFO(this->get_logger(), "[StartUp Program]: Time is out!");
            time_check = true;
        }
    }

    void UpdateTeamAndPoint(const int code) {
        int start_pt_code = 0;
        bool isTreenameSet = false;
        if (Robot_name_ == Bot1_name_) {
            for (int i = 0; i < number_of_plans_[0] - 1; i ++) {
                if (code == (i + 1) * 10) {
                    start_pt_code = int(start_points_bot1_yellow_[i]);
                    groot_filename = name_of_bot1_yellow_plans[i];
                    RCLCPP_INFO_STREAM(this->get_logger(), "[Bot1]: Yellow team plan " << (char)(i + 65));
                    isTreenameSet = true;
                    break;
                }
            }
            if (code == 100 * 10) {
                start_pt_code = int(start_points_bot1_yellow_[number_of_plans_[0] - 1]);
                groot_filename = name_of_bot1_yellow_plans[number_of_plans_[0] - 1];
                isTreenameSet = true;
            }
            for (int i = 0; i < number_of_plans_[1] - 1; i ++) {
                if (isTreenameSet)
                    break;
                if (code == (i + 1) * 10 + 1) {
                    start_pt_code = int(start_points_bot1_blue_[i]) + 4;
                    groot_filename = name_of_bot1_blue_plans[i];
                    RCLCPP_INFO_STREAM(this->get_logger(), "[Bot1]: Blue team plan " << (char)(i + 65));
                    isTreenameSet = true;
                    break;
                }
            }
            if (code == 100 * 10 + 1) {
                start_pt_code = int(start_points_bot1_blue_[number_of_plans_[0] - 1]) + 4;
                groot_filename = name_of_bot1_blue_plans[number_of_plans_[1] - 1];
                isTreenameSet = true;
            }
            if (!isTreenameSet)
                RCLCPP_ERROR_STREAM(this->get_logger(), "no plan match");
        }
        else if (Robot_name_ == Bot2_name_) {
            isTreenameSet = false;
            for (int i = 0; i < number_of_plans_[2] - 1; i++) {
                if (code == (i + 1) * 10) {
                    start_pt_code = int(start_points_bot2_yellow_[i]);
                    groot_filename = name_of_bot2_yellow_plans[i];
                    RCLCPP_INFO_STREAM(this->get_logger(), "[Bot2]: Yellow team plan " << (char)(i + 65));
                    isTreenameSet = true;
                    break;
                }
            }
            if (code == 100 * 10) {
                start_pt_code = int(start_points_bot2_yellow_[number_of_plans_[0] - 1]);
                groot_filename = name_of_bot2_yellow_plans[number_of_plans_[2] - 1];
                isTreenameSet = true;
            }
            for (int i = 0; i < number_of_plans_[3] - 1; i ++) {
                if (isTreenameSet)
                    break;
                if (code == (i + 1) * 10 + 1) {
                    start_pt_code = int(start_points_bot2_blue_[i] + 4);
                    groot_filename = name_of_bot2_blue_plans[i];
                    RCLCPP_INFO_STREAM(this->get_logger(), "[Bot2]: Blue team plan " << (char)(i + 65));
                    isTreenameSet = true;
                    break;
                }
            }
            if (code == 100 * 10 + 1) {
                start_pt_code = int(start_points_bot2_blue_[number_of_plans_[0] - 1] + 4);
                groot_filename = name_of_bot2_blue_plans[number_of_plans_[3] - 1];
                isTreenameSet = true;
            }
            if (!isTreenameSet)
                RCLCPP_ERROR_STREAM(this->get_logger(), "no plan match");
        }
        start_position.pose.pose.position.x = material_points_[start_pt_code * 5];
        start_position.pose.pose.position.y = material_points_[start_pt_code * 5 + 1];
        tf2::Quaternion q; // declare Quaternion
        q.setRPY(0, 0, material_points_[start_pt_code * 5 + 2] * PI / 2); // change degree-z into Quaternion
        start_position.pose.pose.orientation.x = q.x();
        start_position.pose.pose.orientation.y = q.y();
        start_position.pose.pose.orientation.z = q.z();
        start_position.pose.pose.orientation.w = q.w();
        start_plan.data = groot_filename + std::to_string(team_colcor_);
    }

    // will be triggered by the plug
    void StartCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data == true && prev_start_msg == false) {
            start = true;               
        }
        prev_start_msg = msg->data;
    }
    void VisionCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        if (msg->data == 3 && (prev_msg[0] == READY || prev_msg[0] == INIT)) {
            ready_feedback[0] = START;
        }
        prev_msg[0] = msg->data;
    }
    void LocalizationCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        if (msg->data == 3 && (prev_msg[1] == READY || prev_msg[1] == INIT)) {
            ready_feedback[1] = START;
        }
        prev_msg[1] = msg->data;
    }
    void NavigationCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        if (msg->data == 3 && (prev_msg[1] == READY || prev_msg[1] == INIT)) {
            ready_feedback[2] = START;
        }
        prev_msg[2] = msg->data;
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ready_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr time_pub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr start_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr vision_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr localization_sub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr navigation_sub;

    rclcpp::Publisher<btcpp_ros2_interfaces::msg::Obstacles>::SharedPtr obstacles_pub_;

    // Parameters
    std::string* name_of_bot1_yellow_plans = NULL;
    std::string* name_of_bot1_blue_plans = NULL;
    std::string* name_of_bot2_yellow_plans = NULL;
    std::string* name_of_bot2_blue_plans = NULL;
    std::string Robot_name_, Bot1_name_, Bot2_name_;
    std::string groot_filename;

    btcpp_ros2_interfaces::msg::CircleObstacle c;

    int team_colcor_;
    int number_of_plans_[4];                           // plan numbers of different color and different bot
    int plan_code_;
    bool ready = false;
    StartUpState prev_msg[3] = {INIT};                 // ready message from other programs
    StartUpState ready_feedback[3] = {START};   // it should be INIT        // ready message from other programs
    bool prev_start_msg = false;                       // plug message
    bool start = true;  // it should be false          // plug message
    double starting_time = 0;
    StartUpState start_up_state;                       // state of startup program
    std::vector<double> material_points_;              // prepared for choosing start point
    std::vector<double> number_of_plans_double_;
    std::vector<double> start_points_bot1_yellow_, start_points_bot1_blue_, start_points_bot2_yellow_, start_points_bot2_blue_;
    // ROS message
    geometry_msgs::msg::PoseWithCovarianceStamped start_position;
    std_msgs::msg::String start_plan;
    std_msgs::msg::Bool start_signal;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StartUp>());
    rclcpp::shutdown();
    return 0;
}