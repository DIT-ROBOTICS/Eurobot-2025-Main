/* Simple rclcpp publisher */
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "btcpp_ros2_interfaces/srv/start_up_srv.hpp" 

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
    StartUp() : Node("startup_node"), rate(100) {
        // initial_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 2);
        ready_pub = this->create_publisher<std_msgs::msg::String>("/robot/startup/plan", 2);
        time_pub = this->create_publisher<std_msgs::msg::Float32>("/robot/startup/time", 2);
        group_state_pub = this->create_publisher<std_msgs::msg::Int32MultiArray>("/robot/startup/groups_state", 2);
        web_plan_client = this->create_client<std_srvs::srv::Trigger>("/robot/startup/web_plan");
        start_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/startup/plug", 2, std::bind(&StartUp::StartCallback, this, std::placeholders::_1));
        ready_srv_server = this->create_service<btcpp_ros2_interfaces::srv::StartUpSrv>(
            "/robot/startup/ready_signal", std::bind(&StartUp::ReadyFeedback, this, std::placeholders::_1, std::placeholders::_2));
        start_srv_client = this->create_client<std_srvs::srv::SetBool>("/robot/startup/start_signal");
        obstacles_pub_ = this->create_publisher<btcpp_ros2_interfaces::msg::Obstacles>("ball_obstacles", 10);
        sima_start_pub_ = this->create_publisher<std_msgs::msg::Int16>("/sima/start", 10);
        
        this->declare_parameter<std::string>("Robot_name", "Panda");
        this->declare_parameter<std::vector<double>>("map_points_1", std::vector<double>{});
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
        this->declare_parameter<std::string>("sima_config_path", "/home/ros/share/data/sima.json");

        this->get_parameter("Robot_name", Robot_name_);
        this->get_parameter("map_points_1", material_points_);
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
        this->get_parameter("sima_config_path", sima_config_path_);
        sima_start_time_ = ReadSimaStartTime();

        start_up_state = INIT;
        sima_timer_ = nullptr;  // Initialize timer pointer to null
        plan_code_ = 0;
        web_plan_requested_ = false;
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(100),
            std::bind(&StartUp::StateMachine, this)
        );
    }

    double ReadSimaStartTime() {
        Json::Value root;
        std::ifstream file(sima_config_path_);
        if (!file.is_open()) {
            RCLCPP_WARN(this->get_logger(), "Could not open SIMA config file at %s, using default value 85", sima_config_path_.c_str());
            return 85;
        }
        
        Json::CharReaderBuilder builder;
        JSONCPP_STRING errs;
        if (!parseFromStream(builder, file, &root, &errs)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse SIMA config file: %s", errs.c_str());
            return 85;
        }

        if (root.isMember("sima_start_time")) {
            return root["sima_start_time"].asInt();
        } else {
            RCLCPP_WARN(this->get_logger(), "sima_start_time not found in config file, using default value 85");
            return 85;
        }
    }

    void StateMachine() {
        switch (start_up_state) {

        case INIT:
            // Call web_plan service to get plan code (only once)
            if (!plan_code_ && !web_plan_requested_) {
                CallWebPlanService();
            }

            /* choose plan from pannel and get robot init position */
            if (plan_code_) {
                /*****************************************************************/
                RCLCPP_INFO_STREAM(rclcpp::get_logger("startup"), "plan_code: " << plan_code_);
                team_colcor_ = (plan_code_ - plan_code_ / 10 * 10);
                UpdateTeamAndPoint(plan_code_);
                start_up_state = READY;
                groups_state.data = {0, 0, 0, 0};
                RCLCPP_INFO(this->get_logger(), "[StartUp Program]: INIT -> READY");
            }
            break;
        case READY:
            PublishReadySignal(ready_pub, group_state_pub);                                     // publish plan file name as start message
            if (ready_feedback[0] == START && ready_feedback[1] == START && ready_feedback[2] == START && ready_feedback[3] == START) {
                if (ready == false) {
                    RCLCPP_INFO(this->get_logger(), "[StartUp Program]: All of the programs are ready!");
                }
                ready = true;
            }
            /* Plug out the plug */
            if ((ready_feedback[0] == START && ready_feedback[1] == START && ready_feedback[2] == START && ready_feedback[3] == START) && start) {
                start_up_state = START;
                RCLCPP_INFO(this->get_logger(), "[StartUp Program]: READY -> START");
                /* Publish start signal */
                PublishStartSignal();
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

    // publish plan file name as ready message to every groups 
    void PublishReadySignal(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr plan_pub, rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr groups_state_pub) {
        // RCLCPP_INFO_STREAM(this->get_logger(), "publishing ready signal");
        start_position.header.stamp = this->get_clock()->now();
        for (int i = 0; i < 4; i++)
            groups_state.data[i] = static_cast<int>(ready_feedback[i]);
        // initial_pub->publish(start_position);                               // publish initial pose to everyone
        plan_pub->publish(start_plan);                                         // publish a string that is the xml file name
        groups_state_pub->publish(groups_state);
        rate.sleep();
    }

    // send the service request to bt_m (main function) to ask it to creat and start the behaviortree
    void PublishStartSignal() {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;
    
        start_srv_client->async_send_request(request, 
            [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "Response: success=%s, message=%s", response->success ? "true" : "false", response->message.c_str());
            }
        );
    }

    // publish time to everyone
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

        static bool sima_start_enabled = false;
        static bool time_check = false;

        // After sima_start_time_ seconds, start sending SIMA start signals continuously at 10Hz
        if (msg.data >= sima_start_time_) {
            if (!sima_start_enabled) {
                RCLCPP_INFO(this->get_logger(), "[StartUp Program]: Starting continuous SIMA start signal at time %d!", sima_start_time_);
                sima_start_enabled = true;
                
                // Create timer for continuous publishing if not already created
                if (!sima_timer_) {
                    sima_timer_ = this->create_wall_timer(
                        std::chrono::milliseconds(100), // 10Hz (100ms)
                        std::bind(&StartUp::PublishSIMAStartSignal, this)
                    );
                }
            }
        }

        if (msg.data >= 100 && !time_check) {                                  // check if it's timeout
            RCLCPP_INFO(this->get_logger(), "[StartUp Program]: Time is out!");
            time_check = true;
        }
    }

    // decode the plan code and find to correspon plan file name
    void UpdateTeamAndPoint(const int code) {
        int start_pt_code = 0;
        bool isTreenameSet = false;
        if (Robot_name_ == Bot1_name_) {                                       // for bot 1
            for (int i = 0; i < number_of_plans_[0] - 1; i ++) {               // for yellow team
                if (code == (i + 1) * 10) {
                    start_pt_code = int(start_points_bot1_yellow_[i]);
                    groot_filename = name_of_bot1_yellow_plans[i];
                    RCLCPP_INFO_STREAM(this->get_logger(), "[Bot1]: Yellow team plan " << (char)(i + 65));
                    isTreenameSet = true;
                    break;
                }
            }
            if (code == 100 * 10) {                                            // for spetial plan of yellow team
                start_pt_code = int(start_points_bot1_yellow_[number_of_plans_[0] - 1]);
                groot_filename = name_of_bot1_yellow_plans[number_of_plans_[0] - 1];
                RCLCPP_INFO_STREAM(this->get_logger(), "[Bot1]: Yellow team spetial plan");
                isTreenameSet = true;
            }
            for (int i = 0; i < number_of_plans_[1] - 1; i ++) {               // for blue team
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
            if (code == 100 * 10 + 1) {                                        // for spetial plan of blue team
                start_pt_code = int(start_points_bot1_blue_[number_of_plans_[0] - 1]) + 4;
                groot_filename = name_of_bot1_blue_plans[number_of_plans_[1] - 1];
                RCLCPP_INFO_STREAM(this->get_logger(), "[Bot1]: Blue team spetial plan");
                isTreenameSet = true;
            }
            if (!isTreenameSet)                                                // if can't find
                RCLCPP_ERROR_STREAM(this->get_logger(), "no plan match");
        }
        else if (Robot_name_ == Bot2_name_) {                                  // for bot 2
            isTreenameSet = false;
            for (int i = 0; i < number_of_plans_[2] - 1; i++) {                // for yellow team
                if (code == (i + 1) * 10) {
                    start_pt_code = int(start_points_bot2_yellow_[i]);
                    groot_filename = name_of_bot2_yellow_plans[i];
                    RCLCPP_INFO_STREAM(this->get_logger(), "[Bot2]: Yellow team plan " << (char)(i + 65));
                    isTreenameSet = true;
                    break;
                }
            }
            if (code == 100 * 10) {                                            // for spetial plan of yellow team
                start_pt_code = int(start_points_bot2_yellow_[number_of_plans_[0] - 1]);
                groot_filename = name_of_bot2_yellow_plans[number_of_plans_[2] - 1];
                RCLCPP_INFO_STREAM(this->get_logger(), "[Bot2]: Yellow team spetial plan");
                isTreenameSet = true;
            }
            for (int i = 0; i < number_of_plans_[3] - 1; i ++) {               // for blue team
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
            if (code == 100 * 10 + 1) {                                        // for spetial plan of blue team
                start_pt_code = int(start_points_bot2_blue_[number_of_plans_[0] - 1] + 4);
                groot_filename = name_of_bot2_blue_plans[number_of_plans_[3] - 1];
                RCLCPP_INFO_STREAM(this->get_logger(), "[Bot2]: Blue team spetial plan");
                isTreenameSet = true;
            }
            if (!isTreenameSet)
                RCLCPP_ERROR_STREAM(this->get_logger(), "no plan match");
        }

        // setting message of initial pose (no useage for now)
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

    //receive ready message from every groups
    void ReadyFeedback(const std::shared_ptr<btcpp_ros2_interfaces::srv::StartUpSrv::Request> request,
        std::shared_ptr<btcpp_ros2_interfaces::srv::StartUpSrv::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Received state %d from %d", request->state, request->group);
        response->group = request->group;
        response->success = false;
        if (request->group == 0) {                                             // message from bt_m (main program)
            if (request->state == 3 && (prev_msg[0] == READY || prev_msg[0] == INIT)) {
                response->success = true;
                ready_feedback[0] = START;
            }
            prev_msg[0] = StartUpState(request->state);                        // turn the message type into enum and store
        }
        else if (request->group == 2) {                                        // message from vision
            if (request->state == 3 && (prev_msg[1] == READY || prev_msg[1] == INIT)) {
                response->success = true;
                ready_feedback[1] = START;
            }
            prev_msg[1] = StartUpState(request->state);                        // turn the message type into enum and store
        }
        else if (request->group == 3) {                                        // message from navigation
            if (request->state == 3 && (prev_msg[2] == READY || prev_msg[2] == INIT)) {
                response->success = true;
                ready_feedback[2] = START;
            }
            prev_msg[2] = StartUpState(request->state);                        // turn the message type into enum and store
        }
        else if (request->group == 4) {                                        // message from localization
            if (request->state == 3 && (prev_msg[3] == READY || prev_msg[3] == INIT)) {
                response->success = true;
                ready_feedback[3] = START;
            }
            prev_msg[3] = StartUpState(request->state);                        // turn the message type into enum and store
        }
        RCLCPP_INFO(this->get_logger(), "Response %d to %d", int(response->success), response->group);
    }

    void CallWebPlanService() {
        if (!web_plan_client->wait_for_service(std::chrono::milliseconds(2000))) {
            // Service not available yet, will try again in next cycle
            plan_code_ = 40;
            return;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        web_plan_requested_ = true;  // Set flag to prevent multiple calls
        
        web_plan_client->async_send_request(request, 
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                try {
                    auto response = future.get();
                    if (response->success) {
                        plan_code_ = std::stoi(response->message);
                        RCLCPP_INFO(this->get_logger(), "Received plan code from web_plan service: %d", plan_code_);
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Web plan service call failed");
                        web_plan_requested_ = false;  // Reset flag to allow retry
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Error calling web plan service: %s", e.what());
                    web_plan_requested_ = false;  // Reset flag to allow retry
                }
            }
        );
    }

    void StartCallback(const std_msgs::msg::Bool::SharedPtr msg) {             // will be triggered by the plug
        if (msg->data == true && prev_start_msg == false) {
            start = true;
        }
        prev_start_msg = msg->data;
    }

    // Continuously publish SIMA start signal at 10Hz
    void PublishSIMAStartSignal() {
        std_msgs::msg::Int16 sima_msg;
        int sima_plan_code = 0;
        Json::Value root;
        std::ifstream file(sima_config_path_);
        if (file.is_open()) {
            Json::CharReaderBuilder builder;
            JSONCPP_STRING errs;
            if (parseFromStream(builder, file, &root, &errs)) {
                if (root.isMember("plan_code")) {
                    sima_plan_code = root["plan_code"].asInt();
                }
            }
        }
        int team_digit = (team_colcor_ == 0) ? 1 : 2; // Yellow = 1, Blue = 2
        sima_msg.data = sima_plan_code * 10 + team_digit;
        // RCLCPP_INFO(this->get_logger(), "SIMA start signal: sima_plan_code=%d, team_color=%d, publishing=%d", 
        //             sima_plan_code, team_colcor_, sima_msg.data);
        sima_start_pub_->publish(sima_msg);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr sima_timer_;    // Timer for continuous SIMA publishing
    // rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ready_pub;             // publish plan message as ready signal to every groups
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr time_pub;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr group_state_pub;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr web_plan_client;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub;            // plug message
    rclcpp::Service<btcpp_ros2_interfaces::srv::StartUpSrv>::SharedPtr ready_srv_server; // receive to check if every group start successfully
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr start_srv_client;        // it might can be removed, main can listen to start topic by plug directly

    rclcpp::Publisher<btcpp_ros2_interfaces::msg::Obstacles>::SharedPtr obstacles_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr sima_start_pub_;

    // Parameters
    std::string* name_of_bot1_yellow_plans = NULL;
    std::string* name_of_bot1_blue_plans = NULL;
    std::string* name_of_bot2_yellow_plans = NULL;
    std::string* name_of_bot2_blue_plans = NULL;
    std::string Robot_name_, Bot1_name_, Bot2_name_;
    std::string groot_filename;

    btcpp_ros2_interfaces::msg::CircleObstacle c;
    rclcpp::Rate rate;

    int team_colcor_;
    int number_of_plans_[4];                                                   // plan numbers of different color and different bot
    int plan_code_;
    bool ready = false;
    StartUpState prev_msg[4] = {INIT, INIT, INIT, INIT};                       // ready message from other programs
    StartUpState ready_feedback[4] = {INIT, START, START, START};   // it should be INIT        // ready message from other programs
    bool prev_start_msg = false;                                               // plug message
    bool start = false;  // it should be false                                  // plug message
    double starting_time = 0;
    StartUpState start_up_state;                                               // state of startup program
    std::vector<double> material_points_;                                      // prepared for choosing start point
    std::vector<double> number_of_plans_double_;
    std::vector<double> start_points_bot1_yellow_, start_points_bot1_blue_, start_points_bot2_yellow_, start_points_bot2_blue_;
    // ROS message
    geometry_msgs::msg::PoseWithCovarianceStamped start_position;
    std_msgs::msg::String start_plan;
    std_msgs::msg::Bool start_signal;
    std_msgs::msg::Int32MultiArray groups_state;
    std::string sima_config_path_;
    int sima_start_time_;
    bool web_plan_requested_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StartUp>());
    rclcpp::shutdown();
    return 0;
}