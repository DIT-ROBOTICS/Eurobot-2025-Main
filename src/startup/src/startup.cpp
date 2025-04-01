/* Simple rclcpp publisher */
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include <jsoncpp/json/json.h>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <stdio.h>
#include <string>

typedef enum StartUpState {
    INIT = 0,
    READY,
    ERROR,
    START
} StartUpState;

class StartUp : public rclcpp::Node {
public:
    StartUp() : Node("startup_node") {
        pub = this->create_publisher<std_msgs::msg::String>("/robot/startup/ready_signal", 2);
        start_pub = this->create_publisher<std_msgs::msg::String>("/robot/startup/start_signal", 2);
        time_pub = this->create_publisher<std_msgs::msg::Float32>("/robot/startup/time", 2);
        start_sub = this->create_subscription<std_msgs::msg::Int32>("/robot/Start", 2, std::bind(&StartUp::StartCallback, this, std::placeholders::_1));
        // srv = this->create_service<btcpp_ros2_interfaces::srv::StartUpSrv>("/robot/startup/ready_signal_feedback",  &StartUp::ReadyFeedback);
        
        // file for bot1
        this->declare_parameter<std::string>("Bot1_YellowA_config", "bt_plan_a_Yellow.xml");
        this->declare_parameter<std::string>("Bot1_YellowB_config", "bt_plan_b_Yellow.xml");
        this->declare_parameter<std::string>("Bot1_YellowC_config", "bt_plan_c_Yellow.xml");
        this->declare_parameter<std::string>("Bot1_YellowD_config", "");
        this->declare_parameter<std::string>("Bot1_YellowE_config", "");
        this->declare_parameter<std::string>("Bot1_YellowF_config", "");
        this->declare_parameter<std::string>("Bot1_YellowS_config", "");
        this->declare_parameter<std::string>("Bot1_BlueA_config", "bt_plan_a_Blue.xml");
        this->declare_parameter<std::string>("Bot1_BlueB_config", "bt_plan_b_Blue.xml");
        this->declare_parameter<std::string>("Bot1_BlueC_config", "bt_plan_c_Blue.xml");
        this->declare_parameter<std::string>("Bot1_BlueD_config", "");
        this->declare_parameter<std::string>("Bot1_BlueE_config", "");
        this->declare_parameter<std::string>("Bot1_BlueF_config", "");
        this->declare_parameter<std::string>("Bot1_BlueS_config", "");
        // file for bot2
        this->declare_parameter<std::string>("Bot2_YellowA_config", "bot2_yellow_a.xml");
        this->declare_parameter<std::string>("Bot2_YellowB_config", "bot2_yellow_b.xml");
        this->declare_parameter<std::string>("Bot2_YellowC_config", "bot2_yellow_c.xml");
        this->declare_parameter<std::string>("Bot2_YellowD_config", "");
        this->declare_parameter<std::string>("Bot2_YellowE_config", "");
        this->declare_parameter<std::string>("Bot2_YellowF_config", "");
        this->declare_parameter<std::string>("Bot2_YellowS_config", "");
        this->declare_parameter<std::string>("Bot2_BlueA_config", "bot2_blue_a.xml");
        this->declare_parameter<std::string>("Bot2_BlueB_config", "bot2_blue_b.xml");
        this->declare_parameter<std::string>("Bot2_BlueC_config", "bot2_blue_c.xml");
        this->declare_parameter<std::string>("Bot2_BlueD_config", "");
        this->declare_parameter<std::string>("Bot2_BlueE_config", "");
        this->declare_parameter<std::string>("Bot2_BlueF_config", "");
        this->declare_parameter<std::string>("Bot2_BlueS_config", "");

        this->declare_parameter<std::string>("Robot_name", "Tongue");
        this->declare_parameter<std::vector<double>>("material_points", std::vector<double>{});
        this->get_parameter("Robot_name", Robot_name_);
        this->get_parameter("material_points", material_points_);
        this->get_parameter("Bot1_YellowA_config", Bot1_YellowA_file);
        this->get_parameter("Bot1_YellowB_config", Bot1_YellowB_file);
        this->get_parameter("Bot1_YellowC_config", Bot1_YellowC_file);
        this->get_parameter("Bot1_YellowS_config", Bot1_YellowS_file);
        this->get_parameter("Bot1_BlueA_config", Bot1_BlueA_file);
        this->get_parameter("Bot1_BlueB_config", Bot1_BlueB_file);
        this->get_parameter("Bot1_BlueC_config", Bot1_BlueC_file);
        this->get_parameter("Bot1_BlueS_config", Bot1_BlueS_file);
        this->get_parameter("Bot2_BlueA_config", Bot2_BlueA_file);

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
            PublishReadySignal(pub);
            if (ready_feedback == 1) {
                if (ready == false) {
                    RCLCPP_INFO(this->get_logger(), "[StartUp Program]: All of the programs are ready!");
                }
                ready = true;
            }
            /* Press start signal */
            if ( (ready_feedback == 1) && start) {
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

    void PublishReadySignal(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub) {
        // start_position.header.stamp = this->get_clock()->now();
        // pub->publish(start_position);
        pub->publish(start_plan);
    }

    void PublishStartSignal(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub) {
        // start_position.header.stamp = this->get_clock()->now();
        // pub->publish(start_position);
        pub->publish(start_plan);
    }

    void PublishTime(rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub) {

        // Get current time by second
        double current_time = this->get_clock()->now().seconds();

        std_msgs::msg::Float32 msg;
        msg.data = current_time - starting_time;
        pub->publish(msg);

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
        if (Robot_name_ == "Tongue") {
            switch (code) {
            case 10:
                start_pt_code = 11;
                groot_filename = Bot1_YellowA_file;
                RCLCPP_INFO(this->get_logger(), "[Bot1]: Yellow team plan A");
                break;
            case 20:
                start_pt_code = 10;
                groot_filename = Bot1_YellowB_file;
                RCLCPP_INFO(this->get_logger(), "[Bot1]: Yellow team plan B");
                break;
            case 30: 
                start_pt_code = 10;
                groot_filename = Bot1_YellowC_file;
                RCLCPP_INFO(this->get_logger(), "[Bot1]: Yellow team plan C");
                break;
            case 11:
                start_pt_code = 15;
                groot_filename = Bot1_BlueA_file;
                RCLCPP_INFO(this->get_logger(), "[Bot1]: Blue team plan A");
                break;
            case 21:
                start_pt_code = 19;
                groot_filename = Bot1_BlueB_file;
                RCLCPP_INFO(this->get_logger(), "[Bot1]: Blue team plan B");
                break;
            case 31: 
                start_pt_code = 19;
                groot_filename = Bot1_BlueC_file;
                RCLCPP_INFO(this->get_logger(), "[Bot1]: Blue team plan C");
                break;
            default: 
                start_pt_code = 10;
                groot_filename = Bot1_YellowB_file;
                RCLCPP_INFO(this->get_logger(), "[Bot1]: Yellow team plan B");
                break;
            }
        }
        else if (Robot_name_ == "Invisible") {
            switch (code) {
            case 10:
                start_pt_code = 13;
                groot_filename = Bot2_YellowA_file;
                RCLCPP_INFO(this->get_logger(), "[Bot2]: Yellow team plan A");
                break;
            case 20:
                start_pt_code = 10;
                groot_filename = Bot2_YellowB_file;
                RCLCPP_INFO(this->get_logger(), "[Bot2]: Yellow team plan B");
                break;
            case 30: 
                start_pt_code = 10;
                groot_filename = Bot2_YellowC_file;
                RCLCPP_INFO(this->get_logger(), "[Bot2]: Yellow team plan C");
                break;
            case 11:
                start_pt_code = 17;
                groot_filename = Bot2_BlueA_file;
                RCLCPP_INFO(this->get_logger(), "[Bot2]: Blue team plan A");
                break;
            case 21:
                start_pt_code = 20;
                groot_filename = Bot2_BlueB_file;
                RCLCPP_INFO(this->get_logger(), "[Bot2]: Blue team plan B");
                break;
            case 31: 
                start_pt_code = 20;
                groot_filename = Bot2_BlueC_file;
                RCLCPP_INFO(this->get_logger(), "[Bot2]: Blue team plan C");
                break;
            default:
                start_pt_code = 17;
                groot_filename = Bot2_BlueA_file;
                RCLCPP_INFO(this->get_logger(), "[Bot2]: Blue team plan A");
                break;
            }
        }
        start_position.point.x = material_points_[start_pt_code * 4];
        start_position.point.y = material_points_[start_pt_code * 4 + 1];
        start_position.point.z = material_points_[start_pt_code * 4 + 2];
        start_plan.data = groot_filename + std::to_string(team_colcor_);
    }

    void StartCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        static int prev_msg = 0;

        if (msg->data == 1) {
            if (prev_msg == 0) {
                ready_feedback = 1;
                start = true;
                RCLCPP_INFO_STREAM(this->get_logger(), "start callback: " << ready_feedback << start);
            }
        }
        prev_msg = msg->data;
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr start_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr time_pub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr start_sub;

    // Parameters
    std::string Bot1_YellowA_file, Bot1_YellowB_file, Bot1_YellowC_file, Bot1_YellowD_file, Bot1_YellowE_file, Bot1_YellowF_file, Bot1_YellowS_file;
    std::string Bot1_BlueA_file, Bot1_BlueB_file, Bot1_BlueC_file, Bot1_BlueD_file, Bot1_BlueE_file, Bot1_BlueF_file, Bot1_BlueS_file;
    std::string Bot2_YellowA_file, Bot2_YellowB_file, Bot2_YellowC_file, Bot2_YellowD_file, Bot2_YellowE_file, Bot2_YellowF_file, Bot2_YellowS_file;
    std::string Bot2_BlueA_file, Bot2_BlueB_file, Bot2_BlueC_file, Bot2_BlueD_file, Bot2_BlueE_file, Bot2_BlueF_file, Bot2_BlueS_file;
    std::string Robot_name_;
    std::string groot_filename;

    int team_colcor_;
    int ready_feedback = 0;
    int plan_code_;
    bool ready = false;
    bool pub_ready = false;
    bool start = false;
    double starting_time = 0;
    StartUpState start_up_state;
    std::vector<double> material_points_;
    geometry_msgs::msg::PointStamped start_position;
    std_msgs::msg::String start_plan;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StartUp>());
    rclcpp::shutdown();
    return 0;
}