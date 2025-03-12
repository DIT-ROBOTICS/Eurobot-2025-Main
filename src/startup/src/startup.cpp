/* Simple rclcpp publisher */
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

/* Include startup necessary service */
#include "btcpp_ros2_interfaces/srv/start_up_srv.hpp"

#include <jsoncpp/json/json.h>
#include <fstream>

typedef enum StartUpState {
    INIT = 0,
    READY,
    ERROR,
    START
} StartUpState;

class StartUp : public rclcpp::Node {
public:
    StartUp() : Node("startup_node") {
        pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/robot/startup/ready_signal", 2);
        start_pub = this->create_publisher<geometry_msgs::msg::PointStamped>("/robot/startup/start_signal", 2);
        time_pub = this->create_publisher<std_msgs::msg::Float32>("/robot/startup/time", 2);
        start_sub = this->create_subscription<std_msgs::msg::Int32>("/robot/Start", 2, std::bind(&StartUp::StartCallback, this, std::placeholders::_1));
        // srv = this->create_service<btcpp_ros2_interfaces::srv::StartUpSrv>("/robot/startup/ready_signal_feedback",  &StartUp::ReadyFeedback);

        /* temp: will be delete and get the `start_point_` from web pannel */ 
        int start_point_ = 0;
        this->declare_parameter<int>("start_point", 0);
        this->get_parameter("start_point", start_point_);
        /*******************************************************************/
        RCLCPP_INFO_STREAM(rclcpp::get_logger("startup"), "start_point: " << start_point_);
        
        st_point[start_point_] = 1;
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
            start_point = CheckStartPoint();

            if (/* Press ready signal and get robot init position */ start_point) {
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

            if (/* Press start signal */ (ready_feedback == 1) && start) {
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

    void PublishReadySignal(rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub) {

        start_position.header.stamp = this->get_clock()->now();

        if (team == 0) {
            start_position.header.frame_id = "0";
        } else {
            start_position.header.frame_id = "1";
        }
        pub->publish(start_position);
    }

    void PublishStartSignal(rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub) {

        start_position.header.stamp = this->get_clock()->now();

        if (team == 0) {
            start_position.header.frame_id = "0";
        } else {
            start_position.header.frame_id = "1";
        }
        pub->publish(start_position);
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

    // bool ReadyFeedback(const std::shared_ptr<btcpp_ros2_interfaces::srv::StartUpSrv::Request> req,
    //                         std::shared_ptr<btcpp_ros2_interfaces::srv::StartUpSrv::Response> res) {

        // ready_feedback += req->group;

    //     return true;
    // }
    void UpdateTeamAndPoint(int point) {
        switch (point) {
        case 0:
            team = 0;
            plan = 'A';
            start_position.point.x = 2.6;
            start_position.point.y = 0.9;
            start_position.point.z = 1;
            break;
        case 1:
            team = 0;
            plan = 'B';
            start_position.point.x = 0.5;
            start_position.point.y = 1.7;
            start_position.point.z = 2;
            break;
        case 2: 
            team = 0;
            plan = 'C';
            start_position.point.x = 1.25;
            start_position.point.y = 0.25;
            start_position.point.z = 3;
            break;
        case 3:
            team = 1;
            plan = 'A';
            start_position.point.x = 2.7;
            start_position.point.y = 0.9;
            start_position.point.z = 1;
            break;
        case 4:
            team = 1;
            plan = 'B';
            start_position.point.x = 2.5;
            start_position.point.y = 1.7;
            start_position.point.z = 2;
            break;
        case 5: 
            team = 1;
            plan = 'C';
            start_position.point.x = 1.75;
            start_position.point.y = 0.25;
            start_position.point.z = 3;
            break;
        }
    }

    int CheckStartPoint() {
        
        for (int i = 0; i < 6; i++) {
            if (st_point[i] == 1) {
                UpdateTeamAndPoint(i);
                return i;
            }
        }
        return 0;
    }

    void StartCallback(const std_msgs::msg::Int32::SharedPtr msg) {

        static int prev_msg = -1;

        if (msg->data == 1) {
            if (prev_msg == 0) {
                start = true;
            }
        }

        prev_msg = msg->data;
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr start_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr time_pub;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr start_sub;
    rclcpp::Service<btcpp_ros2_interfaces::srv::StartUpSrv>::SharedPtr srv;

    int team;
    char plan;
    int ready_feedback = 1;
    int st_point[6];
    int start_point;
    bool ready = false;
    bool pub_ready = false;
    bool start = true;
    double starting_time = 0;
    StartUpState start_up_state;
    geometry_msgs::msg::PointStamped start_position;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StartUp>());
    rclcpp::shutdown();
    return 0;
}