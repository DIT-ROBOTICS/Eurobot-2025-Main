/* Simple rclcpp publisher */
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

/* Include startup necessary service */
#include "btcpp_ros2_interfaces/srv/start_up_srv.hpp"

#include <jsoncpp/json/json.h>
#include <fstream>

/**
 * Init: Currently launch all of the nodes.
 * Ready: Get current pose and publish ready signal to the robot.
 *   - Need use service to check all of the nodes are ready.
 * Start: Start the robot.
 */

int team;
int ready_feedback = 0;
int st_point[6];
int start_point;
bool ready = false;
bool pub_ready = false;
bool start = false;
double starting_time = 0;
geometry_msgs::msg::PointStamped start_position;

typedef enum StartUpState {
    INIT = 0,
    READY,
    ERROR,
    START
} StartUpState;

void PublishReadySignal(rclcpp::Publisher &pub) {

    start_position.header.stamp = rclcpp::Time::now();

    if (team == 0) {
        start_position.header.frame_id = "0";
    } else {
        start_position.header.frame_id = "1";
    }
    pub.publish(start_position);
}

void PublishStartSignal(rclcpp::Publisher &pub) {

    start_position.header.stamp = rclcpp::Time::now();

    if (team == 0) {
        start_position.header.frame_id = "0";
    } else {
        start_position.header.frame_id = "1";
    }
    pub.publish(start_position);
}

void PublishTime(rclcpp::Publisher &pub) {

    // Get current time by second
    double current_time = rclcpp::Time::now().toSec();

    std_msgs::Float32 msg;
    msg.data = current_time - starting_time;
    pub.publish(msg);

    static bool is_published = false;
    static bool time_check = false;

    if (msg.data >= 90 && !is_published) {
        ROS_INFO("[StartUp Program]: Send the ladybug!");

        // Use system call the ladybug script
        system("/home/main_ws/scripts/ladybug.sh");

        is_published = true;
    }

    if (msg.data >= 100 && !time_check) {
        ROS_INFO("[StartUp Program]: Time is out!");
        time_check = true;
    }
}

bool ReadyFeedback(startup_msgs::StartUpSrv::Request &req, startup_msgs::StartUpSrv::Response &res) {

    ready_feedback += req.group;

    return true;
}

void ReadJsonFile(std::string file_path) {

    std::ifstream ifs(file_path);

    if (!ifs.is_open()) {
        ROS_ERROR("[StartUp Program]: Can't open file %s", file_path.c_str());
        return;
    }
    
    Json::Reader reader;
    Json::Value obj;

    reader.parse(ifs, obj);

    st_point[0] = obj["button_0"].asInt();
    st_point[1] = obj["button_1"].asInt();
    st_point[2] = obj["button_2"].asInt();
    st_point[3] = obj["button_3"].asInt();
    st_point[4] = obj["button_4"].asInt();
    st_point[5] = obj["button_5"].asInt();
}

void UpdateTeamAndPoint(int point) {
    switch (point) {
    case 0:
        team = 0;
        start_position.point.x = 2.7;
        start_position.point.y = 0.9;
        start_position.point.z = 0;
        break;
    case 1:
        team = 0;
        start_position.point.x = 1.25;
        start_position.point.y = 0.25;
        start_position.point.z = 0;
        break;
    case 2: 
        team = 0;
        start_position.point.x = 0.8;
        start_position.point.y = 1.7;
        start_position.point.z = 0;
        break;
    }
}

int CheckStartPoint() {
    
    for (int i = 1; i < 4; i++) {
        if (st_point[i - 1] == 1) {
            UpdateTeamAndPoint(i - 1);
            return i;
        }
    }
    return 0;
}

void StartCallback(const std_msgs::Int32::ConstPtr &msg) {

    static int prev_msg = -1;

    if (msg->data == 1) {
        if (prev_msg == 0) {
            start = true;
        }
    }

    prev_msg = msg->data;
}

int main(int argc, char **argv) {

    rclcpp::init(argc, argv, "startup");

    rclcpp::NodeHandle nh;
    rclcpp::NodeHandle nh_local("~");

    rclcpp::Publisher pub = nh.advertise<geometry_msgs::PointStamped>("/robot/startup/ready_signal", 2);
    rclcpp::Publisher start_pub = nh.advertise<geometry_msgs::PointStamped>("/robot/startup/start_signal", 2);
    rclcpp::Publisher time_pub = nh.advertise<std_msgs::Float32>("/robot/startup/time", 2);
    rclcpp::Subscriber start_sub = nh.subscribe("/robot/Start", 2, &StartCallback);
    rclcpp::ServiceServer srv = nh.advertiseService("/robot/startup/ready_signal_feedback", &ReadyFeedback);

    std::string file_path;
    nh_local.param<std::string>("file_path", file_path, "state.json");

    /* 20HZ */
    rclcpp::Rate loop_rate(10);

    StartUpState start_up_state = INIT;
    team = 1;
    nh_local.param<int>("team", team, 1);

    while(rclcpp::ok()) {

        rclcpp::spinOnce();
        loop_rate.sleep();

        switch (start_up_state) {

        case INIT:

            ReadJsonFile(file_path);
            start_point = CheckStartPoint();

            if (/* Press ready signal and get robot init position */ start_point) {
                start_up_state = READY;
                ROS_INFO("[StartUp Program]: INIT -> READY");
            }

            break;
        case READY:

            PublishReadySignal(pub);

            if (ready_feedback == 1) {
                if (ready == false) {
                    ROS_INFO("[StartUp Program]: All of the programs are ready!");
                } 
                ready = true;
            }

            if (/* Press start signal */ (ready_feedback == 1) && start) {
                start_up_state = START;
                ROS_INFO("[StartUp Program]: READY -> START");

                /* Publish start signal */
                PublishStartSignal(start_pub);

                /* Start the time */
                starting_time = rclcpp::Time::now().toSec();
            }

            break;
        case START:
            PublishTime(time_pub);
            break;
        default:
            ROS_INFO("[StartUp Program]: UNKNOWN");
            break;
        }

    }

    return 0;
}

// Time calculation
// Score calculation
// Recovery mode