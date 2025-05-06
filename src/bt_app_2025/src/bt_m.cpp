// BT 
#include "behaviortree_ros2/bt_action_node.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/decorators/loop_node.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/blackboard.h"
// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/client.hpp"  // Service client
#include "rclcpp/service.hpp" // Service server
// ros message
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "btcpp_ros2_interfaces/srv/start_up_srv.hpp"
// BTaction nodes
#include "bt_app_2025/bt_nodes_firmware.h"
#include "bt_app_2025/bt_nodes_navigation.h"
#include "bt_app_2025/bt_nodes_others.h"
#include "bt_app_2025/bt_nodes_receiver.h"
#include "bt_app_2025/bt_nodes_util.h"
// C++
#include <memory>
#include <string>

using namespace BT;

class MainClass : public rclcpp::Node {
public:
    MainClass() : Node("bt_app_2025"), rate(100) {}
    std::shared_ptr<rclcpp::Node> get_node() {
        node_ = shared_from_this(); 
        return shared_from_this();                                             // Get a shared pointer to this node
    }
    void timeCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void sendReadySignal();
    void readyCallback(const std_msgs::msg::String::SharedPtr msg);
    void startCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    bool shellCmd(const string &cmd, string &result) {
        char buffer[512];

        result = "";
        FILE* pipe = popen(cmd.c_str(), "r");                                  // Open pipe to file
        if (!pipe) {
            return false;
        }
        while (!feof(pipe)) {                                                  // read till end of process
            if (fgets(buffer, sizeof(buffer), pipe) != NULL)                   // use buffer to read and add to result
                result += buffer;
        }
        pclose(pipe);
        return true;
    }

    void InitParam() { 
        materials_info_.data = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
        mission_points_status_.data = {0, 0, 0, 0, 0, 0, 0, 0};
        // Create a shared blackboard
        blackboard = BT::Blackboard::create();
        blackboard->set<double>("current_time", 0);                            // time from startup program
        blackboard->set<int>("front_materials", 0);                            // record materials on robot
        blackboard->set<int>("back_materials", 0);                             // record materials on robot
        blackboard->set<int>("mission_progress", 0);                           // count the steps of a mission subtree
        blackboard->set<std_msgs::msg::Int32MultiArray>("materials_info", materials_info_);     // the condition of each material point
        blackboard->set<std_msgs::msg::Int32MultiArray>("mission_points_status", mission_points_status_);  // record number of missions have done at each point
        blackboard->set<bool>("last_mission_failed", false);
        blackboard->set<bool>("Timeout", false);
        blackboard->set<std::string>("team", "y");
        blackboard->set<std::string>("bot", "1"); 
        blackboard->set<int>("score_from_main", 0);
        blackboard->set<bool>("enable_vision_check", true);
        blackboard->set<int>("current_index", 0);   
        // Subscriber
        time_sub = this->create_subscription<std_msgs::msg::Float32>("/robot/startup/time", 2, std::bind(&MainClass::timeCallback, this, std::placeholders::_1));
        ready_sub = this->create_subscription<std_msgs::msg::String>("/robot/startup/plan", 2, std::bind(&MainClass::readyCallback, this, std::placeholders::_1));
        ready_srv_client = this->create_client<btcpp_ros2_interfaces::srv::StartUpSrv>("/robot/startup/ready_signal");
        start_srv_server = this->create_service<std_srvs::srv::SetBool>(
            "/robot/startup/start_signal", std::bind(&MainClass::startCallback, this, std::placeholders::_1, std::placeholders::_2));
        // Read parameters
        this->declare_parameter<std::string>("groot_xml_config_directory", "/Eurobot-2025-Main/src/bt_app_2025/bt_m_config/");
        this->declare_parameter<std::string>("tree_node_model_config_file", "/Eurobot-2025-Main/src/bt_app_2025/bt_m_config/bt_m_tree_node_model.xml");
        this->declare_parameter<std::string>("tree_name", "FuncTest");
        this->declare_parameter<std::string>("frame_id", "base_link");
        this->declare_parameter<double>("nav_dist_error", 0.03);
        this->declare_parameter<double>("nav_ang_error", 0.4);
        this->declare_parameter<double>("rotate_dist_error", 0.03);
        this->declare_parameter<double>("rotate_ang_error", 0.4);
        this->declare_parameter<double>("safety_dist", 0.33);
        // MissionFInisher params
        this->declare_parameter<std::vector<int>>("front_collect", std::vector<int>{});
        this->declare_parameter<std::vector<int>>("back_collect", std::vector<int>{});
        this->declare_parameter<std::vector<int>>("construct_1", std::vector<int>{});
        this->declare_parameter<std::vector<int>>("construct_2", std::vector<int>{});
        this->declare_parameter<std::vector<int>>("construct_3", std::vector<int>{});
        // map points
        this->declare_parameter<std::vector<double>>("map_points_1", std::vector<double>{});
        this->declare_parameter<std::vector<double>>("map_points_2", std::vector<double>{});
        // get parameters
        this->get_parameter("groot_xml_config_directory", groot_xml_config_directory);
        this->get_parameter("tree_node_model_config_file", bt_tree_node_model);
        this->get_parameter("tree_name", tree_name);
    }

    void CreateTreeNodes() {
        // action nodes
        params.nh = node_;
        /* receiver */
        factory.registerNodeType<TopicSubTest>("TopicSubTest", params);
        factory.registerNodeType<NavReceiver>("NavReceiver", params, blackboard);
        factory.registerNodeType<CamReceiver>("CamReceiver", params, blackboard);
        /* navigation */
        factory.registerNodeType<StopRobot>("StopRobot", params);
        factory.registerNodeType<MaterialChecker>("MaterialChecker", params, blackboard);
        factory.registerNodeType<MissionChecker>("MissionChecker", params, blackboard);
        params.default_port_value = "dock_robot";
        factory.registerNodeType<Navigation>("Navigation", params);
        factory.registerNodeType<Docking>("Docking", params, blackboard);
        factory.registerNodeType<Rotation>("Rotation", params);
        /* firmware */
        params.default_port_value = "firmware_mission";
        factory.registerNodeType<FirmwareMission>("FirmwareMission", params, blackboard);
        factory.registerNodeType<IntegratedMissionNode>("IntegratedMissionNode", params, blackboard);
        factory.registerNodeType<SIMAactivate>("SIMAactivate", params);
        factory.registerNodeType<MissionStart>("MissionStart", params, blackboard);
        factory.registerNodeType<MissionSuccess>("MissionSuccess", params, blackboard);
        factory.registerNodeType<MissionFailure>("MissionFailure", params, blackboard);
        /* others */
        factory.registerNodeType<BTStarter>("BTStarter", params, blackboard);
        factory.registerNodeType<MySetBlackboard>("MySetBlackboard", params, blackboard);
        factory.registerNodeType<Comparator>("Comparator", params);            // decorator
        factory.registerNodeType<TimerChecker>("TimerChecker", blackboard);    // condition node
        factory.registerNodeType<LoopInt32>("LoopInt32", params);
    }

    void LoadXML() {
        RCLCPP_INFO_STREAM(this->get_logger(), "--Loading XML--");
        while (rclcpp::ok() && !isReady) {
            rate.sleep();
        }
        shellCmd("whoami", user_name);                                         // command to get user name
        user_name.pop_back();
        // register tree xml
        groot_filename = "/home/" + user_name + groot_xml_config_directory + groot_filename;
        RCLCPP_INFO_STREAM(this->get_logger(), groot_filename);
        factory.registerBehaviorTreeFromFile(groot_filename);                  // translate new tree nodes into xml language
        // add new tree nodes into xml
        xml_models = BT::writeTreeNodesModelXML(factory);
        bt_tree_node_model = "/home/" + user_name + bt_tree_node_model;
        std::ofstream file(bt_tree_node_model);                                // open the xml that store the tree nodes
        file << xml_models;
        file.close();
    }

    void RunTheTree() {
        // create tree
        RCLCPP_INFO(node_->get_logger(), "--Create tree--");
        tree = factory.createTree(tree_name, blackboard);
        // BT::Groot2Publisher publisher(tree, 2227);
        BT::NodeStatus status = BT::NodeStatus::RUNNING;
        RCLCPP_INFO_STREAM(this->get_logger(), "i");
        while (rclcpp::ok() && !canStart) {
            rate.sleep();
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "ii");
        RCLCPP_INFO(this->get_logger(), "[BT Application]: Behavior Tree start running!");
        while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
            rate.sleep();
            status = tree.rootNode()->executeTick();
        }
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr time_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ready_sub;
    rclcpp::Client<btcpp_ros2_interfaces::srv::StartUpSrv>::SharedPtr ready_srv_client;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_srv_server;
    
    BT::Blackboard::Ptr blackboard;
    std::shared_ptr<rclcpp::Node> node_;

    BT::Tree tree;
    rclcpp::Rate rate;
    std::string xml_models; // new tree nodes string
    // Behavior Tree Factory
    BT::BehaviorTreeFactory factory;
    BT::RosNodeParams params;
    // ROS msg
    std_msgs::msg::Int32 ready_feedback;
    std_msgs::msg::Int32MultiArray materials_info_, mission_points_status_;
    double game_time = 0.0;
    char team = '0';
    bool isReady = false;
    bool canStart = false;
    std::string groot_filename;
    // Parameters
    std::string groot_xml_config_directory;
    std::string bt_tree_node_model;
    std::string Bot1_YellowA_file, Bot1_YellowB_file, Bot1_YellowC_file, Bot1_YellowS_file, Bot1_BlueA_file, Bot1_BlueB_file, Bot1_BlueC_file, Bot1_BlueS_file;
    std::string Bot2_BlueA_file;
    std::string tree_name;
    std::string user_name;
};

void MainClass::timeCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    game_time = msg->data;
}

void MainClass::sendReadySignal() {
    RCLCPP_INFO_STREAM(this->get_logger(), "send ready signal");

    auto request = std::make_shared<btcpp_ros2_interfaces::srv::StartUpSrv::Request>();
    request->group = 0;
    request->state = 3;

    ready_srv_client->async_send_request(request, 
        [this](rclcpp::Client<btcpp_ros2_interfaces::srv::StartUpSrv>::SharedFuture future) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Response: success=%d, group=%d", int(response->success), response->group);
        }
    );
}

void MainClass::readyCallback(const std_msgs::msg::String::SharedPtr msg) {
    // RCLCPP_INFO_STREAM(this->get_logger(), "start ready callback");
    if (isReady) {
        return;
    }
    team = msg->data.back();                                                   // get team color
    if (team == '0')
        blackboard->set<std::string>("team", "y");                             // team color is yellow
    else
        blackboard->set<std::string>("team", "b");                             // team color is blue
    msg->data.pop_back();                                                      // delete the last char of string 
    groot_filename = msg->data;                                                // the remain string is the plan xml file name
    blackboard->set<std::string>("bot", groot_filename.substr(3, 1)); 
    isReady = true;                                                            // set as received ready message
    // RCLCPP_INFO_STREAM(this->get_logger(), "ready callback");
    sendReadySignal();
}

void MainClass::startCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Received request: %s", request->data ? "true" : "false");
    response->success = true;
    response->message = request->data ? "Enabled" : "Disabled";
    canStart = request->data;
}

int main(int argc, char **argv) {
    // initialize
    rclcpp::init(argc, argv);
    rclcpp::Rate rate(100);
    auto node = std::make_shared<MainClass>();
    node->get_node();
    node->InitParam();
    node->CreateTreeNodes();
    std::thread spin_thread([&]() { rclcpp::spin(node); });                    // create a thread to spin the node
    node->LoadXML();
    node->RunTheTree();
    rclcpp::shutdown();
    return 0;
}
