#include "bt_app/bt_action_node_lib.h"
#include "bt_app/bt_decorator_node_lib.h"
#include "bt_app/bt_utils_node_lib.h"

int main(int argc, char *argv[]) {

    // Initialize ROS
    ros::init(argc, argv, "bt_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_local("~");

    // =============================
    // === Kernel Initialization ===
    // =============================

    // Create the kernel
    auto kernel = std::make_shared<Kernel>(nh, nh_local);

    // ==============================
    // === Behavior Tree Creation ===
    // ==============================
    
    // Create the BT factory
    BT::BehaviorTreeFactory factory;

    // Register the custom node 
    // TODO: Add the custom node
    factory.registerNodeType<Testing>("Testing");
    factory.registerNodeType<SimpleNavigation>("SimpleNavigation", kernel);
    factory.registerNodeType<TickFlow>("TickFlow");
    factory.registerNodeType<GeneratePathPoint>("GeneratePathPoint");
    factory.registerNodeType<BT::LoopNode<geometry_msgs::TwistStamped>>("LoopWayPoint");
    factory.registerNodeType<ElapseTimeCheck>("ElapseTimeCheck", kernel);
    factory.registerNodeType<StartRace>("StartRace", kernel);
    factory.registerNodeType<RaceTimeCheck>("RaceTimeCheck", kernel);

    // Write tree nodes model
    std::string xml_tree_model = BT::writeTreeNodesModelXML(factory);
    std::ofstream file("/home/user/groot_ws/src/bt_app/bt_config/bt_tree_node_model.xml");
    file << xml_tree_model;
    file.close();

    // Create the tree from the XML file
    std::string groot_xml_config_directory = "/home/user/groot_ws/src/bt_app/bt_config/";
    for (auto const& entry : std::filesystem::directory_iterator(groot_xml_config_directory)) {
        if (entry.path().extension() == ".xml") {
            factory.registerBehaviorTreeFromFile((groot_xml_config_directory + "/" + entry.path().filename().string()).c_str());
        }
    }

    // Create the tree
    auto tree = factory.createTree("Waypoint-Demo");


    // ======================
    // === Node Execution ===
    // ======================

    // TODO: Change the rate
    ros::Rate rate(10);

    // Use Grove2Publisher to connecting to Groot
    // TODO: Change the port number
    BT::Groot2Publisher publisher(tree, 2227);

    // Run
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (ros::ok() && status == BT::NodeStatus::RUNNING) {
        status = tree.rootNode()->executeTick();
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}