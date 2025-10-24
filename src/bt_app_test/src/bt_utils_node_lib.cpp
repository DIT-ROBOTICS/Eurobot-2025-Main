#include "bt_app_test/bt_utils_node_lib.h"

BT::PortsList GeneratePathPoint::providedPorts() {
    return {
        BT::OutputPort<BT::SharedQueue<geometry_msgs::msg::TwistStamped>>("path_point")
    };
}

BT::NodeStatus GeneratePathPoint::tick() {

    auto shared_queue = std::make_shared<std::deque<geometry_msgs::msg::TwistStamped>>();

    // Generate the path points
    for (int i = 1; i < 10; i++) {
        geometry_msgs::msg::TwistStamped path_point;
        path_point.twist.linear.x = i;
        path_point.twist.linear.y = i;
        shared_queue->push_back(path_point);
    }

    // Set the output port
    setOutput("path_point", shared_queue);

    return BT::NodeStatus::SUCCESS;
}