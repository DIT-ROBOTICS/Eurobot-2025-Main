#include <rclcpp/rclcpp.hpp>
// register node
#include <rclcpp_components/register_node_macro.hpp>
//tf broadcast
#include <tf2_ros/transform_broadcaster.h>
//tf listener
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
// message
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace bt_app_test{

class TestLocServer : public rclcpp::Node {
public:
    explicit TestLocServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("test_loc_server", options), buffer_(this->get_clock()), listener_(buffer_)
    {
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        timer_1_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&TestLocServer::broadcastTransform, this)
        );
        timer_2_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TestLocServer::lookupTransform, this)
        );
    }

private:
    void broadcastTransform() {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "robot/map";
        transform.child_frame_id = "robot/base_footprint";

        transform.transform.translation.x = -0.25;
        transform.transform.translation.y = -0.7;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.7071;
        transform.transform.rotation.w = 0.7071;

        broadcaster_->sendTransform(transform);

        std::cout << "broadcast:(" << transform.transform.translation.x << ", " << transform.transform.translation.y << ")\n";
    }
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_1_;

    void lookupTransform() {
        try {
            auto transform = buffer_.lookupTransform(
                "robot/map", 
                "robot/base_footprint", 
                tf2::TimePointZero
            );
            RCLCPP_INFO(this->get_logger(), "Transform: x=%f, y=%f, z=%f", 
                        transform.transform.translation.x, 
                        transform.transform.translation.y, 
                        transform.transform.translation.z);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Failed to lookup transform: %s", ex.what());
        }
    }

    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    rclcpp::TimerBase::SharedPtr timer_2_;
};
}
// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<TestLocServer>());
//     rclcpp::shutdown();
//     return 0;
// }

RCLCPP_COMPONENTS_REGISTER_NODE(bt_app_test::TestLocServer)
