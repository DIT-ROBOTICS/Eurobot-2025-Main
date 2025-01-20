#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);   // 初始化ROS
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    
    // 創建一個叫做 pub_name 的 Node 
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("pub_name");  
    publisher_ = node->create_publisher<std_msgs::msg::Int32>("number", 10);
    std_msgs::msg::Int32 message;
    message.data = 0;
    
    // 用Node的get_logger() function來print出Hello World!
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Hello World!");
    
    // use rate to loop at 1Hz
    rclcpp::WallRate loop_rate(1);

    // 讓Node持續運行
    while(rclcpp::ok()) {
        rclcpp::spin_some(node);
        // RCLCPP_INFO(node->get_logger(), "Hello World in Loop!");
        message.data++;
        publisher_->publish(message);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing: %d", (int)message.data);
        loop_rate.sleep();
    }
    
    // 關閉ROS
    rclcpp::shutdown();
    
    return 0;
}