#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);   // 初始化ROS
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
    
    // 創建一個叫做 pub_name 的 Node 
    auto node = rclcpp::Node::make_shared("pub_name");  
    publisher_ = node->create_publisher<std_msgs::msg::Int64>("number", 10);
    int message = 0;
    
    // 用Node的get_logger() function來print出Hello World!
    RCLCPP_INFO(node->get_logger(), "Hello World!");
    
    // use rate to loop at 1Hz
    rclcpp::WallRate loop_rate(1);

    // 讓Node持續運行
    while(rclcpp::ok()) {
        rclcpp::spin_some(node);
        // RCLCPP_INFO(node->get_logger(), "Hello World in Loop!");
        publisher_->publish(message++);
        RCLCPP_INFO(node->get_logger(), "Publishing: %d", message);
        loop_rate.sleep();
    }
    
    // 關閉ROS
    rclcpp::shutdown();
    
    return 0;
}