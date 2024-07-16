#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

int message = 0;
void topic_callback(const std_msgs::msg::Int64::SharedPtr msg) {
    printf("test\n");
    printf("%ld\n",  msg->data);
    message = (int)msg->data;
    // RCLCPP_INFO(node->get_logger(), "%d %ld\n", message, msg->data);
}
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);   // 初始化ROS
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscription_;
    
    // 創建一個叫做 sub_name 的 Node 
    auto node = rclcpp::Node::make_shared("sub_name");  
    subscription_ = node->create_subscription<std_msgs::msg::Int64>("number", 10, topic_callback);
    
    // use rate to loop at 1Hz
    rclcpp::WallRate loop_rate(1);

    // 讓Node持續運行
    while(rclcpp::ok()) {
        rclcpp::spin_some(node);
        RCLCPP_INFO(node->get_logger(), "Subsccribing: %d", message);
        loop_rate.sleep();
    }
    
    // 關閉ROS
    rclcpp::shutdown();
    
    return 0;
}