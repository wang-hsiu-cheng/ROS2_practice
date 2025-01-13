#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

using std::placeholders::_1;

int message = 0;

class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode(): Node("sub_name")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Int64>("number", 10, std::bind(&SubscriberNode::topic_callback, this, _1));
    }

private:
    void topic_callback(const std_msgs::msg::Int64::SharedPtr msg) const {
        message = (int)msg->data;
        printf("I heard: '%ld'\n", msg->data);
        RCLCPP_INFO(this->get_logger(), "I heard: '%ld'", msg->data);
    }

    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberNode>());
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Finish node");
    rclcpp::shutdown();
    
    return 0;
}