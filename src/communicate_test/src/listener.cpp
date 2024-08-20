#include <functional>
#include <memory>
#include <string>
#include "serial/serial.h"
#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

serial::Serial ros_ser;
class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber() : Node("minimal_sync_subscriber")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }
 
private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        
        unsigned char buffer[1] = {0};    //我这里每次只需发送一个字符&#xff0c;所以就定义了长度为1的数 
                                        // 组&#xff0c;这个可以根据需求自己定义

        if (msg->data == "A")     //当接收到发布者"A"字符串时向串口发送"A"字符&#xff08;0x41代表字符"A"&#xff09;
        {
            buffer[0] = 0x41;
            //buffer[1] = 0x00;
            //buffer[2] = 0x00;
            //buffer[3] = 0x00;
            //buffer[4] = 0x00;
            //buffer[5] = 0x00;
            //buffer[6] = 0x00;
            //buffer[7] = 0x00;
        }
        else                   //否则向串口发送"B"字符&#xff08;0x42代表字符"B"&#xff09;
        {
            buffer[0] = 0x42;
            //buffer[1] = 0x00;
            //buffer[2] = 0x00;
            //buffer[3] = 0x00;
            //buffer[4] = 0x00;
            //buffer[5] = 0x00;
            //buffer[6] = 0x00;
            //buffer[7] = 0x00;
        }
        RCLCPP_INFO(get_logger(), "result: %s", msg->data.c_str());
        for (int i = 0; i < 1; i++)        //因为我就发送一个字符&#xff0c;所以i上限定为1&#xff0c;这个也根据需求 
        {                                  //自己定义
        std::cout << std::hex << (buffer[i] &0xff)<< " ";
        }
        std::cout<<std::endl;
        ros_ser.write(buffer,1);         //因为我就发送一个字符&#xff0c;所以发送长度定为1&#xff0c;这个也根据需求 
    }                                  //自己定义
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
 
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    ros_ser.setPort("/dev/ttyUSB0");            //设置端口号
    ros_ser.setBaudrate(115200);                //设置波特率
    serial::Timeout to =serial::Timeout::simpleTimeout(1000);
    ros_ser.setTimeout(to);
    try
    {
        ros_ser.open();
    }
    catch(serial::IOException &e)
    {
        std::cout<<"unable to open"<<std::endl;     //若打开串口失败打印"unable to open"到终端
        return -1;
    }
    if(ros_ser.isOpen())
    {
        std::cout<<"open"<<std::endl;              //若打开串口打印"open"到终端
    }
    else
    {
        return -1;
    }
    
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    ros_ser.close();
    return 0;
}