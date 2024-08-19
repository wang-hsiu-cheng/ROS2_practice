#include "rclcpp/rclcpp.hpp"
#include "basic_function/srv/example.hpp"     // CHANGE

#include <memory>

void add(const std::shared_ptr<basic_function::srv::example::Request> request,     // CHANGE
          std::shared_ptr<basic_function::srv::example::Response>       response)  // CHANGE
{
  response->sum = request->a + request->b + request->c;                                       // CHANGE
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %.6g" " b: %.6g" " c: %.6g",   // CHANGE
                request->a, request->b, request->c);                                          // CHANGE
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%.6g]", (double)response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_doubles_server");  // CHANGE

  rclcpp::Service<basic_function::srv::example>::SharedPtr service = node->create_service<basic_function::srv::example>("add_three_doubles",  &add);     // CHANGE

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add three doubles.");      // CHANGE

  rclcpp::spin(node);
  rclcpp::shutdown();
}