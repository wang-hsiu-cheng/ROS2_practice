#include "rclcpp/rclcpp.hpp"
#include "basic_function/srv/example.hpp"        // CHANGE

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 4) { // CHANGE
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_three_doubles_client X Y Z");      // CHANGE
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_three_doubles_client"); // CHANGE
  rclcpp::Client<basic_function::srv::Example>::SharedPtr client = node->create_client<basic_function::srv::Example>("add_three_doubles");                  // CHANGE

  // setup request service
  auto request = std::make_shared<basic_function::srv::Example::Request>();               // CHANGE
  request->a = atoll(argv[1]);
  request->b = atoll(argv[2]);
  request->c = atoll(argv[3]);               // CHANGE

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  // send request to the server
  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %.6g", result.get()->sum);
  } 
  else 
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_three_doubles");    // CHANGE
  }

  rclcpp::shutdown();
  return 0;
}