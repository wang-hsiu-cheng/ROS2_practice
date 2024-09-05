#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "basic_function/action/example.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

class ExampleActionClient : public rclcpp::Node
{
public:
  using Example = basic_function::action::Example;
  using GoalHandleExample = rclcpp_action::ClientGoalHandle<Example>;

  explicit ExampleActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("example_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Example>(
      this,
      "example");

    // this->timer_ = this->create_wall_timer(
    //   std::chrono::milliseconds(500),
    //   std::bind(&ExampleActionClient::send_goal, this));
  }

  void send_goal()
  {
    // rclcpp::spin_some(this->get_node_base_interface());
    using namespace std::placeholders;

    // this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = Example::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Example>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&ExampleActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&ExampleActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&ExampleActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Example>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(std::shared_future<GoalHandleExample::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleExample::SharedPtr,
    const std::shared_ptr<const Example::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleExample::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    for (auto number : result.result->sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};  // class ExampleActionClient

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<ExampleActionClient>();

  action_server->send_goal();

  rclcpp::spin(action_server);
  
  rclcpp::shutdown();
  return 0;
}