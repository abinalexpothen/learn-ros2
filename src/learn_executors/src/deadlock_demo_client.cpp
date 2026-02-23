#include "learn_executors/deadlock_demo_client.hpp"

using namespace std::chrono_literals;

DeadlockDemoClient::DeadlockDemoClient(rclcpp::NodeOptions &options) : rclcpp::Node("deadlock_demo_client", options)
{
    client_ = create_client<std_srvs::srv::Empty>("is_alive");
    client_->wait_for_service();
    timer_ = create_wall_timer(500ms, std::bind(&DeadlockDemoClient::timerCallback, this));
    RCLCPP_INFO(get_logger(), "Client started");
}

void DeadlockDemoClient::timerCallback()
{
    RCLCPP_INFO(get_logger(), "Sending request");
    std::shared_ptr request = std::make_unique<std_srvs::srv::Empty::Request>();
    auto future_result = client_->async_send_request(request);
    future_result.wait();
    RCLCPP_INFO(get_logger(), "Got response from the server!");
}