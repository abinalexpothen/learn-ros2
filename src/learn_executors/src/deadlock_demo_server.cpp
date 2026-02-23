#include "learn_executors/deadlock_demo_server.hpp"

DeadlockDemoServer::DeadlockDemoServer(const rclcpp::NodeOptions &options) : rclcpp::Node("deadlock_demo_server", options)
{
    service_ = create_service<std_srvs::srv::Empty>("is_alive", std::bind(&DeadlockDemoServer::callbackFunction, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "Service started");
}

void DeadlockDemoServer::callbackFunction(std_srvs::srv::Empty::Request::SharedPtr srv_request, std_srvs::srv::Empty::Response::SharedPtr srv_response)
{
    RCLCPP_INFO(get_logger(), "Request received!");
}