#include "learn_executors/deadlock_demo_server.hpp"
#include "tracy/Tracy.hpp"

DeadlockDemoServer::DeadlockDemoServer(const rclcpp::NodeOptions &options) : rclcpp::Node("deadlock_demo_server", options)
{
    ZoneScoped;
    ZoneName("ServerConstructor", 18);
    service_ = create_service<std_srvs::srv::Empty>("is_alive", std::bind(&DeadlockDemoServer::callbackFunction, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "Service started");
}

void DeadlockDemoServer::callbackFunction([[maybe_unused]] std_srvs::srv::Empty::Request::SharedPtr srv_request, [[maybe_unused]] std_srvs::srv::Empty::Response::SharedPtr srv_response)
{
    ZoneScoped;
    RCLCPP_INFO(get_logger(), "Request received!");
}

int main(int argc, char** argv)
{
    while (!TracyIsConnected) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    ZoneScoped;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DeadlockDemoServer>(rclcpp::NodeOptions());
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}