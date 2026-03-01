#include "learn_executors/demo_server.hpp"
// #include "tracy/Tracy.hpp"

DemoServer::DemoServer(const rclcpp::NodeOptions &options) : rclcpp::Node("demo_server", options)
{
    // ZoneScoped;
    // ZoneName("ServerConstructor", 18);
    service_ = create_service<std_srvs::srv::Empty>("is_alive", std::bind(&DemoServer::callbackFunction, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "Service started");
}

void DemoServer::callbackFunction([[maybe_unused]] std_srvs::srv::Empty::Request::SharedPtr srv_request, [[maybe_unused]] std_srvs::srv::Empty::Response::SharedPtr srv_response)
{
    // ZoneScoped;
    RCLCPP_INFO(get_logger(), "Request received!");
}

int main(int argc, char** argv)
{
    // while (!TracyIsConnected) {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }

    // ZoneScoped;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DemoServer>(rclcpp::NodeOptions());
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}