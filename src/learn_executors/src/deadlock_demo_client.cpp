#include "learn_executors/deadlock_demo_client.hpp"
#include "tracy/Tracy.hpp"

using namespace std::chrono_literals;

DeadlockDemoClient::DeadlockDemoClient(const rclcpp::NodeOptions &options) : rclcpp::Node("deadlock_demo_client", options)
{
    ZoneScoped;
    ZoneName("ClientConstructor", 17);
    client_ = create_client<std_srvs::srv::Empty>("is_alive");
    client_->wait_for_service();
    timer_ = create_wall_timer(500ms, std::bind(&DeadlockDemoClient::timerCallback, this));
    RCLCPP_INFO(get_logger(), "Client started");
}

void DeadlockDemoClient::timerCallback()
{
    ZoneScoped;
    RCLCPP_INFO(get_logger(), "Sending request");
    std::shared_ptr request = std::make_unique<std_srvs::srv::Empty::Request>();
    auto future_result = client_->async_send_request(request);
    future_result.wait();
    RCLCPP_INFO(get_logger(), "Got response from the server!");
}

int main(int argc, char** argv)
{
    while (!TracyIsConnected) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    ZoneScoped;
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DeadlockDemoClient>(rclcpp::NodeOptions());
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}