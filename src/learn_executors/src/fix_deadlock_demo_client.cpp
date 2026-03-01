#include "learn_executors/fix_deadlock_demo_client.hpp"
// #include "tracy/Tracy.hpp"

FixDeadlockDemoClient::FixDeadlockDemoClient(const rclcpp::NodeOptions &options) : rclcpp::Node("fix_deadlock_demo_client", options)
{
    client_ = create_client<std_srvs::srv::Empty>("is_alive");
    client_->wait_for_service();
    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&FixDeadlockDemoClient::timerCallback, this), cb_group_);
    RCLCPP_INFO(get_logger(), "Client started");
}

void FixDeadlockDemoClient::timerCallback()
{
    RCLCPP_INFO(get_logger(), "Sending request");
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto future = client_->async_send_request(request);
    future.wait();
    RCLCPP_INFO(get_logger(), "Got response from the server!");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FixDeadlockDemoClient>(rclcpp::NodeOptions());
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}