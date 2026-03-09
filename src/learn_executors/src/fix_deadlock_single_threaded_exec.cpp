#include "learn_executors/fix_deadlock_single_threaded_exec.hpp"
// #include "tracy/Tracy.hpp"

FixDeadlockSingleThreadedExec::FixDeadlockSingleThreadedExec(const rclcpp::NodeOptions &options) : rclcpp::Node("fix_deadlock_single_threaded_exec_", options)
{
    cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    client_ = create_client<std_srvs::srv::Empty>("is_alive", rclcpp::ServicesQoS(), cb_group_);
    client_->wait_for_service();
    client_executor_.add_callback_group(cb_group_, get_node_base_interface());
    timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&FixDeadlockSingleThreadedExec::timerCallback, this), cb_group_);
    RCLCPP_INFO(get_logger(), "Client started");
}

void FixDeadlockSingleThreadedExec::timerCallback()
{
    RCLCPP_INFO(get_logger(), "Sending request");
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    auto future = client_->async_send_request(request);
    while (future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
        client_executor_.spin_some();
        // create the additional executor and spin it to gain control over the client's callback,
        // bringing it into the same thread as the timer callback and thus avoiding the deadlock
    }
    RCLCPP_INFO(get_logger(), "Got response from the server!");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FixDeadlockSingleThreadedExec>(rclcpp::NodeOptions());
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}