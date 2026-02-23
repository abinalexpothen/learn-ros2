#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

class DeadlockDemoServer : public rclcpp::Node
{
public:
    DeadlockDemoServer(const rclcpp::NodeOptions &options);

private:
    void callbackFunction(std_srvs::srv::Empty::Request::SharedPtr srv_request, std_srvs::srv::Empty::Response::SharedPtr srv_response);

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_;
};