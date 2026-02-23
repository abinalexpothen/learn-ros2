#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

class DeadlockDemoClient : public rclcpp::Node
{
    public:
    DeadlockDemoClient(rclcpp::NodeOptions &options);
    
    private:
    void timerCallback();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_;
};