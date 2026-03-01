#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

// idea is to assign timer_ to a separate callback group

class FixDeadlockDemoClient : public rclcpp:: Node
{
    public:
    FixDeadlockDemoClient(const rclcpp::NodeOptions &options);

    private:
    void timerCallback();

    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_;
};