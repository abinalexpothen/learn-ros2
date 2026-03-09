#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

// idea is to assign timer_ to a separate callback group

class FixDeadlockSingleThreadedExec : public rclcpp:: Node
{
    public:
    FixDeadlockSingleThreadedExec(const rclcpp::NodeOptions &options);

    private:
    void timerCallback();

    rclcpp::CallbackGroup::SharedPtr cb_group_;
    // add an additional single threaded executor to out node and associatre with out callback group
    rclcpp::executors::SingleThreadedExecutor client_executor_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_;
};