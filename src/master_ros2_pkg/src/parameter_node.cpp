#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ParameterNode : public rclcpp::Node
{
    public:
    ParameterNode(): Node("parameter_node")
    {
        // declare parameters
        this->declare_parameter<std::string>("string_parameter", "Default String");
        this->declare_parameter<int>("integer_parameter", 42);

        // create a parameter subscriber
        param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

        // set a callback for this node's parameters
        auto cb = [this](const rclcpp::Parameter &p)
        {
            RCLCPP_INFO(this->get_logger(), "cb: Parameter '%s' has been updated to '%s'", p.get_name().c_str(), p.value_to_string().c_str());
        };
        
        cb_string_param_ =  param_subscriber_->add_parameter_callback("string_parameter", cb);
        cb_int_param_ = param_subscriber_->add_parameter_callback("integer_parameter", cb);

        // parameters to change every 1 second
        timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
            static int counter = 0;
            this->set_parameters("Updated String " + std::to_string(counter), counter);
            counter++;
        });

    }

    void set_parameters(std::string new_string, int new_int)
    {
        // set parameters to trigger callbacks
        this->set_parameter(rclcpp::Parameter("string_parameter", new_string));
        this->set_parameter(rclcpp::Parameter("integer_parameter", new_int));
    }

    private:
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_string_param_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_int_param_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParameterNode>());
    rclcpp::shutdown();
    return 0;
}