#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "master_ros2_interface/msg/custom_msg.hpp"

class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode() : Node("subscriber_node")
    {
        // QoS profile for both subscribers
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10))
                               .reliable()
                               .transient_local();

        // subscribe to the topic "std_string_topic" with the QoS profile
        subscription_ = this->create_subscription<std_msgs::msg::String>("std_string_topic", qos_profile, std::bind(&SubscriberNode::string_callback, this, std::placeholders::_1));

        // subscribe to the topic "custom_msg_topic" with the QoS profile
        custom_subscription_ = this->create_subscription<master_ros2_interface::msg::CustomMsg>("custom_msg_topic", qos_profile, std::bind(&SubscriberNode::custom_msg_callback, this, std::placeholders::_1));
    }

private:
    void string_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
    }

    void custom_msg_callback(const master_ros2_interface::msg::CustomMsg::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "received custom message: %s, number: %d", msg->data.c_str(), msg->number);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Subscription<master_ros2_interface::msg::CustomMsg>::SharedPtr custom_subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberNode>());
    rclcpp::shutdown();
    return 0;
}