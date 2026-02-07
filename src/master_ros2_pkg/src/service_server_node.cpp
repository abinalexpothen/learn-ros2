#include "rclcpp/rclcpp.hpp"
#include "master_ros2_interface/srv/concat_strings.hpp"

class ServiceServerNode : public rclcpp::Node
{
public:
    ServiceServerNode() : Node("service_server_node")
    {
        service_ = this->create_service<master_ros2_interface::srv::ConcatStrings>(
            "concat_strings",
            std::bind(&ServiceServerNode::handle_service, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void handle_service(const std::shared_ptr<master_ros2_interface::srv::ConcatStrings::Request> request,
                        std::shared_ptr<master_ros2_interface::srv::ConcatStrings::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received request: str1='%s', str2='%s'", request->str1.c_str(), request->str2.c_str());
        response->concatenated_str = request->str1 + request->str2;
        RCLCPP_INFO(this->get_logger(), "Sending response: concatenated_str='%s'", response->concatenated_str.c_str());
    }

    rclcpp::Service<master_ros2_interface::srv::ConcatStrings>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServiceServerNode>());
    rclcpp::shutdown();
    return 0;
}