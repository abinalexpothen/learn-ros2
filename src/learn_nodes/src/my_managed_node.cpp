#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class MyManagedNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    MyManagedNode() : rclcpp_lifecycle::LifecycleNode("my_managed_node")
    {
        // some constructor
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyManagedNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}