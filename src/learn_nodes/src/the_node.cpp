/* Some serious deep dive into the mechanics of ROS2 */

#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("best_node_name");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}