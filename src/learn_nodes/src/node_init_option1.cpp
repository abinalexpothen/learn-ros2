/* Some serious deep dive into the mechanics of ROS2 */

#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("node_init_option1"); // <-- node created as a shared pointer

    /* the node shared pointer can be used to read params, create publishers, subscribers, etc.
    It can also be shared with other functions or classes who can optionally store a copy of it -> although 
    this can be problematic. */

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}