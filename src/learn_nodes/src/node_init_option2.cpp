#include "rclcpp/rclcpp.hpp"

/* Why this option 2? Reusability -> easy to isolate into a lib
independent from the execution of a process.

*/

class OurClass : public rclcpp::Node {
    public:
    OurClass() : Node("node_init_option2"){
        // some constructor
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OurClass>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}