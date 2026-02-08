/* A node created as a component can optionally be controlled 
with an executor, which can run multiple components. */

#include "rclcpp/rclcpp.hpp"

namespace our_namespace 
{
    class OurClass : public rclcpp::Node 
    {
        public:
        OurClass(const rclcpp::NodeOptions & options) : Node("node_init_components", options)
        {
            // some constructor
        }
    };
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(our_namespace::OurClass)

