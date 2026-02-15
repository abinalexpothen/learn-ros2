/* Managed node 1 for the learn nodes exercise */

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace learn_nodes_exercise 
{
    class LearnNodesManaged2 : public rclcpp_lifecycle::LifecycleNode
    {
        public:
        LearnNodesManaged2(const rclcpp::NodeOptions &options) : rclcpp_lifecycle::LifecycleNode("learn_nodes_managed_1", options)
        {
            // some constructor
        }
    };
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(learn_nodes_exercise::LearnNodesManaged2)