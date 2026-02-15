/* learn_nodes_node - a node that accepts a service request and initialize two managed nodes sequentially

 This needs to have a service server so it can receive the message, take action and send a response back

*/

#include "rclcpp/rclcpp.hpp"
#include "learn_nodes/srv/learn_nodes_exercise_service.hpp"

namespace learn_nodes_exercise
{
    class LearnNodesNode : public rclcpp::Node
    {
        public:
        LearnNodesNode(const rclcpp::NodeOptions &options) : Node("learn_nodes_node", options)
        {
            service_  = this->create_service<learn_nodes::srv::LearnNodesExerciseService>("learn_nodes_command", std::bind(&LearnNodesNode::handle_service, this, std::placeholders::_1, std::placeholders::_2));
        }

        private:

        void handle_service(const std::shared_ptr<learn_nodes::srv::LearnNodesExerciseService::Request> request, std::shared_ptr<learn_nodes::srv::LearnNodesExerciseService::Response> response)
        {
            RCLCPP_INFO(this->get_logger(), "Received request: %s", request->command.c_str());

            // do something based on the request

            RCLCPP_INFO(this->get_logger(), "Sending response: %s", response->response.c_str());
        }

        rclcpp::Service<learn_nodes::srv::LearnNodesExerciseService>::SharedPtr service_;
    };
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(learn_nodes_exercise::LearnNodesNode)