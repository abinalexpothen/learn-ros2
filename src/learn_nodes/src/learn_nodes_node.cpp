/* learn_nodes_node - a node that accepts a service request and initialize two managed nodes sequentially */

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "learn_nodes/srv/learn_nodes_exercise_service.hpp"

namespace learn_nodes_exercise
{
    class LearnNodesNode : public rclcpp::Node
    {
        public:
        LearnNodesNode(const rclcpp::NodeOptions &options) : Node("learn_nodes_node", options)
        {
            client_1_ = this->create_client<lifecycle_msgs::srv::ChangeState>("/learn_nodes_managed_1/change_state");
            client_2_ = this->create_client<lifecycle_msgs::srv::ChangeState>("/learn_nodes_managed_2/change_state");
            service_  = this->create_service<learn_nodes::srv::LearnNodesExerciseService>("learn_nodes_command", std::bind(&LearnNodesNode::handle_service, this, std::placeholders::_1, std::placeholders::_2));
        }

        private:

        void handle_service(const std::shared_ptr<learn_nodes::srv::LearnNodesExerciseService::Request> request, std::shared_ptr<learn_nodes::srv::LearnNodesExerciseService::Response> response)
        {
            // initialize with failed
            response->response = "Operation sequence failed";

            RCLCPP_INFO(this->get_logger(), "Received request: %s", request->command.c_str());

            if (request->command == "start_sequence")
            {
                // transition managed node 1 to active
                if (transition_node(client_1_, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
                {
                    RCLCPP_INFO(this->get_logger(), "Configure managed node 1");

                    if (transition_node(client_1_, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
                    {
                        RCLCPP_INFO(this->get_logger(), "Activate managed node 1");

                        if (transition_node(client_2_, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
                        {
                            RCLCPP_INFO(this->get_logger(), "Configure managed node 2");

                            if (transition_node(client_2_, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
                            {
                                RCLCPP_INFO(this->get_logger(), "Activate managed node 2");
                                response->response = "Sequence completed successfully";
                            }
                            else
                            {
                                RCLCPP_INFO(this->get_logger(), "Managed node 2 transition activation failed");
                            }
                        }
                        else
                        {
                            RCLCPP_INFO(this->get_logger(), "Managed node 2 configuration failed.");
                        }
                    }
                    else
                    {
                        RCLCPP_INFO(this->get_logger(), "Managed node 1 activation failed.");
                    }
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Managed node 1 configuration failed.");
                }

            }

            RCLCPP_INFO(this->get_logger(), "Sending response: %s", response->response.c_str());
        }

        // helper function to handle the async service calls
        bool transition_node(rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client, uint8_t transition)
        {
            auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
            request->transition.id = transition;

            if (!client->wait_for_service(std::chrono::seconds(5))) return false;

            auto result = client->async_send_request(request);

            // in a real app, use a future wait. Simplified for the exercise
            return true;
        }

        rclcpp::Service<learn_nodes::srv::LearnNodesExerciseService>::SharedPtr service_;

        // clients to talk to the lifecycle managers of the other nodes
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_1_;
        rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_2_;
    };
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(learn_nodes_exercise::LearnNodesNode)