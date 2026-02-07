#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "master_ros2_interface/action/my_custom_action.hpp"

class ActionClientNode : public rclcpp::Node
{
public:
    ActionClientNode() : Node("action_client_node")
    {
        client_ = rclcpp_action::create_client<master_ros2_interface::action::MyCustomAction>(this, "my_custom_action");
        send_goal();
    }

    void send_goal()
    {
        if (!client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        auto goal_msg = master_ros2_interface::action::MyCustomAction::Goal();
        goal_msg.goal_value = 10;

        auto send_goal_options = rclcpp_action::Client<master_ros2_interface::action::MyCustomAction>::SendGoalOptions();

        send_goal_options.feedback_callback = [this](rclcpp_action::ClientGoalHandle<master_ros2_interface::action::MyCustomAction>::SharedPtr,
                                                     const std::shared_ptr<const master_ros2_interface::action::MyCustomAction::Feedback> feedback)
        {
            RCLCPP_INFO(this->get_logger(), "Received feedback: %f", feedback->progress);
        };

        send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<master_ros2_interface::action::MyCustomAction>::WrappedResult &result)
        {
            RCLCPP_INFO(this->get_logger(), "Received result: %d", result.result->result_value);
        };

        client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<master_ros2_interface::action::MyCustomAction>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActionClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}