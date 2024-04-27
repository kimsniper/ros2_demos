#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/count_until.hpp"

using CountUntil = my_robot_interfaces::action::CountUntil;
using CountUntilGoalHandle = rclcpp_action::ClientGoalHandle<CountUntil>;
using namespace std::placeholders;

class CountUntilClientNode: public rclcpp::Node
{
private:

    rclcpp_action::Client<CountUntil>::SharedPtr _count_until_client;
    rclcpp::TimerBase::SharedPtr _timer;
    CountUntilGoalHandle::SharedPtr _goal_handle;

    void goal_result_callback(const CountUntilGoalHandle::WrappedResult &result)
    {
        auto status = result.code;
        int reached_number = result.result->reached_number;
        
        if(status ==rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "Succeeded");
        }
        else if(status ==rclcpp_action::ResultCode::ABORTED)
        {
            RCLCPP_ERROR(this->get_logger(), "Aborted");
        }
        else if(status ==rclcpp_action::ResultCode::CANCELED)
        {
            RCLCPP_WARN(this->get_logger(), "Canceled");
        }
        else
        {
            /* Do nothing */
        }
        
        RCLCPP_INFO(this->get_logger(), "Result: %d", reached_number);
    }

    void goal_reponse_callback(const CountUntilGoalHandle::SharedPtr &goal_handle)
    {
        if(!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal got rejected");
        }
        else
        {
            this->_goal_handle = goal_handle;
            RCLCPP_INFO(this->get_logger(), "Goal got accepted");
        }
    }

    void goal_feedback_callback(const CountUntilGoalHandle::SharedPtr &goal_handle, const std::shared_ptr<const CountUntil::Feedback> feedback)
    {
        (void)goal_handle;
        int number = feedback->current_number;
        RCLCPP_INFO(this->get_logger(), "got feedback: %d", number);
    }

    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Cancel the goal");
        _count_until_client->async_cancel_goal(_goal_handle);
        _timer->cancel();
    }

public:
    CountUntilClientNode(): Node("count_until_cient")
    {
        _count_until_client = rclcpp_action::create_client<CountUntil>(
            this,
            "count_until"
        );

        RCLCPP_INFO(this->get_logger(), "Action client has been started");
    }

    void send_goal(int target_number, double period)
    {
        /* Wait for action server */
        _count_until_client->wait_for_action_server();

        /* Create a goal */
        auto goal = CountUntil::Goal();
        goal.target_number = target_number;
        goal.period = period;

        /* Add callbacks */
        auto options = rclcpp_action::Client<CountUntil>::SendGoalOptions();
        options.result_callback = std::bind(&CountUntilClientNode::goal_result_callback, this, _1);
        options.goal_response_callback = std::bind(&CountUntilClientNode::goal_reponse_callback, this, _1);
        options.feedback_callback = std::bind(&CountUntilClientNode::goal_feedback_callback, this, _1, _2);

        /* Send the goal */
        RCLCPP_INFO(this->get_logger(), "Sending goal");
        _count_until_client->async_send_goal(goal, options);

        /* Test canceling sequence */
        _timer = this->create_wall_timer(std::chrono::seconds(2), std::bind(&CountUntilClientNode::timer_callback, this));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilClientNode>();
    node->send_goal(7,1);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}