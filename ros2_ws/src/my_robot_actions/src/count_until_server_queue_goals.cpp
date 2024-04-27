#include <queue>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/count_until.hpp"

using CountUntil = my_robot_interfaces::action::CountUntil;
using CountUntilGoalHandle = rclcpp_action::ServerGoalHandle<CountUntil>;
using namespace std::placeholders;

class CountUntilServerNode: public rclcpp::Node
{
private:

    rclcpp_action::Server<CountUntil>::SharedPtr _count_until_server;
    rclcpp::CallbackGroup::SharedPtr _cb_group;
    std::mutex _mutex;
    std::queue<std::shared_ptr<CountUntilGoalHandle>> _goal_queue;
    std::thread _goal_queue_thread;

    rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const CountUntil::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received a goal");

        if(goal->target_number <= 0)
        {
            RCLCPP_INFO(this->get_logger(), "Rejecting the goal");
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(this->get_logger(), "Accepting the goal");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<CountUntilGoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_callback(const std::shared_ptr<CountUntilGoalHandle> goal_handle)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _goal_queue.push(goal_handle);
        RCLCPP_INFO(this->get_logger(), "Adding goal to queue");
        RCLCPP_INFO(this->get_logger(), "New goal queue size: %d", (int)_goal_queue.size());
    }

    void run_goal_queue_thread()
    {
        rclcpp::Rate loop_rate(1000.0);
        while(rclcpp::ok())
        {
            std::shared_ptr<CountUntilGoalHandle> _next_goal;

            {
                std::lock_guard<std::mutex> lock(_mutex);
                if(_goal_queue.size() > 0)
                {
                    _next_goal = _goal_queue.front();
                    _goal_queue.pop();
                }
            }

            if(_next_goal)
            {
                RCLCPP_INFO(this->get_logger(), "Execute next goal in queue");
                execute_goal(_next_goal);
            }

            loop_rate.sleep();
        }
    }

    void execute_goal(const std::shared_ptr<CountUntilGoalHandle> goal_handle)
    {
        /* Get request from goal */
        int target_number = goal_handle->get_goal()->target_number;
        double period = goal_handle->get_goal()->period;
        int counter = 0;
        auto feedback = std::make_shared<CountUntil::Feedback>();
        auto result = std::make_shared<CountUntil::Result>();

        /* Execute the action */
        rclcpp::Rate loop_rate(1.0/period);
        for(int i = 0;i < target_number;i++)
        {
            if(goal_handle->is_canceling())
            {
                result->reached_number = counter;
                goal_handle->canceled(result);

                return;
            }
            counter++;
            RCLCPP_INFO(this->get_logger(), "%d", counter);
            feedback->current_number = counter;
            goal_handle->publish_feedback(feedback);
            loop_rate.sleep();
        }

        /* Set final state and return result */
        result->reached_number = counter;
        goal_handle->succeed(result);
    }

public:
    CountUntilServerNode(): Node("count_until_server")
    {
        _goal_queue_thread = std::thread(&CountUntilServerNode::run_goal_queue_thread, this);
        _cb_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        _count_until_server = rclcpp_action::create_server<CountUntil>(
            this,
            "count_until",
            std::bind(&CountUntilServerNode::goal_callback, this, _1,_2),
            std::bind(&CountUntilServerNode::cancel_callback, this, _1),
            std::bind(&CountUntilServerNode::handle_accepted_callback, this, _1),
            rcl_action_server_get_default_options(),
            _cb_group
        );

        RCLCPP_INFO(this->get_logger(), "Action server has been started");
    }

    ~CountUntilServerNode()
    {
        _goal_queue_thread.join();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilServerNode>();

    /* multithreaded executor implementation to handle multiple callbacks */
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}