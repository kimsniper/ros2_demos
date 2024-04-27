#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "example_interfaces/msg/int64.hpp"

using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class NumberPublisherNode : public rclcpp_lifecycle::LifecycleNode
{
private:

    int _number;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr _number_publisher;
    rclcpp::TimerBase::SharedPtr _number_timer;
    double publish_frequency = 1.0;

    void publishNumber()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = _number;
        _number_publisher->publish(msg);
        _number++;
    }

public:
    NumberPublisherNode() : LifecycleNode("number_publisher")
    {
        _number = 1;
        publish_frequency = 1.0;
        RCLCPP_INFO(this->get_logger(), "In constructor");
    }

    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "In on_configure");
        _number_publisher = 
            this->create_publisher<example_interfaces::msg::Int64>("number", 10);
        _number_timer = 
            this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / publish_frequency)),
                                    std::bind(&NumberPublisherNode::publishNumber, this));
        _number_timer->cancel();

        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(this->get_logger(), "In on_deactivate");
        _number_timer->cancel();
        rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(this->get_logger(), "In on_activate");
        _number_timer->reset();
        rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "In on_cleanup");
        _number_publisher.reset();
        _number_timer.reset();

        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "In on_shutdown");
        _number_publisher.reset();
        _number_timer.reset();

        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_error(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "In on_error");
        _number_publisher.reset();
        _number_timer.reset();

        return LifecycleCallbackReturn::FAILURE;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisherNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
