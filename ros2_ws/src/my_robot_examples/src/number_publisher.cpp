#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class NumberPublisherNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;
    int _number;
    double _publish_frequency;

    void timerCallback()
    {
        /* Publish number */
        auto msg = example_interfaces::msg::Int64();
        msg.data = _number;
        _publisher->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Published value - %d", (int)msg.data);
    }

public:
    NumberPublisherNode() : Node("number_publisher")
    {
        this->declare_parameter("number_to_publish", 2);
        this->declare_parameter("publish_frequency", 1.0);

        _number = this->get_parameter("number_to_publish").as_int();
        _publish_frequency = this->get_parameter("publish_frequency").as_double();

        _publisher = this->create_publisher<example_interfaces::msg::Int64>("number", 10);

        _timer = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0/_publish_frequency)),
                                         std::bind(&NumberPublisherNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Number Publisher has been started");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}