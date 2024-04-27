#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

class RobotNewsPublisherNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr _publisher;
    std::string _robot_name;
    rclcpp::TimerBase::SharedPtr _timer;

    void timerCallback()
    {
        /* Publish news */
        auto msg = example_interfaces::msg::String();
        msg.data = std::string("This is ") + _robot_name + std::string(" from the Robot News Station");
        _publisher->publish(msg);
    }

public:
    RobotNewsPublisherNode() : Node("robot_news_publisher"), _robot_name("Robot")
    {
        _publisher = this->create_publisher<example_interfaces::msg::String>("robot_news", 10);

        _timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                         std::bind(&RobotNewsPublisherNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Robot News Publisher has been started");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNewsPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}