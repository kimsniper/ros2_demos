#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

class RobotNewsSubscriberNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr _subscriber;

    void subscribeCallback(const example_interfaces::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
    }

public:
    RobotNewsSubscriberNode() : Node("robot_news_subscriber")
    {
        _subscriber = this->create_subscription<example_interfaces::msg::String>("robot_news",
                      10,
                      std::bind(&RobotNewsSubscriberNode::subscribeCallback,
                      this,
                      std::placeholders::_1)); /* This parameter is required because the callback function has one parameter
                                               *  For additional parameters, use placeholders::_2 . . .. 
                                               */

        RCLCPP_INFO(this->get_logger(), "Robot News subscriber has been started");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNewsSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}