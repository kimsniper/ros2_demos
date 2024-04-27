#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/hardware_status.hpp"

class HardwareStatusPublisherNode : public rclcpp::Node
{
private:
    rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;

    void timerCallback()
    {
        auto msg = my_robot_interfaces::msg::HardwareStatus();
        msg.temperature = 38;
        msg.motors_ready = false;
        msg.debug_msg = "Motor temperature is normal";
        _publisher->publish(msg);
    }

public:
    HardwareStatusPublisherNode() : Node("hardware_status_publisher")
    {
        _publisher = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>("hardware_status", 10);

        _timer = this->create_wall_timer(std::chrono::milliseconds(1000),
                                         std::bind(&HardwareStatusPublisherNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "hardware Status Publisher has been started");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareStatusPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}