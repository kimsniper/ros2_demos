#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"
#include "my_robot_interfaces/msg/led_states.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class BatteryNode : public rclcpp::Node
{
private:
    rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr _server;
    rclcpp::Publisher<my_robot_interfaces::msg::LedStates>::SharedPtr _publisher;

    bool led_states[3] {false};

    void ServiceCallback(const my_robot_interfaces::srv::SetLed::Request::SharedPtr request,
                         const my_robot_interfaces::srv::SetLed::Response::SharedPtr response)
    {
        led_states[request->led_num - 1] = request->led_state;
        response->is_success = true;

        RCLCPP_INFO(this->get_logger(), "LED %d is set to %s", request->led_num, request->led_state ? "ON" : "OFF");

        auto msg = my_robot_interfaces::msg::LedStates();

        for(int i=0;i<3;i++)
        {
            msg.led_states[i] = led_states[i];
        }

        _publisher->publish(msg);
    }

public:
    BatteryNode() : Node("led_panel")
    {
        _server = this->create_service<my_robot_interfaces::srv::SetLed>("set_led",
                                                                         std::bind(&BatteryNode::ServiceCallback,
                                                                                   this,
                                                                                   _1,
                                                                                   _2));

        _publisher = this->create_publisher<my_robot_interfaces::msg::LedStates>("led_panel_state", 10);

        RCLCPP_INFO(this->get_logger(), "Service server has been started");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}