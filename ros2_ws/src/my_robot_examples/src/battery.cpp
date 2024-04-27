#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

class BatteryNode: public rclcpp::Node
{

private:
    std::vector<std::thread> _threads; 
public:
    BatteryNode(): Node("battery")
    {
        _threads.push_back(std::thread(std::bind(&BatteryNode::SetLedService, this, 3, false)));
        _threads.push_back(std::thread(std::bind(&BatteryNode::SetLedService, this, 3, true)));
    }

    void SetLedService(int _led_num, bool _state)
    {
        auto client = this->create_client<my_robot_interfaces::srv::SetLed>("set_led");

        while(!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for server to be up. . .");
        }

        auto request = std::make_shared<my_robot_interfaces::srv::SetLed::Request>();
        request->led_num = _led_num;
        request->led_state = _state;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "LED %d is set to %s", _led_num, _state ? "ON" : "OFF");
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed!");
        }
        
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