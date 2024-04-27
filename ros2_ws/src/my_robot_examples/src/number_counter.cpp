#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class NumberCounterNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr _subscriber;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr _publisher;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr _server;
    int _counter;

    void subscribeCallback(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received Value - %d", (int)msg->data);

        /* Update _counter variable */
        _counter += (int)msg->data;

        /* Publish number */
        auto msg_pub = example_interfaces::msg::Int64();
        msg_pub.data = _counter;
        _publisher->publish(msg_pub);

        RCLCPP_INFO(this->get_logger(), "Published value - %d", (int)msg_pub.data);
    }

    void ServiceCallback(const example_interfaces::srv::SetBool::Request::SharedPtr request,
                         const example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
        if(request->data)
        {
            RCLCPP_INFO(this->get_logger(), "Resetting counter . . .");
            _counter = 0;
            response->success = true;
            response->message = "Counter was reset";
        }
        else
        {
            response->success = false;
            response->message = "Counter not reset";
        }
    }

public:
    NumberCounterNode() : Node("number_counter")
    {
        _publisher = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
        
        _subscriber = this->create_subscription<example_interfaces::msg::Int64>("number",
                      10,
                      std::bind(&NumberCounterNode::subscribeCallback,
                                this,
                                std::placeholders::_1)); /* This parameter is required because the callback function has one parameter
                                                          *  For additional parameters, use placeholders::_2 . . .. 
                                                          */
        
        _server = this->create_service<example_interfaces::srv::SetBool>("reset_counter",
                                                                            std::bind(&NumberCounterNode::ServiceCallback,
                                                                                      this,
                                                                                      _1,
                                                                                      _2));

        RCLCPP_INFO(this->get_logger(), "Number Counter has been started");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}