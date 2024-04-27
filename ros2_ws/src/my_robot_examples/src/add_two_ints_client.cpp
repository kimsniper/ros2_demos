#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class AddTwoIntsClientNode: public rclcpp::Node
{

private:
    std::vector<std::thread> _threads; 
public:
    AddTwoIntsClientNode(): Node("add_two_ints_client")
    {
        _threads.push_back(std::thread(std::bind(&AddTwoIntsClientNode::AddTwoIntService, this, 1, 4)));
        _threads.push_back(std::thread(std::bind(&AddTwoIntsClientNode::AddTwoIntService, this, 5, 4)));
    }

    void AddTwoIntService(int _a, int _b)
    {
        auto client = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

        while(!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for server to be up. . .");
        }

        auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = _a;
        request->b = _b;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "%d + %d = %d", _a, _b, (int)response->sum);
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
    auto node = std::make_shared<AddTwoIntsClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}