#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/compute_rectangle_area.hpp"

class ComputeRectangleAreaClientNode: public rclcpp::Node
{

private:
    std::vector<std::thread> _threads; 
public:
    ComputeRectangleAreaClientNode(): Node("compute_rectangle_area_client")
    {
        _threads.push_back(std::thread(std::bind(&ComputeRectangleAreaClientNode::ComputeRectangleAreaService, this, 3, 4)));
        _threads.push_back(std::thread(std::bind(&ComputeRectangleAreaClientNode::ComputeRectangleAreaService, this, 5, 4)));
    }

    void ComputeRectangleAreaService(float _length, float _width)
    {
        auto client = this->create_client<my_robot_interfaces::srv::ComputeRectangleArea>("compute_rectangle_area");

        while(!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for server to be up. . .");
        }

        auto request = std::make_shared<my_robot_interfaces::srv::ComputeRectangleArea::Request>();
        request->length = _length;
        request->width = _width;

        auto future = client->async_send_request(request);

        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "%.2f * %.2f = %.2f", _length, _width, response->area);
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
    auto node = std::make_shared<ComputeRectangleAreaClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}