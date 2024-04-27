#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/compute_rectangle_area.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class ComputeRectangleAreaServerNode : public rclcpp::Node
{
private:
    rclcpp::Service<my_robot_interfaces::srv::ComputeRectangleArea>::SharedPtr _server;

    void ServiceCallback(const my_robot_interfaces::srv::ComputeRectangleArea::Request::SharedPtr request,
                         const my_robot_interfaces::srv::ComputeRectangleArea::Response::SharedPtr response)
    {
        response->area = request->length * request->width;

        RCLCPP_INFO(this->get_logger(), "Operation Area = LxW - %.2f * %.2f = %.2f", request->length, request->width, response->area);
    }

public:
    ComputeRectangleAreaServerNode() : Node("compute_rectangle_area_server")
    {
        _server = this->create_service<my_robot_interfaces::srv::ComputeRectangleArea>("compute_rectangle_area",
                                                                            std::bind(&ComputeRectangleAreaServerNode::ServiceCallback,
                                                                                      this,
                                                                                      _1,
                                                                                      _2));
        
        RCLCPP_INFO(this->get_logger(), "Service server has been started");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ComputeRectangleAreaServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}