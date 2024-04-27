#include "rclcpp/rclcpp.hpp"

class NodeName: public rclcpp::Node
{
private:

public:
    NodeName(): Node("node_name")
    {

    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NodeName>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}