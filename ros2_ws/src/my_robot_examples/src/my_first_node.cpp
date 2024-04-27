#include "rclcpp/rclcpp.hpp"

class MyNode: public rclcpp::Node
{
private:
    void timerCallback();
    rclcpp::TimerBase::SharedPtr _timer;
    int _counter;

public:
    MyNode(): Node("cpp_test"), _counter(0)
    {
        RCLCPP_INFO(this->get_logger(), "Hello Cpp Node");

        _timer = this->create_wall_timer(std::chrono::seconds(1),
                                       std::bind(&MyNode::timerCallback, this));
    }
};

void MyNode::timerCallback()
{
    this->_counter++;
    RCLCPP_INFO(this->get_logger(), "Count: %d", this->_counter);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}