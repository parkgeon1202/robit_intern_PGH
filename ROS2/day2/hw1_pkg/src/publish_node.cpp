#include "hw1.hpp"
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<publisher>();
    node->publish_data();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
