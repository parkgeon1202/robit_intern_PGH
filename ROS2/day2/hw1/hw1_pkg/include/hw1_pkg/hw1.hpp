#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include "custom_interfaces/msg/colcon.hpp"
#include <vector>
class publisher : public rclcpp::Node
{
    
    rclcpp::Publisher<custom_interfaces::msg::Colcon>::SharedPtr pub;
   public:
    publisher();
    void publish_data();
    

};
class subscriber : public rclcpp::Node
{
    rclcpp::Subscription<custom_interfaces::msg::Colcon>::SharedPtr sub;
    
   public:
    subscriber();
    void callback(const custom_interfaces::msg::Colcon::SharedPtr msg);
};
