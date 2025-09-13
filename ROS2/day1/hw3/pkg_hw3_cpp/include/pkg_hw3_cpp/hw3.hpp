#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/srv/set_pen.hpp>
#include <std_srvs/srv/empty.hpp>
class publisher: public rclcpp::Node{
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr set_pen;
    public:
    publisher();
    void publish_message();
    
};
