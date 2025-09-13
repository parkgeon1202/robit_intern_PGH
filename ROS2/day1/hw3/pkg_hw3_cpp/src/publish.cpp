#include "hw3.hpp"
int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<publisher>();
    node->publish_message();
    rclcpp::shutdown();
    return 0;
}