#include "hw1.hpp"
#include <iostream>

publisher::publisher() : Node("publisher")
{
    pub = this->create_publisher<custom_interfaces::msg::Colcon>("topic", 10);
}
void publisher:: publish_data(){
    int num=0;
    custom_interfaces::msg::Colcon data;
    RCLCPP_INFO(this->get_logger(), "10개의 정수를 입력하세요: ");
    for(int i=0; i< 10; i++){
    std::cin>> num;
    if(!std::cin){
        std::cout<<"다시 입력하세요"<<std::endl;
        std::cin.clear(); // clear the error flag
        std::cin.ignore(1000, '\n'); // discard invalid input
        i--;
        continue; // prompt for input again
    }
    std_msgs::msg::Int32 integer = std_msgs::msg::Int32();
    integer.data = num;
    
    data.data.push_back(integer);
    }
    pub->publish(data);

}
void subscriber::callback(const custom_interfaces::msg::Colcon::SharedPtr msg){
   
    RCLCPP_INFO(this->get_logger(), "받은 데이터: ");
    for(auto i : msg->data){
       
    RCLCPP_INFO(this->get_logger(), "%d ", i.data);
    }
}
subscriber::subscriber() : Node("subscriber")
{
    sub = this->create_subscription<custom_interfaces::msg::Colcon>(
        "topic", 10, std::bind(&subscriber::callback, this, std::placeholders::_1));
}