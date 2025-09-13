#include "hw3.hpp"
#include <iostream>
#include <chrono>
publisher::publisher():Node("publish_node"){
    pub = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel",10);
    set_pen = this->create_client<turtlesim::srv::SetPen>("turtle1/set_pen");

}
void publisher::publish_message(){
    char ch=0;
    while(1){
     RCLCPP_INFO(this->get_logger(),"W: 정사각형, A: 정삼각형, S: 원, D: 삭제");
     std::cin>>ch;
     if(!std::cin||(ch!='W'&& ch!= 'w'&&ch!='A'&& ch!='a'&&ch!='S'&& ch!='s'&&ch!='D'&& ch!='d')){
         RCLCPP_INFO(this->get_logger(),"잘못된 입력입니다.");
         std::cin.clear();
         std::cin.ignore(1000,'\n');
         continue;
     }
     if(ch == 'W'|| ch=='w'){
        auto req = std::make_shared<turtlesim::srv::SetPen::Request>();
        req->r = 255;
        req->g = 0;
        req->b = 0;
        req->width = 3;
        set_pen->async_send_request(req);
        geometry_msgs::msg::Twist temp;
        for(int i=0;i<4;i++){
        temp.linear.x=2.0;
        temp.angular.z=0.0;
        pub-> publish(temp);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        temp.linear.x=0.0;
        temp.angular.z=1.57;
        pub-> publish(temp);
        std::this_thread::sleep_for(std::chrono::seconds(3));
     }
    }
     else if(ch== 'A'||ch=='a'){
        auto req = std::make_shared<turtlesim::srv::SetPen::Request>();
        req->r = 255;
        req->g = 0;
        req->b = 0;
        req->width = 3;
        set_pen->async_send_request(req);
        geometry_msgs::msg::Twist temp;
        for(int i=0;i<3;i++){
        temp.linear.x=2.0;
        temp.angular.z=0.0;
        pub-> publish(temp);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        temp.linear.x=0.0;
        temp.angular.z=2.09;
        pub-> publish(temp);
        std::this_thread::sleep_for(std::chrono::seconds(3));
     }
    }
     else if(ch== 'S'||ch=='s'){
        auto req = std::make_shared<turtlesim::srv::SetPen::Request>();
        req->r = 255;
        req->g = 0;
        req->b = 0;
        req->width = 3;
        set_pen->async_send_request(req);
        geometry_msgs::msg::Twist temp;
        for(int i=0; i<4; i++){
        temp.linear.x=2.0;
        temp.angular.z=1.8;
        pub-> publish(temp);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        
      }
     else if(ch== 'D'||ch=='d'){
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr empty_ptr;
        empty_ptr = this->create_client<std_srvs::srv::Empty>("/clear");
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        empty_ptr->async_send_request(request);
     }
    
    }
}
