#include "pkg_hw2.hpp"
#include <chrono>
using namespace std::chrono_literals;

      MyCppNode:: MyCppNode():Node("publish_node"){
        RCLCPP_INFO(this->get_logger(), "Hello, ROS 2 C++ Node!");
        mycpp_publisher_= this->create_publisher<std_msgs::msg::String>("topicname", 10);
        mycpp_publisher_int= this->create_publisher<std_msgs::msg::Int32>("int_topic", 10);
        mycpp_publisher_float= this->create_publisher<std_msgs::msg::Float32>("float_topic", 10);
        timer_= this->create_wall_timer(1s, std::bind(&MyCppNode::timer_callback, this));
      }

  
      void MyCppNode:: timer_callback(){
        auto msg = std_msgs::msg::String();
        auto num = std_msgs::msg::Int32();
        auto fl= std_msgs::msg::Float32();
        msg.data= "Hello World: "+ std::to_string(count_++);
        num.data = 3456+count_;
        fl.data = 3.4567+count_;
        RCLCPP_INFO(this->get_logger(), "Published message: '%s', int: '%d', float: '%f'", msg.data.c_str(), num.data, fl.data);
        
        mycpp_publisher_ -> publish(msg);
        mycpp_publisher_int->publish(num);
        mycpp_publisher_float->publish(fl);
      }
      

      
   







     void subscribe_node:: topic_callback_str(const std_msgs::msg::String::SharedPtr msg){
        
        RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
     }
     void subscribe_node:: topic_callback_int( const std_msgs::msg::Int32::SharedPtr num){
        
        RCLCPP_INFO(this->get_logger(), "Received message: '%d'",  num->data);
     }
     void subscribe_node:: topic_callback_float( const std_msgs::msg::Float32::SharedPtr fl){
        
        RCLCPP_INFO(this->get_logger(), "Received message: '%f'", fl->data);
     }
     
     subscribe_node:: subscribe_node():Node("subscribe_node"){
         mycpp_subscriber_= this->create_subscription<std_msgs::msg::String>("topicname", 10, std::bind(&subscribe_node::topic_callback_str, this, std::placeholders::_1));
         mycpp_subscriber_int =  this->create_subscription<std_msgs::msg::Int32>("int_topic", 10, std::bind(&subscribe_node::topic_callback_int, this, std::placeholders::_1));
         mycpp_subscriber_float =  this->create_subscription<std_msgs::msg::Float32>("float_topic", 10, std::bind(&subscribe_node::topic_callback_float, this, std::placeholders::_1));
      }

