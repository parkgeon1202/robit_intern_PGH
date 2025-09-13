/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date August 2024
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/hw2_pkg/qnode.hpp"
#include "ui_mainwindow.h"
#include <chrono>
QNode::QNode()
{
  int argc = 0;
  char** argv = NULL;
  rclcpp::init(argc, argv);
  pub_ptr = std::make_shared<publisher>();
  this->start();
}

QNode::~QNode()
{
  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }
}

void QNode::run()
{
  rclcpp::WallRate loop_rate(20);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(pub_ptr);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  Q_EMIT rosShutDown();
}
publisher::publisher():Node("publish_node"){
    pub = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel",10);
    set_pen = this->create_client<turtlesim::srv::SetPen>("turtle1/set_pen");
    
}
void publisher::publish_message(int flag){
  

     if(flag==1){
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
        ui->lineEdit-> setText("linear: "+QString::number(temp.linear.x) + "," + "angular: "+QString::number(temp.angular.z));
        QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        temp.linear.x=0.0;
        temp.angular.z=1.57;
        pub-> publish(temp);
        ui->lineEdit-> setText("linear: "+QString::number(temp.linear.x) + "," + "angular: "+QString::number(temp.angular.z));
        QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
        std::this_thread::sleep_for(std::chrono::seconds(3));
     }
    }
     else if(flag==2){
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
        ui->lineEdit-> setText("linear: "+QString::number(temp.linear.x) + "," + "angular: "+QString::number(temp.angular.z));
        QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
        std::this_thread::sleep_for(std::chrono::seconds(3));
        temp.linear.x=0.0;
        temp.angular.z=2.09;
        pub-> publish(temp);
        ui->lineEdit-> setText("linear: "+QString::number(temp.linear.x) + "," + "angular: "+QString::number(temp.angular.z));
        QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
        std::this_thread::sleep_for(std::chrono::seconds(3));
     }
    }
     else if(flag==3){
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
        ui->lineEdit-> setText("linear: "+QString::number(temp.linear.x) + "," + "angular: "+QString::number(temp.angular.z));
        QCoreApplication::processEvents(QEventLoop::AllEvents, 1);
        std::this_thread::sleep_for(std::chrono::seconds(1)); 
      }
   
      }
     else if(flag==4){
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr empty_ptr;
        empty_ptr = this->create_client<std_srvs::srv::Empty>("/clear");
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        empty_ptr->async_send_request(request);
     }
    
    
}