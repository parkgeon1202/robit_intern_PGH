/**
 * @file /include/hw2_pkg/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef hw2_pkg_QNODE_HPP_
#define hw2_pkg_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/srv/set_pen.hpp>
#include <std_srvs/srv/empty.hpp>
#endif
#include <QThread>

namespace Ui { class MainWindow; }
/*****************************************************************************
** Class
*****************************************************************************/
class publisher: public rclcpp::Node{
   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
   rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr set_pen;
   
   public:
   Ui:: MainWindow* ui;
   publisher();
   void publish_message(int flag);
};
class QNode : public QThread
{
  Q_OBJECT
public:
  QNode();
  ~QNode();
  std::shared_ptr<publisher> pub_ptr;
protected:
  void run();

private:
 
  
Q_SIGNALS:
  void rosShutDown();
};

#endif /* hw2_pkg_QNODE_HPP_ */
