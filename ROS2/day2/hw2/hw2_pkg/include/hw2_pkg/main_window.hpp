/**
 * @file /include/hw2_pkg/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date August 2024
 **/

#ifndef hw2_pkg_MAIN_WINDOW_H
#define hw2_pkg_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include <QIcon>
#include "qnode.hpp"
#include <geometry_msgs/msg/twist.hpp>

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */

namespace Ui {
class MainWindow;
}
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();
  Ui::MainWindow* get_ui(void){return ui;}
  friend class publisher;
private slots:
  void on_drawSquare_clicked();
  void on_drawCircle_clicked();
  void on_drawTriangle_clicked();
  void on_Delete_clicked();
private:
  Ui::MainWindow* ui;
  void closeEvent(QCloseEvent* event);
  QNode* qnode;
  
};
// class subscribe_node: public rclcpp::Node{
//   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub;     
//   Ui:: MainWindow* ui;  
//   void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
   
//   public:
//   subscribe_node(Ui::MainWindow* ui);
// };
#endif  // hw2_pkg_MAIN_WINDOW_H
