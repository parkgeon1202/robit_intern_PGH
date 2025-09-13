#include <QApplication>
#include <QTimer>
#include "../include/hw2_pkg/main_window.hpp"

int main(int argc, char* argv[])
{
  
  QApplication a(argc, argv);
  MainWindow w;
  // auto sub = std::make_shared<subscribe_node>(w.get_ui());
  // QTimer timer;
  // QObject::connect(&timer, &QTimer::timeout, [&](){
  //   rclcpp::spin_some(sub);         
  // });
  // timer.start(10);
  // timer.setInterval(10);
  w.show();
  return a.exec();
}
