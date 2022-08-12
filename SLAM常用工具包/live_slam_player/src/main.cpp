#include "mainwindow.h"
#include <QApplication>
//#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glu.h>
//#include <GL/glut.h>
#include <ros/ros.h>
#include <QProcess>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "live_slam_player");
  ros::NodeHandle nh;

  QApplication a(argc, argv);
  MainWindow w;
  w.RosInit(nh);
  w.show();

   return a.exec();
}
