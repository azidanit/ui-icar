//
// Created by icar on 25/07/20.
//

#ifndef SRC_APP_H
#define SRC_APP_H

#include "mainwindow.h"
#include <QApplication>
#include <QObject>
#include <typeinfo>
#include <ros/ros.h>
#include "ui_main_ui.h"

class App : public QApplication{
Q_OBJECT
public:
    MainWindow *w;
    App(int &argc, char** argv) ;
    ~App();

private:
    ros::NodeHandle nh;
//    MissionWrapper *missionWrapper;
//    std::thread mission_thread;
    void initConnection();

};


#endif //SRC_APP_H
