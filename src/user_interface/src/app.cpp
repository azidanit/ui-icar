//
// Created by icar on 25/07/20.
//

#include <QApplication>
#include <QObject>
#include "mainwindow.h"
#include <typeinfo>
#include <ros/ros.h>
#include "app.h"

App::App(int &argc, char** argv)
        : QApplication(argc, argv)
{
    w = new MainWindow();
    ROS_INFO("User Interface App started");
    w->setWindowFlags(Qt::Window | Qt::FramelessWindowHint);
    w->setWindowState(Qt::WindowFullScreen);
    w->show();


}

App::~App(){
    delete w;
//    qDebug() << "App deleted";
}