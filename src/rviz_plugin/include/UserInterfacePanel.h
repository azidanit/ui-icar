//
// Created by azidanit on 21/07/20.
//

#ifndef SRC_USERINTERFACEPANEL_H
#define SRC_USERINTERFACEPANEL_H

#include <ros/ros.h>
#include <ros/package.h>
#include <rviz/panel.h>
// #include <QtMultimedia/QSound>
// #include <QtMultimedia/QMediaPlayer>
#include <QProcess>
#include <QTimer>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <std_msgs/UInt8MultiArray.h>
#include "rviz_plugin/userCmd.h"
#include <car/odometry.h>
#include <ctime>

#include "ui_user_interface.h"

namespace Ui{
    class userInterface;
}

class UserInterfacePanel: public rviz::Panel {
    Q_OBJECT

public:
    UserInterfacePanel(QWidget* parent = 0);
    ~UserInterfacePanel();

public Q_SLOTS:
    void terminalCLickedCallback(int value);
    void startStopCLickedCallback();
    void changeTime();

protected:
    Ui::userInterface *ui_;

private:
    ros::NodeHandle nh;
    ros::Subscriber send_ctrl_subs, state_terminal_subs, odometry_subs;
    ros::Publisher user_cmd_publisher;
    int terminal_dest_fb, terminal_dest_cmd, status_auto;
    int status_cmd;
    QTimer * timer_min;

    void initSubscriber();
    void initPublihser();

    void initUi();
    void connectWidget();

    //subscriber Callback
    void sendControlCallback(const geometry_msgs::Twist& msg);
//    void isArrivedOnTerminal();
    void stateTerminalCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg);

    void odometryCallback(const car::odometry msg);

//  Audio Process
    void playBeep();
    void playWarn();
    void playAlert();

    void blinkLeftSignal(bool);
    void blinkRightSignal(bool);
    QTimer turn_signal_timer;

};


#endif //SRC_USERINTERFACEPANEL_H
