//
// Created by icar on 25/07/20.
//

#ifndef SRC_MAINWINDOW_H
#define SRC_MAINWINDOW_H

#include <QMainWindow>
#include <QMessageBox>
#include <ros/package.h>
#include <ros/ros.h>
#include <QtWidgets/QLabel>
#include <QtWebKit/QtWebKit>
// #include <QtWebKit/QWebView>
// #include <QtWebView>

#include <iostream>
#include <mutex>
#include <thread>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/TwistStamped.h>
#include "rviz_plugin/userCmd.h"

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    // void init();


private:
    const std::string HALTE_NAME[7] = {" ", "Asrama", "Manarul Ilmi", "Bundaran", "Teknik Lingkungan", "Rektorat", "Kantin Pusat"};
    ros::NodeHandle nh;
    ros::Publisher user_cmd_publisher;
    ros::Subscriber send_ctrl_subs, state_terminal_subs, mavros_speed, odom_sub, eta_subs;

    std::thread subs_thread, play_sound_thread, play_blinker_sound_thread;
    Ui::MainWindow *ui;
    std::string path_pkg;
    QTimer* mTimer;
    void initUi();
    void initConnection();
    void initPublisher();
    void initSubscriber();
    void subscribingThread();

    void changeTerminalButton(int terminal, bool status);
    void updateTerminalButton(int terminal);
    void changeStartStopButton();
    void stateTerminalCallback(const std_msgs::Int32MultiArray msg);
    void sendControlCallback(const geometry_msgs::Twist &msg);
    void localVelCallback(const geometry_msgs::TwistStamped &msg);
    void etaCallback(const std_msgs::Float32 &msg);
    void terminalArrived(int terminal);
    void gotoTerminal(int terminal);
    void playTerminalVoice(int terminal);
    void playSound(std::string sound_, double duration, int loop_times);
    void playSoundThread(std::string sound_, double duration, int loop_times);
    void playBlinkerSoundThread();
    void playBlinkerSound();

    // void odomVelCallback()
    std::mutex general_mtx, terminal_mtx;
    int terminal_to,terminal_dest_fb;
    int is_auto;

    bool is_blinker_playing;
    std::mutex blinker_mtx;



public Q_SLOTS:
    void playAttention();

Q_SIGNALS:
    void StartStopClicked();
    void terminalClicked(int terminal);
};


#endif //SRC_MAINWINDOW_H
