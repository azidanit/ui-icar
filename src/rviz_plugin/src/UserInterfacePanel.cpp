//
// Created by azidanit on 21/07/20.
//

#include "UserInterfacePanel.h"

UserInterfacePanel::UserInterfacePanel(QWidget *parent):
rviz::Panel(parent), ui_(new Ui::userInterface()){

    ui_->setupUi(this);
    timer_min = new QTimer();
    changeTime();
    timer_min->start(60000);
    initSubscriber();
    initPublihser();
    initUi();
    connectWidget();
//    terminal_dest_fb = 0;
//    terminal_dest_cmd = 0;
    status_cmd = 0;
    status_auto = 0;

}

UserInterfacePanel::~UserInterfacePanel() {
    timer_min->stop();
    delete(timer_min);
    delete(ui_);
}

void UserInterfacePanel::initUi(){
    ui_->steer_left_bar->setMaximum(6500);
    ui_->steer_right_bar->setMaximum(6500);

    ui_->state_label->setText("STANDBY");

    std::string path = ros::package::getPath("rviz_plugin");

//    ui_->car_logo->setPixmap(QPixmap((path + "/media/golf-cart.png").c_str()));
//    ui_->maps_logo->setPixmap(QPixmap((path + "/media/compass.png").c_str()));
//    ui_->setting_logo->setPixmap(QPixmap((path + "/media/settings.png").c_str()));
    ui_->turn_left_lamp->setPixmap(QPixmap((path + "/media/arrow_left.png").c_str()));
    ui_->turn_right_lamp->setPixmap(QPixmap((path + "/media/arrow_right.png").c_str()));
    ui_->its_logo_label->setPixmap(QPixmap((path + "/media/logoits.png").c_str()));
    ui_->robotics_logo_label->setPixmap(QPixmap((path + "/media/itsrobot.png").c_str()));
    ui_->wifi_logo->setPixmap(QPixmap((path + "/media/internet.png").c_str()));
    ui_->steering_logo->setPixmap(QPixmap((path + "/media/steering-wheel.png").c_str()));

    ui_->turn_right_lamp->hide();
    ui_->turn_left_lamp->hide();
}

void UserInterfacePanel::initSubscriber() {
    send_ctrl_subs = nh.subscribe("/data_to_stm", 10, &UserInterfacePanel::sendControlCallback, this);
//    state_terminal_subs = nh.subscribe("/autonomous_car/state_terminal", 10, &UserInterfacePanel::stateTerminalCallback, this);
    odometry_subs = nh.subscribe("/car/odometry", 1, &UserInterfacePanel::odometryCallback, this);
}

void UserInterfacePanel::initPublihser() {
    user_cmd_publisher = nh.advertise<std_msgs::UInt8MultiArray>("/rviz_plugin/user_cmd",1);
}

void UserInterfacePanel::connectWidget(){
    connect(timer_min, &QTimer::timeout, this, &UserInterfacePanel::changeTime);
//    connect(ui_->start_stop_button, SIGNAL(clicked()), this, SLOT(startStopCLickedCallback()));

//    connect(ui_->terminal1_button, &QPushButton::clicked, this, [=](){terminalCLickedCallback(1);});
//    connect(ui_->terminal2_button, &QPushButton::clicked, this, [=](){terminalCLickedCallback(2);});
//    connect(ui_->terminal3_button, &QPushButton::clicked, this, [=](){terminalCLickedCallback(3);});
}

void UserInterfacePanel::startStopCLickedCallback(){
//    playBeep();
//    std_msgs::UInt8MultiArray msg;
//    if (ui_->start_stop_button->text() == "START"){
//        msg.data.push_back(1);
//        std::cout << "STARTING\n";
//        ui_->start_stop_button->setText("STOP");
//        ui_->start_stop_button->setStyleSheet("background-color: red");
//    }else if(ui_->start_stop_button->text() == "STOP") {
//        msg.data.push_back(0);
//        std::cout << "PAUSING\n";
//        ui_->start_stop_button->setText("CONTINUE");
//        ui_->start_stop_button->setStyleSheet("background-color: yellow");
//    }else if(ui_->start_stop_button->text() == "CONTINUE") {
//        msg.data.push_back(2);
//        std::cout << "PAUSING\n";
//        ui_->start_stop_button->setText("STOP");
//        ui_->start_stop_button->setStyleSheet("background-color: red");
//    }else{
//        msg.data.push_back(0);
//        std::cout << "STOPPING\n";
//        ui_->start_stop_button->setText("START");
//        ui_->start_stop_button->setStyleSheet("background-color: green");
//    }
//
//
//    msg.data.push_back(terminal_dest_cmd);
//    user_cmd_publisher.publish(msg);
}

void UserInterfacePanel::terminalCLickedCallback(int value) {
//    std::cout << "TERMINAL CLICKED " << value << std::endl;
//    terminal_dest_cmd = value;
//
//    ui_->terminal1_button->setStyleSheet("");
//    ui_->terminal2_button->setStyleSheet("");
//    ui_->terminal3_button->setStyleSheet("");
//
//    switch (value) {
//        case 1 :
//            ui_->terminal1_button->setStyleSheet("background-color : red");
//            break;
//        case 2:
//            ui_->terminal2_button->setStyleSheet("background-color : red");
//            break;
//        case 3:
//            ui_->terminal3_button->setStyleSheet("background-color : red");
//            break;
//    }
}

void UserInterfacePanel::sendControlCallback(const geometry_msgs::Twist& msg) {
    if (msg.angular.z <= 0){
        ui_->steer_left_bar->setValue((int)-1*msg.angular.z);
        ui_->steer_right_bar->setValue(0);
    }else{
        ui_->steer_right_bar->setValue( (int)(msg.angular.z));
        ui_->steer_left_bar->setValue(0);
    }
}

void UserInterfacePanel::blinkLeftSignal(bool val) {
//    if(turn_signal_timer.setInterval())

}

void UserInterfacePanel::stateTerminalCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg){
//    terminal_dest_fb = msg->data[0];
//    status_auto = msg->data[1];
//
//    switch (status_auto) {
//        case 0:
//            ui_->state_label->setText("MANUAL DRIVING MODE");
//            ui_->start_stop_button->setText("START");
//            ui_->start_stop_button->setStyleSheet("background-color: green");
//            break;
//        case 1:
//            ui_->state_label->setText("AUTONOMOUS DRIVING MODE");
//            break;
//        case 2:
//            ui_->state_label->setText("PARKED");
//            break;
//    }
//
//    terminalCLickedCallback(terminal_dest_fb);
}

void UserInterfacePanel::playBeep() {
    std::string path = ros::package::getPath("rviz_plugin");
    path = path + "/media/tone.wav";
    std::cout << "PATH RVIZ " << path;
    QProcess qprocess;
    qprocess.startDetached("play",QStringList()<<path.c_str());
}

void UserInterfacePanel::playAlert() {
    std::string path = ros::package::getPath("rviz_plugin");
    path = path + "/media/electronic.wav";
    std::cout << "PATH RVIZ " << path;
    QProcess qprocess;
    qprocess.startDetached("play",QStringList()<<path.c_str());
}

void UserInterfacePanel::odometryCallback(const car::odometry msg) {
    ui_->speed_label->setText(QString::number((int)(msg.velocity * 3.6)));
}

void UserInterfacePanel::changeTime() {
    time_t now = time(0);
    tm *ltm = localtime(&now);
    char clk[8];
    sprintf(clk, "%02d:%02d", ltm->tm_hour, ltm->tm_min);
    ui_->clock_label->setText(clk);
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(UserInterfacePanel,rviz::Panel )
