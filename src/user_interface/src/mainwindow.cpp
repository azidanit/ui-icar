//
// Created by icar on 25/07/20.
//


#include <std_msgs/Int8MultiArray.h>
#include <QProcess>
#include "mainwindow.h"
//#include "ui_main_ui.h"
#include "ui_ui_maps.h"

MainWindow::MainWindow(QWidget *parent) :
        QMainWindow(parent),
        ui(new Ui::MainWindow)
{
    is_blinker_playing = false;
    ui->setupUi(this);
    initUi();
    mTimer = new QTimer(this);
    // mTimer->setSingleShot(true);
    this->initConnection();
    initPublisher();
    initSubscriber();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::initPublisher() {
    user_cmd_publisher = nh.advertise<std_msgs::UInt8MultiArray>("/user_interface/user_cmd",1);
}

void MainWindow::initSubscriber() {
    // odom_sub = subscribe("/car/odomtery", 10, &MainWindow::localVelCallback, this);
//    mavros_speed = nh.subscribe("/mavros/local_position/velocity", 10, &MainWindow::localVelCallback, this);
    send_ctrl_subs = nh.subscribe("/data_to_stm", 10, &MainWindow::sendControlCallback, this);
    state_terminal_subs = nh.subscribe("/autonomous_car/state_terminal", 10, &MainWindow::stateTerminalCallback, this);

    subs_thread = std::thread(&MainWindow::subscribingThread, this);
}

void MainWindow::subscribingThread() {
    ros::Rate rr(50);
    while (ros::ok()){
        ros::spinOnce();
        rr.sleep();
//        std::cout<<"SUSBSS\n";
    }
}

void MainWindow::initConnection() {
    connect(ui->terminal_1_button, &QPushButton::clicked, this, [=](){changeTerminalButton(1,1);});
    connect(ui->terminal_2_button, &QPushButton::clicked, this, [=](){changeTerminalButton(2,1);});
    connect(ui->terminal_3_button, &QPushButton::clicked, this, [=](){changeTerminalButton(3,1);});
    connect(ui->start_stop_button, &QPushButton::clicked, this, [=](){changeStartStopButton();});

    connect(mTimer, &QTimer::timeout, this, &MainWindow::playAttention);

    // connect(this, &MainWindow::terminalArrived, this, [=](int terminal){std::cout << "terminal arrived" << std::endl; changeTerminalButton(terminal, 2);});
}

void MainWindow::initUi() {
    path_pkg = ros::package::getPath("user_interface");
    ui->map_bar->setPixmap(QPixmap((path_pkg + "/ui/Assets/bundaran1_2.png").c_str()));
//    ui->car_bar->setPixmap(QPixmap((path_pkg + "/ui/Assets/car_bar.png").c_str()));
//    ui->media_bar->setPixmap(QPixmap((path_pkg + "/ui/Assets/media_bar.png").c_str()));
//    ui->weather_bar->setPixmap(QPixmap((path_pkg + "/ui/Assets/weather_bar.png").c_str()));
//    ui->start_stop_button->setPixmap(QPixmap((path_pkg + "/ui/Assets/start.png").c_str()));
    ui->overview_bar->setPixmap(QPixmap((path_pkg + "/ui/Assets/overview_bar.png").c_str()));
    ui->auto_bar->setPixmap(QPixmap((path_pkg + "/ui/Assets/auto_bar.png").c_str()));
//    ui->terminal_button->setPixmap(QPixmap((path_pkg + "/ui/Assets/notActive/TerminalButton.png").c_str()));

    this->changeTerminalButton(2, 1);
    ui->terminal_3_button->setDisabled(true);
}

void MainWindow::changeTerminalButton(int terminal, bool status){
    playSound("beep_0.2s.wav",0.2,1);

    terminal_mtx.lock();
    this->terminal_to = terminal;
    ui->terminal_1_button->setStyleSheet("");
    ui->terminal_2_button->setStyleSheet("");
    ui->terminal_3_button->setStyleSheet("");

    // ui->terminal_1_button->setText("Terminal 1");
    // ui->terminal_2_button->setText("Terminal 2");
    // ui->terminal_3_button->setText("Terminal 3");

    if (status==1){
        switch (terminal) {
            case 1:
                ui->terminal_1_button->setStyleSheet("background-color: rgb(0,131,216);color:white");
                break;
            case 2:
                ui->terminal_2_button->setStyleSheet("background-color: rgb(0,131,216);color:white");
                break;
            case 3:
                ui->terminal_3_button->setStyleSheet("background-color: rgb(0,131,216);color:white");
                break;
        }
    }
    else if (status==2){
        std::cout<< "STATUS 2\n";
        switch (terminal) {
            case 1:
                ui->terminal_1_button->setText("Terminal 1 o");
                ui->terminal_3_button->setText("Terminal 3");
                break;
            case 2:
                ui->terminal_2_button->setText("Terminal 2 o");
                ui->terminal_1_button->setText("Terminal 1");
                break;
            case 3:
                ui->terminal_3_button->setText("Terminal 3 o");
                ui->terminal_2_button->setText("Terminal 2");
                break;
        }
    }

    terminal_mtx.unlock();
}

void MainWindow::changeStartStopButton() {
    playSound("notif_1.5s.wav",1.5,1);
    std_msgs::UInt8MultiArray msg;
    if (ui->start_stop_button->text() == "START"){
        // playSound("berangkat.wav",1,1);
        msg.data.push_back(1);
        std::cout << "STARTING\n";
        ui->start_stop_button->setText("STOP");
        ui->start_stop_button->setStyleSheet("background-color: red");
        // printf("I+DIDAASD");
        //override suara
        // gotoTerminal((2));
        // std_msgs::Int32MultiArray msg_int32;
        // msg_int32.data.push_back(1);
        // msg_int32.data.push_back(2);
        // stateTerminalCallback(msg_int32);
    // }else if(ui->start_stop_button->text() == "STOP") {
    //     msg.data.push_back(0);
    //     std::cout << "STOPPING\n";
    //     ui->start_stop_button->setText("CONTINUE");
    //     ui->start_stop_button->setStyleSheet("background-color: yellow");
    // }else if(ui->start_stop_button->text() == "CONTINUE") {
    //     msg.data.push_back(2);
    //     std::cout << "PAUSING\n";
    //     ui->start_stop_button->setText("STOP");
    //     ui->start_stop_button->setStyleSheet("background-color: red");
    }else{
        msg.data.push_back(0);
        std::cout << "STOPPING\n";
        ui->start_stop_button->setText("START");
        ui->start_stop_button->setStyleSheet("background-color: green");
    }

    msg.data.push_back(terminal_to);
    user_cmd_publisher.publish(msg);

}

void MainWindow::stateTerminalCallback(const std_msgs::Int32MultiArray msg){
    if(msg.data.size() < 2){
        return;
    }
    terminal_dest_fb = msg.data[0];
    is_auto = msg.data[1];
    std::cout << "GET STATE TERMINAL "<<is_auto <<std::endl;
    switch (is_auto) {
        case 0:
            ui->driving_mode->setText("Manual");
            ui->driving_label->setText("Mode");
            ui->start_stop_button->setText("START");
            ui->start_stop_button->setStyleSheet("background-color: green");
            break;
        case 1: // Sampai ke terminal tujuan penumpang
            ui->driving_mode->setText("Autonomous");
            ui->driving_label->setText("Stop");

            ui->start_stop_button->setText("START");
            ui->start_stop_button->setStyleSheet("background-color: green");
            updateTerminalButton(terminal_dest_fb);
            terminalArrived((terminal_dest_fb));
            changeTerminalButton(terminal_dest_fb+1,1);

//            changeTerminalButton(0,0);
            break;

        case 2: // menuju ke terminal tujuan
            ui->driving_mode->setText("Autonomous");
            ui->driving_label->setText("Driving");
            gotoTerminal((terminal_dest_fb));
//            changeTerminalButton(0,0);
            break;

        case 3:
            ui->driving_mode->setText("Parked");
            ui->driving_label->setText("On Terminal");
//            changeTerminalButton(0,0);
            changeTerminalButton(terminal_dest_fb+1,1);
            break;

        case 4:
            playTerminalVoice(terminal_dest_fb);
            break;
    }
}

void MainWindow::playTerminalVoice(int terminal) {
    switch (terminal) {
        case 1:
            playSound("rektorat_sampai.wav",1,1);
            break;
        case 2:
            playSound("rektorat_sampai.wav",1,1);
            break;
        case 3:
            // playSound("bundaran_its.wav",1,1);
            break;
    } 

}

void MainWindow::updateTerminalButton(int terminal) {
    ui->terminal_1_button->setEnabled(true);
    ui->terminal_2_button->setEnabled(true);
    ui->terminal_3_button->setEnabled(true);
    if(terminal==1){
        ui->terminal_1_button->setDisabled(true);
    }
    else if(terminal==2){
        ui->terminal_2_button->setDisabled(true);
    }
    else if(terminal==3){
        ui->terminal_3_button->setDisabled(true);
    }
    ui->terminal_3_button->setDisabled(true);
    ui->terminal_1_button->setDisabled(true);
}

void MainWindow::terminalArrived(int terminal){

    ui->last_ckpt->setText(HALTE_NAME[terminal].c_str());
    if (terminal == 1){
        ui->map_bar->setPixmap(QPixmap((path_pkg + "/ui/Assets/rektorat1_2.png").c_str()));
    }else if (terminal == 2){
        ui->map_bar->setPixmap(QPixmap((path_pkg + "/ui/Assets/rektorat2_2.png").c_str()));
    }else if(terminal == 3){
        ui->map_bar->setPixmap(QPixmap((path_pkg + "/ui/Assets/parkiran_manarul_2.png").c_str()));
    }
    playSound("notif_1.5s.wav",1.5,2);
}


void MainWindow::gotoTerminal(int terminal){
    ui->next_stop->setText(HALTE_NAME[terminal].c_str());
//    playSound("notif_1.5s.wav",1.5,5);
//    if (terminal == 1)
//        return;
    playSound("berangkat.wav",1,1);

    mTimer->start(6000);
}

void MainWindow::playAttention(){
    // mTimer->stop();
    std::cout << "play attention" << std::endl;
    playSound("attention.wav",1,1);
}
void MainWindow::localVelCallback(const geometry_msgs::TwistStamped &msg) {
    double y_vec = double(msg.twist.linear.y);
    double x_vec = double(msg.twist.linear.x);
    double velocity_resultant = sqrt(pow(y_vec,2) + pow(x_vec,2));
    velocity_resultant *= 3.6;
    std::ostringstream out;
    out.precision(2);
    out << std::fixed << velocity_resultant;
//    ui->speed_label->setText(QString(out.str().c_str()));
}

void MainWindow::sendControlCallback(const geometry_msgs::Twist &msg) {
    if((int)msg.angular.x & 3)
        playBlinkerSound();
}

void MainWindow::playSound(std::string sound_, double duration, int loop_times) {
    if (play_sound_thread.joinable())
        play_sound_thread.detach();

    play_sound_thread = std::thread(&MainWindow::playSoundThread, this, sound_, duration, loop_times);

}

void MainWindow::playSoundThread(std::string sound_, double duration, int loop_times) {
    std::string path = ros::package::getPath("user_interface");
    path = path + "/media/" + sound_;
//    qprocess.closeReadChannel(QProcess::StandardOutput);
    while(loop_times--){
        QProcess qprocess;
        qprocess.startDetached("play",QStringList()<<path.c_str());
        if(loop_times != 1){
            ros::Rate rr(1/duration);
            rr.sleep();
        }
    }
}

void MainWindow::playBlinkerSound() {
//    if (is_blinker_playing){
//        return;
//    }
////        play_blinker_sound_thread.detach();
//    else{
//        if (play_blinker_sound_thread.joinable())
//            play_blinker_sound_thread.join();
//        play_blinker_sound_thread = std::thread(&MainWindow::playBlinkerSoundThread, this);
//    }
}

void MainWindow::playBlinkerSoundThread() {
    blinker_mtx.lock();
    is_blinker_playing = true;
    blinker_mtx.unlock();

    std::string path = ros::package::getPath("user_interface");
    path = path + "/media/" + "blinker_1s.wav";
//    qprocess.closeReadChannel(QProcess::StandardOutput);
    QProcess qprocess;
    std::cout << "BEFORE DETACH\n";
    qprocess.startDetached("play",QStringList()<<path.c_str());
    ros::Rate rr(1);
    rr.sleep();
    std::cout << "AFTER DETACH\n";

    blinker_mtx.lock();
    is_blinker_playing = false;
    blinker_mtx.unlock();
}
