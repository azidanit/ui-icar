#ifndef IMGRECEIVER_H
#define IMGRECEIVER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <QThread>
#include <QObject>
#include <QMutex>
#include <QImage>
#include <thread>

using namespace cv;

class ImgReceiver : public QThread {
Q_OBJECT;

public:
    ImgReceiver(QObject *parent = Q_NULLPTR);
    ~ImgReceiver();
    QImage* matToQImage(const cv::Mat&);
    ros::NodeHandle nh;
    QImage* img;

public Q_SLOTS:
            void kill();

Q_SIGNALS:
            void imageReceived(QImage*);

private:
    std::thread subsThread;
    image_transport::ImageTransport itrans;
    image_transport::Subscriber subs;
    Mat frame;
    QMutex mtx;
    void run();
    bool isKilled;
    void mulai();
    void processImage(Mat &img);
    void receiveImage(const sensor_msgs::ImageConstPtr& msg);

};

#endif //IMGRECEIVER_H
