#include "imgreceiver.h"
#include <QDebug>
#include<iostream>
//#include <thread>

ImgReceiver::ImgReceiver(QObject *parent)
        :  itrans(nh)
{
    isKilled = false;
    // qDebug() << "imgr : " << QThread::currentThreadId();
}

ImgReceiver::~ImgReceiver(){

}


void ImgReceiver::run(){
    isKilled = false;
    // qDebug() << "img run : " << QThread::currentThreadId();
    do{
        // qDebug()<< "  seeking publisher";
        subs = itrans.subscribe("object_image", 1, &ImgReceiver::receiveImage, this);
    } while(!isKilled && subs.getNumPublishers()==0);
    if(isKilled)
        return;
    qDebug("Got publisher");
    while(isKilled == false){
       ros::spinOnce();

        // while(!isKilled && subs.getNumPublishers()==0) {
        //     // qDebug()<< "  seeking publisher";
        //     subs = itrans.subscribe("object_image", 1, &ImgReceiver::receiveImage, this);
        // } 
    }
    subs.shutdown();
}

void ImgReceiver::kill(){
    mtx.lock();
    qDebug()<<"killed";
    isKilled = true;
    mtx.unlock();
}

void ImgReceiver::receiveImage(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cvPtr;
    try{
        cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e){
        ROS_INFO("cv_bridge exception $s", e.what());
        return;
    }

    
    // img = matToQImage(cvPtr->image);
   Q_EMIT imageReceived(matToQImage(cvPtr->image));
}

QImage* ImgReceiver::matToQImage(const cv::Mat& mat){
    return new QImage((uchar*) mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
}
