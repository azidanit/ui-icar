#include <camera_plugin.h>
#include <thread>
#include <iostream>

namespace camera_plugin{

CameraPanel::CameraPanel(QWidget* parent):rviz::Panel( parent ){

    cameraFrame = new ImageLabel(parent);
    img_receiver =  new ImgReceiver();

//    std::thread(img_receiver,3); //new object & new thread
//    std::thread(&ImgReceiver::subsImg,img_receiver);
    
    // connect(img_receiver, SIGNAL(imageReceived(QImage*)),cameraFrame, SLOT(drawImage(QImage*)), Qt::);
//    QImage* img;
//    img_receiver->imageReceived(img);

    //frame camera layout

    std::cout <<"mari connect" << std::endl;
    //vertical layout Boat Side 1
    QHBoxLayout* bs1_layout = new QHBoxLayout;
    bs1_layout->addWidget(lbs1_push_button = new QPushButton);
    bs1_layout->addWidget(rbs1_push_button = new QPushButton);

    //all vertical layout
    QVBoxLayout* layout = new QVBoxLayout;
    layout->addWidget(cameraFrame);
    layout->addLayout(bs1_layout);

    setLayout(layout);

    connect(img_receiver, &ImgReceiver::imageReceived, 
        this,
        [=](QImage* image){
            mtx_img.lock();
            cameraFrame->drawPixmap(image);
            delete image;
            mtx_img.unlock();
        },
        Qt::DirectConnection);
    img_receiver->start();
    std::cout << "camera plugin" << std::endl;
}


} //end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(camera_plugin::CameraPanel,rviz::Panel )