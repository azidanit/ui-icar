//
// Created by azindait on 08/02/20.
//

#ifndef RVIZ_PLUGIN_CAMERA_PLUGIN_H
#define RVIZ_PLUGIN_CAMERA_PLUGIN_H

#ifndef Q_MOC_RUN

# include <ros/ros.h>
#include <QLabel>
#include <QPoint>
#include <Qt>
#include <QMouseEvent>
#include <QPixmap>
#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QMutex>

# include <rviz/panel.h>
#endif

#include "imgreceiver.h"
#include "imagelabel.h"


//namespace Ui{
//    class cameraClass;
//}

namespace camera_plugin{

class CameraPanel: public rviz::Panel{

Q_OBJECT
public:
    CameraPanel(QWidget* parent = 0);
    ImgReceiver* img_receiver;

public Q_SLOTS:
//    receivedImage(QImage)

protected Q_SLOTS:

protected:
    ImageLabel* cameraFrame;
    QLineEdit* topic;
    QString output_topic_string;
    QPushButton* lbs1_push_button;
    QPushButton* rbs1_push_button;
    QPushButton* cl_push_button;
    QPushButton* publish_push_button;
    QMutex mtx_img;
};


}





#endif //RVIZ_PLUGIN_CAMERA_PLUGIN_H
