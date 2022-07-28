#include "imagelabel.h"
#include <QPoint>
#include <QDebug>

ImageLabel::ImageLabel(QWidget*parent, Qt::WindowFlags f)
        : QLabel(parent)
{
    setMode = 0;
    controlPoint = QPoint(-1,-1);
    critLine = 200;
    horizon = 0;
    int top_x = 200,
            top_y = 350,
            bot_x = 10,
            bot_y = 470;
    leftBoat_top = QPoint(top_x, top_y);
    leftBoat_bottom = QPoint(bot_x, bot_y);
    rightBoat_top = QPoint(Width-top_x-1, top_y);
    rightBoat_bottom = QPoint(Width-bot_x-1, bot_y);
    leftBoat_top2 = QPoint(top_x, top_y);
    leftBoat_bottom2 = QPoint(bot_x, bot_y);
    rightBoat_top2 = QPoint(Width-top_x-1, top_y);
    rightBoat_bottom2 = QPoint(Width-bot_x-1, bot_y);
    boatSide[0] = &leftBoat_top;
    boatSide[1] = &leftBoat_bottom;
    boatSide[2] = &rightBoat_top;
    boatSide[3] = &rightBoat_bottom;
    boatSide2[0] = &leftBoat_top2;
    boatSide2[1] = &leftBoat_bottom2;
    boatSide2[2] = &rightBoat_top2;
    boatSide2[3] = &rightBoat_bottom2;
    img = NULL;
}

ImageLabel::~ImageLabel(){

}

void ImageLabel::drawImage(QImage* img){
    drawPixmap(img);
}

void ImageLabel::controlBalls(int id, int xm, int ym, int xh, int yh){
    if(id){
        redFar = QPoint(xm,ym);
        greenFar = QPoint(xh,yh);
    }
    else{
        redBall = QPoint(xm, ym);
        greenBall = QPoint(xh, yh);
    }
}

void ImageLabel::mousePressEvent(QMouseEvent* event){
    if (setMode == 0) return;
    QPoint pos = event->pos();
    if(setMode == CRITLINE){
        if(event->button() == Qt::LeftButton)
            critLine = pos.y();
        else if(event->button() == Qt::RightButton)
            horizon = pos.y();
    }
    else if(setMode == LEFTBOAT){
        if(event->button() == Qt::LeftButton){
            leftBoat_top = pos;
            rightBoat_top = QPoint(Width - pos.x()-1, pos.y());
        }
        else if(event->button() == Qt::RightButton){
            leftBoat_bottom = pos;
            rightBoat_bottom = QPoint(Width - pos.x()-1, pos.y());
        }
    }
    else if(setMode==RIGHTBOAT){
        if(event->button() == Qt::LeftButton){
            rightBoat_top = pos;
            leftBoat_top = QPoint(Width - pos.x(), pos.y());
        }
        else if(event->button() == Qt::RightButton){
            rightBoat_bottom = pos;
            leftBoat_bottom = QPoint(Width - pos.x(), pos.y());
        }
    }
    else if(setMode == LEFTBOAT_2){
        if(event->button() == Qt::LeftButton){
            leftBoat_top2 = pos;
            rightBoat_top2 = QPoint(Width - pos.x()-1, pos.y());
        }
        else if(event->button() == Qt::RightButton){
            leftBoat_bottom2 = pos;
            rightBoat_bottom2 = QPoint(Width - pos.x()-1, pos.y());
        }
    }
    else if(setMode==RIGHTBOAT_2){
        if(event->button() == Qt::LeftButton){
            rightBoat_top2 = pos;
            leftBoat_top2 = QPoint(Width - pos.x(), pos.y());
        }
        else if(event->button() == Qt::RightButton){
            rightBoat_bottom2 = pos;
            leftBoat_bottom2 = QPoint(Width - pos.x(), pos.y());
        }
    }
}

void ImageLabel::changeSetMode(int mode){
    setMode = mode;
}

void ImageLabel::drawPixmap(QImage *img){

    if(img==NULL) return;
    QPainter painter(img);
    painter.setPen(QPen(Qt::red, 2));
    painter.drawLine(0, critLine, Width-1, critLine);

    painter.setPen(QPen(Qt::black, 2));
    painter.drawLine(0, horizon, Width-1, horizon);

    painter.setPen(QPen(Qt::blue, 2));
    painter.drawLine(leftBoat_top, leftBoat_bottom);
    painter.drawLine(rightBoat_top, rightBoat_bottom);

    painter.setPen(QPen(Qt::darkMagenta, 2));
    painter.drawLine(leftBoat_top2, leftBoat_bottom2);
    painter.drawLine(rightBoat_top2, rightBoat_bottom2);

    painter.setPen(QPen(Qt::magenta, 10));
    if(redBall.x() != -1 && redBall.y() != -1)
        painter.drawPoint(redBall);
    if(greenBall.x() != -1 && greenBall.y() != -1)
        painter.drawPoint(greenBall);

    painter.setPen(QPen(Qt::cyan, 10));
    painter.drawPoint(redFar);
    painter.drawPoint(greenFar);
    painter.setPen(QPen(Qt::yellow, 5));
    if(controlPoint.x() != -1 && controlPoint.y() != -1)
        painter.drawPoint(controlPoint);
    painter.end();
    this->setPixmap(QPixmap::fromImage(*img));
    update();
}

QPoint** ImageLabel::getBoatSide(){
    return boatSide;
}

QPoint** ImageLabel::getBoatSide2(){
    return boatSide2;
}

int ImageLabel::getCritLine(int i){
    return i ? critLine : horizon;
}

void ImageLabel::changeCritLine(int i, int crit){
    if(i) critLine = crit;
    else horizon = crit;
}

void ImageLabel::changeControlPoint(int x, int y){
    controlPoint = QPoint(x, y);
}
