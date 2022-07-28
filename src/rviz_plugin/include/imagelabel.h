#ifndef IMAGELABEL_H
#define IMAGELABEL_H

#include <QLabel>
#include <QPoint>
#include <Qt>
#include <QMouseEvent>
#include <QPixmap>
#include <QPainter>

class ImageLabel : public QLabel{
Q_OBJECT
public:
    ImageLabel(QWidget* parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());
    ~ImageLabel();
    void mousePressEvent(QMouseEvent* event);
    void drawPixmap(QImage* img);
    QPoint** getBoatSide();
    QPoint** getBoatSide2();
    int getCritLine(int);
    void changeControlPoint(int x, int y);
    void controlBalls(int xr, int xm, int ym, int xh, int yh);
    void changeCritLine(int, int);
    // void paintEvent(QPaintEvent*);

private:
    enum {
        CRITLINE = 1, LEFTBOAT = 2, RIGHTBOAT = 3, LEFTBOAT_2 = 4, RIGHTBOAT_2 =5
    };

    const int Width = 640, Height = 480;
    QPoint controlPoint, leftBoat_top, leftBoat_bottom, rightBoat_top, rightBoat_bottom;
    QPoint leftBoat_top2, leftBoat_bottom2, rightBoat_top2, rightBoat_bottom2;
    QPoint* boatSide[4];
    QPoint* boatSide2[4];
    QPoint redBall, greenBall, redFar, greenFar;
    int critLine, horizon;
    int setMode;
    QImage *img;


public Q_SLOTS:
            void changeSetMode(int);
            void drawImage(QImage*);
};

#endif // IMAGELABELH