#ifndef BOAT_H
#define BOAT_H

#include <string>
#include <math.h>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/JointState.h>
#include <car/odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "gps.h"
#include <geometry_msgs/Quaternion.h>

#include <chrono>

using namespace std;

#define LAT_TO_METER 111000
#define LON_TO_METER 113321

/*
* ROBOTIK 
** top left     : -7.277125, 112.798264
** bottom right : -7.277631, 112.799117


*
** top left     : -7.290732, 112.798620
** bottom right : -7.291262, 112.799175

* DANAU 8 ( 100 x 140 )
** top left     : -7.286116, 112.795583
** bottom right : -7.287329, 112.796532
*/
using namespace std::chrono;
class Car{
private:
    ros::NodeHandle nh;
    ros::Publisher joint_pub;
    tf::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped odom_trans;
    geometry_msgs::Quaternion quart_pix;
    sensor_msgs::JointState joint_state;
    const double degree = M_PI/180;
    double LatitudeTop, LatitudeBot;
    double LongitudeLeft, LongitudeRight;
    double LatitudeCenter, LongitudeCenter;
    double tilt, tinc, swivel, angle, height, hinc;
    GPS *gps;
    GlobalPos pos;
    mavlink_attitude_t attitude;                    // contains current roll, pitch, yaw in radian
    double x,y;

    std::thread readGPS_thread;
    std::mutex gps_mutex, odom_mutex;
    
    void initVenue(const char*);
    void readGPS();

    public:
    Car(ros::NodeHandle, const char*);
    ~Car();
    void update();
    inline void updatePositionUsingOdom();
    // void updatePositionUsingGPS();
    // void calculateGlobalPosition(double x, double y, double &latitude, double &longitude);

// GPS Offset
private:
    uint8_t using_odom;
    double offsetX, offsetY, offsetA;
    double velocity, distance;
    ros::Subscriber offset_sub, using_odom_sub, odom_sub, localpos_sub, positioncorr_sub;
    void gpsOffsetCallback(const geometry_msgs::Point& msg);
    void usingOdomCallback(std_msgs::UInt8 msg);
    void odomCallback(const car::odometry& msg);
    void setLocalPosition(const geometry_msgs::Point& msg);
    void positionCorrCallback(geometry_msgs::Point msg);

    // time to calculate odom
    high_resolution_clock::time_point begin_time, end_time;
    double diff_time_sec;
    double distance_acc;
};

#endif // BOAT_H