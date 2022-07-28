#ifndef GPS_H
#define GPS_H

#include <ros/ros.h>
#include <termios.h>
#include <fcntl.h>
#include <fstream>
#include <vector>
#include <common/mavlink.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt8.h"
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>
#include "tf/transform_datatypes.h"

#include <time.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <ctime>
#include <chrono>
#include <cmath>
#include <geometry_msgs/Point32.h>

/*
*   Class for connection to Pixhawk using MAVLink library.
*/

typedef struct {
    double lat;
    double lon;
    double hdg;
} GlobalPos;

using namespace std::chrono;

class GPS
{
private:
    ros::Publisher gps_pub, mavconn_pub;
    ros::Subscriber sub_hdg, sub_imu, sub_imu_raw, sub_gps, sub_gps_2, sub_diagnostic, sub_localpose, sub_cmpcal, sub_headingcorr;
    ros::Subscriber using_compass_sub;
    ros::Subscriber sub_yaw_gyro, offset_remote_gyro_sub;
    ros::NodeHandle n;
    GlobalPos position, position1, position2;         // the current position
    uint8_t cp;
    mavlink_status_t status;
    mavlink_message_t message;
    mavlink_global_position_int_t global_pos;
    mavlink_gps_raw_int_t raw_global_pos;
    mavlink_attitude_t attitude;                    // contains current roll, pitch, yaw in radian
    mavlink_heartbeat_t heartBeat;
    tf::Quaternion quart_pix;
    
    float pixhawkTemp;
    int system_id, companion_id, autopilot_id;

    time_t startTime;
    bool opened;
    int portNum;
    double radToDegrees;
    const char *baudRate = "115200";
    double xGaussData,yGaussData,D;
    char inBuffer[100];
    const char *delim=",";
    int fd, result;
    double latitude, longitude, heading;
    double compass_compensate = 0;
    double yaw_gyro;
    double offset_gyro_from_remote, offset_gyro_from_remote_before;
    double msg_x, msg_y, msg_z;

    bool is_using_headingcorr;

    int init_serial_port(const char*);
    void chatterCallbackHdg(const std_msgs::Float64& msg);
    void chatterCallbackImu(const sensor_msgs::Imu::ConstPtr& msg);
    void chatterCallbackImuRaw(const sensor_msgs::Imu::ConstPtr& msg);
    void chatterCallbackGps(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void chatterCallbackGps2(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void chatterCallbackDiagnostics(const sensor_msgs::Imu::ConstPtr& msg);
    void chatterCallbackLocalPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void chatterCallbackCmpCal(const std_msgs::Float32& msg);
    void chatterCallbackHeadingCorrection(const std_msgs::Float32MultiArray& msg);
    void chatterCallbackGyroYaw(const std_msgs::Float32 msg);
    void offsetGyroRemoteCallback(geometry_msgs::Point32 msg);

    void usingCompassCallback(const std_msgs::UInt8& msg);


public:
    GPS(ros::NodeHandle, double lat=0, double lon=0, double hdg=0);
    GlobalPos getPosition();
    mavlink_attitude_t getAttitude();

    geometry_msgs::Quaternion getQuartenion();
    
    void readPosition();
    void openConnection();
    void readData();
    void checkMavrosConn();
    int isOpened();
    
    ~GPS();

private:
    bool is_using_compass;
    double compass_hdg;
    double imu_hdg;
    double hdg_offset;
    int count_cmpdata;

    high_resolution_clock::time_point begin_time, end_time;
    double diff_time_sec;
    double yaw_imu, yawAcc;

    double hdg_corrected;
};

#endif // GPS_H