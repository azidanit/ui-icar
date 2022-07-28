#include "car.h"

#include "tf/transform_datatypes.h"

Car::Car(ros::NodeHandle n, const char* venue){
    nh = n;
    offset_sub = n.subscribe("/autonomous_car/gps_offset", 1, &Car::gpsOffsetCallback, this);
    using_odom_sub = n.subscribe("/autonomous_car/using_odom", 1, &Car::usingOdomCallback, this);
    odom_sub = n.subscribe("/car/odometry", 1, &Car::odomCallback, this);
    localpos_sub = n.subscribe("/autonomous_car/localpos", 1, &Car::setLocalPosition, this);
    positioncorr_sub = n.subscribe("/autonomous_car/positioncorr", 1, &Car::positionCorrCallback, this);
    joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tilt = 0;
    tinc = degree;
    swivel=0;
    angle=0;
    height=0;
    hinc=0.005;
    offsetX = 0;
    offsetY = 0;
    offsetA = 0;
    using_odom = 0;
    velocity = 0;

    distance_acc = 0;

    // dummy
    x = 0; y = 0;
/////////////////
    odom_trans.header.frame_id = "world";
    odom_trans.child_frame_id = "chassis";

    initVenue(venue);

    LatitudeCenter = .5 * (LatitudeTop + LatitudeBot);
    LongitudeCenter = .5 * (LongitudeLeft + LongitudeRight);

    printf("%lf %lf\n", LatitudeTop, LongitudeLeft);

    gps = new GPS(n, LatitudeCenter, LongitudeCenter);

    readGPS_thread = std::thread(&Car::readGPS, this);

    begin_time = high_resolution_clock::now();
    // begin_time = clock();
}

Car::~Car(){
    if(readGPS_thread.joinable()){
        readGPS_thread.join();
    }

    std::cout << "Car destroyed" << std::endl;
}

void Car::initVenue(const char* venue){
    std::string filename = ros::package::getPath("map_image") + "/resource/" + venue + ".txt";
    FILE *file = fopen(filename.c_str(), "r");
    fscanf(file, "%lf, %lf %lf, %lf", &LatitudeTop, &LongitudeLeft, &LatitudeBot, &LongitudeRight);
    fclose(file);
}

void Car::update(){
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(3);
    // joint_state.position.resize(3);
    // joint_state.name[0] ="swivel";
    // joint_state.position[0] = swivel;
    // joint_state.name[1] ="tilt";
    // joint_state.position[1] = tilt;
    // joint_state.name[2] ="periscope";
    // joint_state.name[2] ="periscope";
    // joint_state.position[2] = height;

    if(offset_sub.getNumPublishers() <= 0){
        offset_sub = nh.subscribe("/autonomous_car/gps_offset", 1, &Car::gpsOffsetCallback, this);
    }

    if(odom_sub.getNumPublishers() <= 0){
        odom_sub = nh.subscribe("/car/odometry", 1, &Car::odomCallback, this);
    }


    gps_mutex.lock();
    pos.hdg += offsetA;
    attitude = gps->getAttitude();
    gps_mutex.unlock();
    // printf("%lf %lf %f\n", pos.lat, pos.lon, pos.hdg);

    // x += .000001;
    // y += .0000005;
    // if(x>5) x = -5;
    // if(y>5) y = -5;
    // update transform
    odom_trans.header.stamp = ros::Time::now();
    pos = gps->getPosition();
    odom_trans.transform.translation.z = 0;
    end_time = high_resolution_clock::now();
    // end_time = clock();
    if(using_odom==0){
        odom_trans.transform.translation.x = LON_TO_METER * (LongitudeCenter - pos.lon) - offsetX;
        odom_trans.transform.translation.y = LAT_TO_METER * (LatitudeCenter - pos.lat) - offsetY;
    }
    else{
        updatePositionUsingOdom();
    }
    begin_time = high_resolution_clock::now();

//    quart_pix.y = msg->orientation.z;

    tf::Quaternion q;
    geometry_msgs::Quaternion quart_temp;
//    quart_temp = gps->getQuartenion();
//    std::cout << "QUART " << quart_temp;
//    odom_trans.transform.rotation.x = 0;
//    odom_trans.transform.rotation.y = 0;
//    odom_trans.transform.rotation.z = 0;
//    odom_trans.transform.rotation.w = 0;                                                                       0;

    q.setRPY( -attitude.pitch, -attitude.roll , -(pos.hdg) / 180 * M_PI);
    odom_trans.transform.rotation.x = q.x();
    odom_trans.transform.rotation.y = q.y();
    odom_trans.transform.rotation.z = q.z();
    odom_trans.transform.rotation.w = q.w();
//    double roll, pitch, yaw;
    tf::Matrix3x3 m(q);
//    m.getRPY(pitch, roll, yaw);
    // std::cout << (180-pos.hdg) / 180 * M_PI << " : " << yaw << std::endl;

    broadcaster.sendTransform(odom_trans);

    //send the joint state and transform
    joint_pub.publish(joint_state);
}

void Car::readGPS(){
    // if(!gps->isOpened()){
    //     return;
    // }
    ros::Rate r(50);
    while(ros::ok()){
        // if(gps->isOpened()){
        //     gps->readData();
        //     // gps->readPosition();
        // }
        // else{
        //     gps->openConnection();
            
        // }
            // gps->readData();
            gps->checkMavrosConn();
            r.sleep();

    }

    // while(ros::ok()){
    //     gps->readPosition();
    //     usleep(100000);
    // }
    return;
}

void Car::gpsOffsetCallback(const geometry_msgs::Point &msg) {
    gps_mutex.lock();
    if(msg.x == -9999 && msg.y == -9999){
        offsetX = 0;
        offsetY = 0;
    }
    else{
        offsetX += msg.x;
        offsetY += msg.y;
    }
    offsetA = msg.z;
    gps_mutex.unlock();
}

void Car::usingOdomCallback(std_msgs::UInt8 msg){
    odom_mutex.lock();
    using_odom = msg.data;
    distance_acc = 0;
    odom_mutex.unlock();
}

void Car::odomCallback(const car::odometry& msg){
    // std::cout << "ODOM " << msg.velocity << std::endl;
    odom_mutex.lock();
    velocity = msg.velocity;
    odom_mutex.unlock();
}

void Car::updatePositionUsingOdom(){
    duration<double, std::milli> timespan = end_time - begin_time;
    diff_time_sec = timespan.count() / 1000.0;
    distance = velocity * diff_time_sec;
    distance_acc += distance;
    odom_trans.transform.translation.x -= sin(pos.hdg * M_PI/180.0) * distance - offsetX;
    odom_trans.transform.translation.y -= cos(pos.hdg * M_PI/180.0) * distance - offsetY;
    offsetX = 0;
    offsetY = 0;

    // std::cout << "POSITION ODOM " << distance_acc << " " << distance << " " << diff_time_sec << std::endl;
}

void Car::setLocalPosition(const geometry_msgs::Point& msg){
    odom_mutex.lock();
    odom_trans.transform.translation.x = msg.x;
    odom_trans.transform.translation.y = msg.y;
    offsetX = 0;
    offsetY = 0;
    odom_mutex.unlock();
}

void Car::positionCorrCallback(geometry_msgs::Point msg){
    odom_mutex.lock();
    offsetX = msg.x;
    offsetY = msg.y;
    odom_mutex.unlock();
}