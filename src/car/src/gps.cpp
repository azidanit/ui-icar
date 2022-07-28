#include "gps.h"
#include <ctime>
//#include <tf/LinearMath/Matrix3x3.h>

GPS::GPS(ros::NodeHandle n, double lat, double lon, double hdg)
{
	companion_id = 0;
    position.lon =  lon;
    position.lat =  lat;
    position.hdg = hdg;
    position1.lat = 0;
    position1.lon = 0;
    position2.lat = 0;
    position2.lon = 0;
    quart_pix.setX(0);
    quart_pix.setY(0);
    quart_pix.setZ(0);
    quart_pix.setW(0);
    this->n = n;
    // openConnection();
	attitude.yaw = 0;
	attitude.roll = attitude.pitch = 0;
	sub_hdg = n.subscribe("/mavros/global_position/compass_hdg", 1000, &GPS::chatterCallbackHdg, this);
//	sub_imu = n.subscribe("/mavros/imu/data", 1000, &GPS::chatterCallbackImu, this);
//	sub_imu_raw = n.subscribe("/mavros/imu/data_raw", 1000, &GPS::chatterCallbackImuRaw, this);
	sub_localpose = n.subscribe("/mavros/local_position/pose", 1000, &GPS::chatterCallbackLocalPose, this);
   	sub_gps = n.subscribe("/mavros/global_position/global", 1000, &GPS::chatterCallbackGps, this);
   	sub_gps_2 = n.subscribe("/mavros_second/global_position/global", 1000, &GPS::chatterCallbackGps2, this);
    sub_diagnostic = n.subscribe("/diagnostics", 1, &GPS::chatterCallbackDiagnostics, this);
    sub_cmpcal = n.subscribe("/autonomous_car/cmpcal", 1, &GPS::chatterCallbackCmpCal, this);
    sub_headingcorr = n.subscribe("/autonomous_car/headingcorr", 1, &GPS::chatterCallbackHeadingCorrection, this);
    sub_yaw_gyro = n.subscribe("/tl740d/heading", 1, &GPS::chatterCallbackGyroYaw, this);

    offset_remote_gyro_sub = n.subscribe("/serial/offsetGPS", 1, &GPS::offsetGyroRemoteCallback, this);

    mavconn_pub = n.advertise<std_msgs::Int8>("/mavros/conn", 4);
	hdg_offset = 0;
	compass_hdg = -9999;
	count_cmpdata = 0;
    offset_gyro_from_remote = 0;

	// Using Compass / Gyro
	is_using_compass = false;
    using_compass_sub = n.subscribe("/autonomous_car/using_compass", 1, &GPS::usingCompassCallback, this);


	yaw_imu = 0;
	yawAcc = 0;

	hdg_corrected = 0;
	is_using_headingcorr = false;
	begin_time = high_resolution_clock::now();
	// ros::spin();
	// ros::NodeHandle n;	
}

GPS::~GPS()
{
    portNum = -1;
    opened = 0;
}

GlobalPos GPS::getPosition(){
    if(position1.lat!=0){
        if(position2.lat!=0){
            position.lat = (position1.lat + position2.lat)/2.0;
            position.lon = (position1.lon + position2.lon)/2.0;
        }
        else{
            position.lat = position1.lat;
            position.lon = position1.lon;
        }
    }
    else if(position2.lat!=0){
        position.lat = position2.lat;
        position.lon = position2.lon;
    }

	if(is_using_headingcorr){
		position.hdg = hdg_corrected + hdg_offset;
	}
	else if(is_using_compass){
		position.hdg = compass_hdg + hdg_offset;
		// std::cout<<"RETURN Compass\n";

	}
	else{
		position.hdg = yaw_imu + hdg_offset;
	}

	// Normalize heading between 0-360 degree
	if(position.hdg>360){
		position.hdg -= 360;
	}
	else if(position.hdg<0){
		position.hdg += 360;
	}
		
    return position;
}

mavlink_attitude_t GPS::getAttitude(){
	return attitude;
}

void GPS::usingCompassCallback(const std_msgs::UInt8& msg){
	is_using_compass = msg.data;
	std::cout<<"Using Compass : " << (int)(is_using_compass) << std::endl;
}

void GPS::chatterCallbackGps(const sensor_msgs::NavSatFix::ConstPtr& msg){
    position1.lat = msg->latitude;
    position1.lon = msg->longitude;
}

void GPS::chatterCallbackGps2(const sensor_msgs::NavSatFix::ConstPtr& msg){
    position2.lat = msg->latitude;
    position2.lon = msg->longitude;
}

void GPS::chatterCallbackLocalPose(const geometry_msgs::PoseStamped::ConstPtr& msg){
	tf::Matrix3x3 m(quart_pix);
//    tf::Matrix3x3(quart_pix);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

	// position.hdg = (yaw * 180/M_PI);
		// position.hdg = 360 - (yaw * 180/M_PI);
		// if(position.hdg>=360) position.hdg-=360;
		// std::cout << "Hading " << position.hdg << std::endl;
}

void GPS::chatterCallbackCmpCal(const std_msgs::Float32 &msg){
	hdg_offset = msg.data - yaw_imu;
}

void GPS::chatterCallbackImu(const sensor_msgs::Imu::ConstPtr& msg){
    quart_pix.setX(msg->orientation.x);
    quart_pix.setY(msg->orientation.y);
    quart_pix.setZ(msg->orientation.z);
    quart_pix.setW(msg->orientation.w);

    tf::Matrix3x3 m(quart_pix);
//    tf::Matrix3x3(quart_pix);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);


//    attitude.roll = roll;
    attitude.roll = 0;
    // [TODO] : change roll not to use real data, data set to 0
	attitude.pitch = -pitch;
	
		// // position.hdg = (yaw * 180/M_PI);
		// position.hdg = 360 - (yaw * 180/M_PI);
		// if(position.hdg>=360) position.hdg-=360;
		// std::cout << "Hading " << position.hdg << std::endl;

	
//	ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
//	ROS_INFO("RPY roll: [%f], pitch: [%f], yaw: [%f]", roll,pitch,yaw);

}

void GPS::chatterCallbackHeadingCorrection(const std_msgs::Float32MultiArray& msg){
	if(is_using_headingcorr){               // if heading correction was used
        if((int)msg.data[0] < 0.5){         // if the robot is inside a segment where the correction is NOT USED
            yaw_imu = hdg_corrected;        // set yaw_imu to the last corrected heading
            is_using_headingcorr = false;   // disable heading correction
        }
        else{                               // else : if the robot is inside a segment where the correction is USED
            hdg_corrected = -msg.data[1];   // update corrected heading
        }
	}
	else if(msg.data[0]>0.5)                // else (if heading correction is NOT used, and ) the robot is inside a segment where the correction is USED
		is_using_headingcorr = true;        // ebable heading correction
}

void GPS::chatterCallbackImuRaw(const sensor_msgs::Imu::ConstPtr& msg){
	end_time = high_resolution_clock::now();
	duration<double, std::milli> timespan = end_time - begin_time;
	diff_time_sec = timespan.count() / 1000.0;
	yaw_imu -= (msg->angular_velocity.z * diff_time_sec) * 180 / M_PI;

	if(yaw_imu >= 360) yaw_imu -= 360;
	else if (yaw_imu < 0) yaw_imu+=360;
	// position.hdg = yaw_imu + hdg_offset;
	begin_time = high_resolution_clock::now();
	// std::cout << " YAW " << position.hdg << " " << yaw_imu << std::endl;
}

void GPS::chatterCallbackHdg(const std_msgs::Float64& msg)
{
//	std::cout<< "MASUK CHATTER CLA\n";
//	ROS_INFO("CMP HDG: [%f]", msg.data);

// USING COMPASS AS ORIENTATION
	compass_hdg = msg.data + compass_compensate;

// // COMPASS ONLY FOR CALCULATING HEADING OFFSET
// 	double cmp;
// 	cmp = msg.data + compass_compensate;
// 	if(cmp>360){
// 		cmp -= 360;
// 	}
// 	else if(cmp<0){
// 		cmp += 360;
// 	}

// 	compass_hdg = compass_hdg + cmp;


// 	if(count_cmpdata >= 10){
// 		sub_hdg.shutdown();
// 		compass_hdg = compass_hdg / count_cmpdata;
// 		hdg_offset = compass_hdg - yaw_imu;
// 	}
		
//    quart_pix.z = msg.data;

//   ROS_INFO("Imu Seq: [%d]", msg->header.seq);
  
//   ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
}

void GPS::chatterCallbackDiagnostics(const sensor_msgs::Imu::ConstPtr& msg) {
     std_msgs::Int8 conn_msg;
    //  std::cout << "got diag\n";
     conn_msg.data = 1;
     mavconn_pub.publish(conn_msg);
}

void GPS::offsetGyroRemoteCallback(geometry_msgs::Point32 msg){
    offset_gyro_from_remote += msg.z;
    std::cout << "DAPAT OFFSEY GYRO " << msg.z << " ->> " << offset_gyro_from_remote << "\n";
}

void GPS::readPosition(){
    if(position.lat > 1) position.lat = 0;
    else 
		position.lat += .000001;

    if(position.lon < 1) position.lon = 112.8;
    else 
	position.lon -= .000001;

    // if(position.hdg > 360) position.hdg = 0;
    // else position.hdg = 0;
}

//////////////////////////////////////////////
// 					MAVGEN					//
//////////////////////////////////////////////

void GPS::openConnection(){
	opened = false;
    if(!opened){
        FILE* stream;
        std::vector<std::string> portNames;
        char b[32];
        stream = popen("ls /dev/ttyACM*", "r");
        while (!feof(stream)){
            if (fgets(b, 30, stream) != NULL){
                b[strlen(b)-1]=0;
                portNames.push_back(std::string(b));
            }
        }
        pclose(stream);
		auto it=portNames.begin();
		int a=0;
        char tmp[30], tmp_[30], cmd[30], baud[10];
		portNum = -1;
		for(it; it!=portNames.end(); it++){
            memset(baud, 0, sizeof(baud));
            strcpy(tmp, std::string(*it).c_str());
            strcpy(cmd, "stty -F ");
            strcat(cmd, std::string(*it).c_str());
            tmp[strlen(tmp)-1]=0;
            if(strcmp(tmp, "/dev/ttyACM")!=0)continue;
            stream = popen(cmd, "r");
            fgets(tmp_, 30, stream);
            int spaceCounter=0, qq=0;
            for(int q=2; q<strlen(tmp_); q++){
                if(tmp_[q]==' '){
					++spaceCounter;
					continue;
				}
                if(spaceCounter==1){
                    baud[qq]=tmp_[q];
                    ++qq;
                }
                if(spaceCounter==2)break;
            }
            pclose(stream);
            // qDebug()<<baud<<"test";
            // if(strcmp(baud,baudRate)!=0)continue;
			if(init_serial_port(std::string(*it).c_str()) != -1){
				time(&startTime);
				while((time(NULL)-startTime)<2 && portNum==-1){
					result = read(fd, &cp, 1);
					if(result>0) mavlink_parse_char(MAVLINK_COMM_0, cp, &message, &status);
					if(message.msgid == MAVLINK_MSG_ID_HEARTBEAT){
						mavlink_msg_heartbeat_decode(&message, &heartBeat);
						if(heartBeat.type == 2){
							// printf("Port \"%s\" connected to boat's Pixhawk\n", std::string(*it).c_str());
							// opened = true;
							// portNum = a;
							// break;

							system_id = message.sysid;
							autopilot_id = message.compid;
							printf("Port \"%s\" connected to drone's Pixhawk\n", std::string(*it).c_str());
							// m.lock();
							opened = true;
							portNum = a;
							// m.unlock();
							break;
						}
					}
					usleep(1);
				}
			}
			if(opened)break;
			++a;
		}
		if(!opened){
			printf("Error: cannot connect to boat's pixhawk\n");
			portNum = -2;
		}
	}
	if(opened){
		
		mavlink_command_long_t cmd = { 0 };
		cmd.target_system    = system_id;
		cmd.target_component = autopilot_id;
		cmd.command          = MAV_CMD_SET_MESSAGE_INTERVAL;
		cmd.confirmation     = true;
		cmd.param1           = MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
		cmd.param2			 = 10000;
		cmd.param7			 = 0;
		mavlink_message_t message_;

		mavlink_msg_command_long_encode(system_id, companion_id, &message_, &cmd);
		char buf[300];
		unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message_);
		std::cout<<"sending command\n";
		write(fd, buf, len);
		tcdrain(fd);

		// cmd.target_system    = system_id;
		// cmd.target_component = autopilot_id;
		// cmd.command          = MAV_CMD_GET_MESSAGE_INTERVAL;
		// cmd.confirmation     = true;
		// cmd.param1           = MAVLINK_MSG_ID_GLOBAL_POSITION_INT;

		// mavlink_msg_command_long_encode(system_id, companion_id, &message_, &cmd);
		// std::cout<<"sending command\n";
		// write(fd, buf, len);
		// tcdrain(fd);
	}
	else printf("PIXHAWK GAK KONEK\n");
}

void GPS::chatterCallbackGyroYaw(const std_msgs::Float32 msg) {\
    yaw_gyro = msg.data + offset_gyro_from_remote;
    yaw_imu = -(yaw_gyro + 180);

}

int GPS::init_serial_port(const char *port_name){
	fd = open(port_name, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd == -1)return -1;
	else fcntl(fd, F_SETFL, 0);
	struct termios  config;
	if(tcgetattr(fd, &config) < 0)
	{
		fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
		return 1;
	}
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
	config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
	#ifdef OLCUC
		config.c_oflag &= ~OLCUC;
	#endif

	#ifdef ONOEOT
		config.c_oflag &= ~ONOEOT;
	#endif
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;
	config.c_cc[VMIN]  = 0;
	config.c_cc[VTIME] = 10;
	cfsetispeed(&config, B115200);
	cfsetospeed(&config, B115200);
	tcsetattr(fd, TCSAFLUSH, &config);
	return fd;
}

void GPS::readData(){
	// std::cout<<"RAD ATA\n";
	ros::spinOnce();
}

void GPS::checkMavrosConn(){
    if(sub_diagnostic.getNumPublishers()==0) // mavros not running
    {
//        std::cout << "Pub = " << sub_diagnostic.getNumPublishers() << " topic : " << sub_diagnostic.getTopic()<< std::endl;
        std_msgs::Int8 msg;
        msg.data = 0;
        mavconn_pub.publish(msg);
        sub_diagnostic = n.subscribe("/mavros/imu/data_raw", 1, &GPS::chatterCallbackDiagnostics, this);
    }
}

int GPS::isOpened(){
    return opened;
}