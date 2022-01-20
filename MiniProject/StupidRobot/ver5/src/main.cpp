#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include <math.h>

#define PI 3.14159265

serial::Serial ser;

float speed = 0.215;

float pose_orientation_x, pose_orientation_y, pose_orientation_z, pose_orientation_w; 
float pose_position_x, pose_position_y, pose_position_z; 
double roll, pitch, yaw;



void odom_callback(const nav_msgs::Odometry::ConstPtr& msg){

    // ROS_INFO("Seq: [%d]", msg->header.seq);
    // ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
    // ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    // ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);

    pose_orientation_x = msg->pose.pose.orientation.x;
    pose_orientation_y = msg->pose.pose.orientation.y;
    pose_orientation_z = msg->pose.pose.orientation.z;
    pose_orientation_w = msg->pose.pose.orientation.w;

    pose_position_x = msg->pose.pose.position.x;
    pose_position_y = msg->pose.pose.position.y;
    pose_position_z = msg->pose.pose.position.z;

    float x, y, z, w;
    x = pose_orientation_x;
    y = pose_orientation_y;
    z = pose_orientation_z;
    w = pose_orientation_w;
   
    pitch =  -asin(2.0 * (x*z - w*y)) * 180.0 / PI;
    roll  =  atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / PI;
    yaw   =  atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / PI;

//    std::cout << "Roll: " << roll << " " << "Pitch: " << pitch << " " << "Yaw: " << yaw << std::endl;
}

float meterToTime(float meter)
{
    float time = meter / speed;
    return time;
}

void sendCmd(std::string cmd){
    if (cmd == "u"){
        ser.write("u");
    }

    else if (cmd == "d"){
        ser.write("d");
    }

    else if (cmd == "f"){
        ser.write("f");
    }

    else if (cmd == "e"){
        ser.write("e");
    }

    else if (cmd == "l"){
        ser.write("l");
    }

    else if (cmd == "r"){
        ser.write("r");
    }
}

std::string intToString (int a){
    std::string str = std::to_string(a);
    return str;
}

float getAngle(){

    // float x, y, z, w;
    // x = pose_orientation_x;
    // y = pose_orientation_y;
    // z = pose_orientation_z;
    // w = pose_orientation_w;

    // double roll, pitch, yaw;

    // pitch =  -asin(2.0 * (x*z - w*y)) * 180.0 / PI;
    // roll  =  atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / PI;
    // yaw   =  atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / PI;

    // std::cout << "Roll: " << roll << " " << "Pitch: " << pitch << " " << "Yaw: " << yaw << std::endl;

    return yaw;
}

void control (std::string tmp)
{
    char cmd = tmp[0];
    std::string cmd_s = "";
    cmd_s = cmd_s + cmd;
    std::cout << cmd_s << std::endl;
    std::string::size_type sz;     // alias of size_t

    if ( cmd_s == "f" || cmd_s == "e"){

	    sendCmd(cmd_s);
        std::string angle;
	    for (int i = 1; i < tmp.size(); i++){
	  	    if (tmp[i] >= '0' && tmp[i] <= '9'){
			    angle = angle + tmp[i];
		    }	 	
    	}
        // ser.write(angle);
        std::cout << cmd_s << " " << angle << std::endl;
        std::cout << "bum bum" << std::endl;
	//	while(!ser.available());

        float yaw = getAngle();
        std::cout << yaw << std::endl;
        float xxx;

        if (cmd_s == "e"){
            if (pitch + std::stoi(angle) >= 180){
                xxx = 180 - (pitch + std::stoi(angle));   
            }
            else{
                xxx = pitch + std::stoi(angle);
            }

            while(1){
                float pitch_curr = getAngle();
                // std::cout << pitch_curr << std::endl;
                if (pitch_curr > xxx) {
                    
                    ser.write("q");
		            std::cout << "Done" << std::endl;
                    break;
                }           
            }

        }
        else if (cmd_s == "f"){
            if (pitch - std::stoi(angle) <= -180){
                xxx = 306 + (pitch - std::stoi(angle));
            }
            else{
                xxx = pitch - std::stoi(angle);
            }
            std::cout << "xxx " << xxx << std::endl;
            while(1){
                float pitch_curr = getAngle();
                std::cout << "current " << pitch_curr << std::endl;
                if (pitch_curr < xxx) {
                    
                    ser.write("q");
		            std::cout << "Done" << std::endl;
                    break;
                }            
            }
        }
    }    

    else{
 
    	std::string meter;
        for (int i = 1; i < tmp.size(); i++){
		    if (tmp[i] >= '0' && tmp[i] <= '9'){
			    meter = meter + tmp[i];
		    }	 	
    	}
    	std::cout << cmd << " " << meter << std::endl;
        std::cout << cmd_s << std::endl;

        float meter_f = std::stof(meter,&sz);
        std::cout << meter_f << std::endl;

        float time = meterToTime(meter_f);
        std::cout << time << std::endl;

	    sendCmd(cmd_s);
		ros::Time begin = ros::Time::now();
		int lost_time = 0;

	    while(1){
	        ros::Time current = ros::Time::now();

                // Arduino sent Ob signal
	        if (ser.available()){
		        std_msgs::String result;
		        result.data = ser.read(ser.available());

                // Check Ob
		        if (result.data == "0"){
			        ros::Time curr_ob = ros::Time::now();

                // Waiting 3s
			        while(1){
				        ros::Time tmp = ros::Time::now();

                        // Ob
				        if ((tmp.sec - curr_ob.sec) > 5) {
                            std::cout << " Ob " << std::endl;
                            ser.write("q");
                        //    break;
                        }  
                        // Ob moved
					    if (ser.available()) {
					        std_msgs::String result;
					        result.data = ser.read(ser.available());

					        if (result.data == "1") {
						        ros::Time tmp1 = ros::Time::now();
						        lost_time = tmp1.sec - curr_ob.sec;
                                break;
					        }
				        }
			        } 
		        }
		    }
            if ((current.sec - begin.sec) > time + lost_time) break;
        }
        ser.write("q");
        std::cout << "Done" << std::endl;

        std::cout << "Pose: " << std::endl;
        std::cout << pose_position_x << " " << pose_position_y << " " << pose_position_z << std::endl;
    }
}

void cmd_callback(const std_msgs::String::ConstPtr& msg)
{
  control(msg->data.c_str());
}

void cheat(){
    std::cout << "bum" << std::endl;
    control("f30");
    control("e30");
    control("f30");
    control("e30");
}

int main (int argc, char** argv){
    ros::init(argc, argv, "control");
    ros::NodeHandle nh;
    ros::Timer timer1;

    ros::Subscriber write_sub = nh.subscribe("/camera/odom/sample", 1000, odom_callback);
    ros::Subscriber cmd_sub = nh.subscribe("cmd", 1000, cmd_callback);
//    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);

    try
    {
        ser.setPort("/dev/ttyACM0");
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    ros::Rate loop_rate(5);

//    std::cout << "bum chiu" << std::endl;

  //  cheat(pipe);

    while(ros::ok()){

        ros::spinOnce();
	    std::string tmp;
	    fflush(stdin);
        

	    // getline(std::cin, tmp);
	    // std::cout << tmp << std::endl;

        // control(tmp);

    //    cheat(pipe);
        float yaw = getAngle();

        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            result.data = ser.read(ser.available());
            ROS_INFO_STREAM("Read: " << result.data);
//            read_pub.publish(result);
        }
        loop_rate.sleep();
    }
}
