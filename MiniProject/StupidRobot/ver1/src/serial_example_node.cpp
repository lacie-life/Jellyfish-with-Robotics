#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Point.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include "tf/transform_datatypes.h"
#include <math.h>

#define PI 3.14159265

serial::Serial ser;

float speed = 0.215;

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

float getAngle(rs2::pipeline pipe){
    float x, y, z, w;

    // Wait for the next set of frames from the camera
    auto frames = pipe.wait_for_frames();
    // Get a frame from the pose stream
    auto f = frames.first_or_default(RS2_STREAM_POSE);
    // Cast the frame to pose_frame and get its data
    auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

    x = pose_data.rotation.x;
    y = pose_data.rotation.y;
    z = pose_data.rotation.z;
    w = pose_data.rotation.w;

    double roll, pitch, yaw;

    pitch =  -asin(2.0 * (x*z - w*y)) * 180.0 / PI;
    roll  =  atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / PI;
    yaw   =  atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / PI;

    std::cout << "Roll: " << roll << " " << "Pitch: " << pitch << " " << "Yaw: " << yaw << std::endl;

    return pitch;
}

void control (std::string tmp, rs2::pipeline pipe)
{
    char cmd = tmp[0];
    std::string cmd_s = "";
    cmd_s = cmd_s + cmd;

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
	//	while(!ser.available());

        float pitch = getAngle(pipe);

        while(1){
            
            float pitch_curr = getAngle(pipe);
            if (pitch_curr > pitch+std::stoi(angle) && cmd_s == "f") {
                break;
            }
            if (pitch_curr < pitch-std::stoi(angle) && cmd_s == "e"){
                break;
            }
        }
        ser.write("q");
		std::cout << "Done" << std::endl;
    }
    else{
 
    	std::string time;
        for (int i = 1; i < tmp.size(); i++){
		    if (tmp[i] >= '0' && tmp[i] <= '9'){
			    time = time + tmp[i];
		    }	 	
    	}
    	std::cout << cmd << " " << time << std::endl;
        std::cout << cmd_s << std::endl;
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
            if ((current.sec - begin.sec) > std::stoi(time) + lost_time) break;
        }
        ser.write("q");
    }
}

void goalControl(geometry_msgs::Point start, geometry_msgs::Point goal, rs2::pipeline pipe)
{
    float angle;
    float time;
    float dir;

    float curr_angle = getAngle(pipe);

    if (curr_angle < 0){
        curr_angle = curr_angle + 360;
    }

    float huongtinhtien = (atan((goal.y-start.y)/(goal.x-start.x))*360)/(2*PI);

    if ((huongtinhtien < 0) && (start.x <= goal.x)){
        huongtinhtien = huongtinhtien + 360;
    }

    if (huongtinhtien < 0 && (start.x > goal.x)){
        huongtinhtien = huongtinhtien + 180;
    }

    if (curr_angle > huongtinhtien){
        if(curr_angle - huongtinhtien < 180){
            dir = 1;
        }
        else dir = 0;
    }
    if (curr_angle < huongtinhtien){
        if(huongtinhtien - curr_angle < 180){
            dir = 0;
        }
        else dir = 1;
    }
    
    time = sqrt((goal.y-start.y)*(goal.y-start.y) + (goal.x-start.x)*(goal.x-start.x))/speed;
    angle = abs(huongtinhtien - curr_angle);

    std::cout << "Dir: " << dir << "Angle: " << angle << "Time: " << time << std::endl;

    std::string cmd_rotation;
    std::string cmd_run; 
    if (dir == 1){

        cmd_rotation = "f" + intToString(static_cast<int>(angle)); 
        cmd_run = "u" + intToString(static_cast<int>(time));

        control(cmd_rotation, pipe);
        control(cmd_run, pipe);

        std::cout << cmd_rotation << " " << cmd_run << std::endl;
    }
    if (dir == 0){
        cmd_rotation = "e" + intToString(static_cast<int>(angle));
        cmd_run = "u" + intToString(static_cast<int>(time));

        control(cmd_rotation, pipe);
        control(cmd_run, pipe);
        
        std::cout << cmd_rotation << " " << cmd_run << std::endl;
    }
}

int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;
    ros::Timer timer1;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    // Add pose stream
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // Start pipeline with chosen configuration
    pipe.start(cfg);

//    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);

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
    geometry_msgs::Point start;
    geometry_msgs::Point goal;
    while(ros::ok()){

        ros::spinOnce();
	    std::string tmp;
	    fflush(stdin);

       // std::cout << "Start: " << std::endl;
       // std::cin >> start.x >> start.y;
      //  std::cout << std::endl;

    //     std::cout << "Goal: " << std::endl;
    //    std::cin >> goal.x >> goal.y;
    //    std::cout << std::endl;

    //    goalControl(start, goal, pipe);

    //  goalControl();
	  getline(std::cin, tmp);
    //	ser.write(tmp);
	  std::cout << tmp << std::endl;

      control(tmp, pipe);
    //  float pitch = getAngle(pipe);
        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            result.data = ser.read(ser.available());
            ROS_INFO_STREAM("Read: " << result.data);
            read_pub.publish(result);
        }
        loop_rate.sleep();
    }
}

