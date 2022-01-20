#include <ros/ros.h>
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

// Declare RealSense pipeline, encapsulating the actual device and sensors
rs2::pipeline pipe_rs;
// Create a configuration for configuring the pipeline with a non default profile
rs2::config cfg_rs;

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
    float x, y, z, w;

    // Wait for the next set of frames from the camera
    auto frames = pipe_rs.wait_for_frames();
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
	//	while(!ser.available());

        float pitch = getAngle();

        while(1){
            
            float pitch_curr = getAngle();
            if (pitch_curr > pitch+std::stoi(angle) && cmd_s == "e") {
                break;
            }
            if (pitch_curr < pitch-std::stoi(angle) && cmd_s == "f"){
                break;
            }
        }
        ser.write("q");
		std::cout << "Done" << std::endl;
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
    }
}

void cheat(){
    std::cout << "bum" << std::endl;
    control("f30");
    control("e30");
    control("f30");
    control("e30");
}

void cmd_callback(const std_msgs::String::ConstPtr& msg)
{
    std::cout << msg->data.c_str() << std::endl;
    control(msg->data.c_str());
}

int main (int argc, char** argv){
    ros::init(argc, argv, "control");
    ros::NodeHandle nh;
    ros::Timer timer1;

    // Add pose stream
    cfg_rs.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // Start pipeline with chosen configuration
    pipe_rs.start(cfg_rs);

    ros::Subscriber cmd_sub = nh.subscribe("cmd", 1000, cmd_callback);

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
        

	//    getline(std::cin, tmp);
	//    std::cout << tmp << std::endl;

    //    control(tmp);

    //    cheat(pipe);

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