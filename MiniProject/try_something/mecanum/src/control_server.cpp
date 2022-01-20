#include "ros/ros.h"
#include "mecanum/control.h"
#include "../../src/dynamixel_sdk/dynamixel.h"

#include <iostream>
#include <eigen3/Eigen/Eigen>

#define PROTOCOL_VERSION1               1.0                 // See which protocol version is used in the Dynamixel
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller

using namespace Eigen;
using namespace std;

#define L1 22.5/2
#define L2 24.5/2
#define R 0.33
#define PI 3.14159265359

#define MECANUM_1 2
#define MECANUM_2 3
#define MECANUM_3 5
#define MECANUM_4 15

// Motor Torque On/Off
#define ADDRESS_TORQUE       24
//Limit of angle 0-300 degree
#define CW_ANGLE_LIMIT   0
#define CCW_ANGLE_LIMIT  0

#define ADDRESS_SET_MOVING_SPEED 32
#define ADDRESS_GET_MOVING_SPEED 38
#define ADDRESS_CW           6
#define ADDRESS_CCW          8

#define MOVING_FORWARD_SPEED 512
#define MOVING_BACKWARD_SPEED 1536
#define STOP 0

Matrix2f get_current_pose()
{
  Matrix2f pose;
  return pose;
}

char *dev_name = (char*)DEVICENAME;

void setupDynamixel(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler)
{
  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n\n");
    printf(" - Device Name : %s\n", dev_name);
    portHandler->setBaudRate(1000000);
    printf(" - Baudrate    : %d\n\n", portHandler->getBaudRate());
  }

  // Turn on the torque and lock EEPROM area
  _write(portHandler, packetHandler, MECANUM_1, ADDRESS_TORQUE, 2, 1);
  _write(portHandler, packetHandler, MECANUM_2, ADDRESS_TORQUE, 2, 1);
  _write(portHandler, packetHandler, MECANUM_3, ADDRESS_TORQUE, 2, 1);
  _write(portHandler, packetHandler, MECANUM_4, ADDRESS_TORQUE, 2, 1);

  // Set wheel mode
  _write(portHandler, packetHandler, MECANUM_1, ADDRESS_CW, 2, CW_ANGLE_LIMIT);
  _write(portHandler, packetHandler, MECANUM_1, ADDRESS_CCW, 2, CCW_ANGLE_LIMIT);

  _write(portHandler, packetHandler, MECANUM_2, ADDRESS_CW, 2, CW_ANGLE_LIMIT);
  _write(portHandler, packetHandler, MECANUM_2, ADDRESS_CCW, 2, CCW_ANGLE_LIMIT);

  _write(portHandler, packetHandler, MECANUM_3, ADDRESS_CW, 2, CW_ANGLE_LIMIT);
  _write(portHandler, packetHandler, MECANUM_3, ADDRESS_CCW, 2, CCW_ANGLE_LIMIT);

  _write(portHandler, packetHandler, MECANUM_4, ADDRESS_CW, 2, CW_ANGLE_LIMIT);
  _write(portHandler, packetHandler, MECANUM_4, ADDRESS_CCW, 2, CCW_ANGLE_LIMIT);
}

void stop(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler)
{

  _write(portHandler, packetHandler, MECANUM_1, ADDRESS_SET_MOVING_SPEED, 2, STOP);
  _write(portHandler, packetHandler, MECANUM_2, ADDRESS_SET_MOVING_SPEED, 2, STOP);
  _write(portHandler, packetHandler, MECANUM_3, ADDRESS_SET_MOVING_SPEED, 2, STOP);
  _write(portHandler, packetHandler, MECANUM_4, ADDRESS_SET_MOVING_SPEED, 2, STOP);
}

void forward(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler)
{
  _write(portHandler, packetHandler, MECANUM_1, ADDRESS_SET_MOVING_SPEED, 2, MOVING_FORWARD_SPEED);
  _write(portHandler, packetHandler, MECANUM_2, ADDRESS_SET_MOVING_SPEED, 2, MOVING_FORWARD_SPEED);
  _write(portHandler, packetHandler, MECANUM_3, ADDRESS_SET_MOVING_SPEED, 2, MOVING_FORWARD_SPEED);
  _write(portHandler, packetHandler, MECANUM_4, ADDRESS_SET_MOVING_SPEED, 2, MOVING_FORWARD_SPEED);
}

void backward(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler)
{
  _write(portHandler, packetHandler, MECANUM_1, ADDRESS_SET_MOVING_SPEED, 2, MOVING_BACKWARD_SPEED);
  _write(portHandler, packetHandler, MECANUM_2, ADDRESS_SET_MOVING_SPEED, 2, MOVING_BACKWARD_SPEED);
  _write(portHandler, packetHandler, MECANUM_3, ADDRESS_SET_MOVING_SPEED, 2, MOVING_BACKWARD_SPEED);
  _write(portHandler, packetHandler, MECANUM_4, ADDRESS_SET_MOVING_SPEED, 2, MOVING_BACKWARD_SPEED);
}

void left(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler)
{
  _write(portHandler, packetHandler, MECANUM_1, ADDRESS_SET_MOVING_SPEED, 2, MOVING_FORWARD_SPEED);
  _write(portHandler, packetHandler, MECANUM_2, ADDRESS_SET_MOVING_SPEED, 2, MOVING_BACKWARD_SPEED);
  _write(portHandler, packetHandler, MECANUM_3, ADDRESS_SET_MOVING_SPEED, 2, MOVING_FORWARD_SPEED);
  _write(portHandler, packetHandler, MECANUM_4, ADDRESS_SET_MOVING_SPEED, 2, MOVING_BACKWARD_SPEED);
}

void right(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler)
{
  _write(portHandler, packetHandler, MECANUM_1, ADDRESS_SET_MOVING_SPEED, 2, MOVING_BACKWARD_SPEED);
  _write(portHandler, packetHandler, MECANUM_2, ADDRESS_SET_MOVING_SPEED, 2, MOVING_FORWARD_SPEED);
  _write(portHandler, packetHandler, MECANUM_3, ADDRESS_SET_MOVING_SPEED, 2, MOVING_BACKWARD_SPEED);
  _write(portHandler, packetHandler, MECANUM_4, ADDRESS_SET_MOVING_SPEED, 2, MOVING_FORWARD_SPEED);
}

void leftDiagonalForward(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler)
{
  _write(portHandler, packetHandler, MECANUM_1, ADDRESS_SET_MOVING_SPEED, 2, MOVING_FORWARD_SPEED);
  _write(portHandler, packetHandler, MECANUM_2, ADDRESS_SET_MOVING_SPEED, 2, STOP);
  _write(portHandler, packetHandler, MECANUM_3, ADDRESS_SET_MOVING_SPEED, 2, MOVING_FORWARD_SPEED);
  _write(portHandler, packetHandler, MECANUM_4, ADDRESS_SET_MOVING_SPEED, 2, STOP);
}

void leftDiagonalBackward(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler)
{
  _write(portHandler, packetHandler, MECANUM_1, ADDRESS_SET_MOVING_SPEED, 2, MOVING_BACKWARD_SPEED);
  _write(portHandler, packetHandler, MECANUM_2, ADDRESS_SET_MOVING_SPEED, 2, STOP);
  _write(portHandler, packetHandler, MECANUM_3, ADDRESS_SET_MOVING_SPEED, 2, MOVING_BACKWARD_SPEED);
  _write(portHandler, packetHandler, MECANUM_4, ADDRESS_SET_MOVING_SPEED, 2, STOP);
}

void rightDiagonalForward(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler)
{
  _write(portHandler, packetHandler, MECANUM_1, ADDRESS_SET_MOVING_SPEED, 2, STOP);
  _write(portHandler, packetHandler, MECANUM_2, ADDRESS_SET_MOVING_SPEED, 2, MOVING_FORWARD_SPEED);
  _write(portHandler, packetHandler, MECANUM_3, ADDRESS_SET_MOVING_SPEED, 2, STOP);
  _write(portHandler, packetHandler, MECANUM_4, ADDRESS_SET_MOVING_SPEED, 2, MOVING_FORWARD_SPEED);
}

void rightDiagonalBackward(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler)
{
  _write(portHandler, packetHandler, MECANUM_1, ADDRESS_SET_MOVING_SPEED, 2, STOP);
  _write(portHandler, packetHandler, MECANUM_2, ADDRESS_SET_MOVING_SPEED, 2, MOVING_BACKWARD_SPEED);
  _write(portHandler, packetHandler, MECANUM_3, ADDRESS_SET_MOVING_SPEED, 2, STOP);
  _write(portHandler, packetHandler, MECANUM_4, ADDRESS_SET_MOVING_SPEED, 2, MOVING_BACKWARD_SPEED);
}

bool control(mecanum::control::Request  &req,
         mecanum::control::Response &res)
{
  
  // Initialize Packethandler1 instance
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION1);
  
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(dev_name);
  
  //setupDynamixel(portHandler, packetHandler);

  if(req.cmd == "stop")
  {
    std::cout << "stop" << std::endl;
  //  stop(portHandler, packetHandler);
  }
  else if(req.cmd == "left")
  {
    std::cout << "left" << std::endl;
  //  left(portHandler, packetHandler);
  }
  else if(req.cmd == "forward")
  {
    std::cout << "forward" << std::endl;
  //  forward(portHandler, packetHandler);
  }
  else if(req.cmd == "backward")
  {
    std::cout << "backward" << std::endl;
  //  backward(portHandler, packetHandler);
  }
  else if(req.cmd == "right")
  {
    std::cout << "right" << std::endl;
  // right(portHandler, packetHandler);
  }
  else if(req.cmd == "leftforward")
  {
    leftDiagonalForward(portHandler, packetHandler);
  }
  else if(req.cmd == "leftbackward")
  {
    leftDiagonalBackward(portHandler, packetHandler);
  }
  else if(req.cmd == "rightforward")
  {
    rightDiagonalForward(portHandler, packetHandler);
  }
  else if(req.cmd == "rightbackward")
  {
    rightDiagonalBackward(portHandler, packetHandler);
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("mecanum", control);
  ROS_INFO("Ready to control");
  ros::spin();

  return 0;
}