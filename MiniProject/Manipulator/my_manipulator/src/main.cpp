#include <iostream>
#include "DynamixelSDK.h"
#include "RobotisManipulator.h"
#include "my_manipulator.h"

int main()
{
	std::string usb_port = "/dev/ttyUSB0";
    std::string baud_rate = "1000000";
	float control_period = 0.010f;
	bool using_platform = true;
	OpenManipulator open_manipulator;
	open_manipulator.initOpenManipulator(using_platform, usb_port, baud_rate, control_period);

	
	return 0;
}
