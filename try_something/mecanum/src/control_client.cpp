#include "ros/ros.h"
#include "mecanum/control.h"
#include <cstdlib>
#include <sensor_msgs/Joy.h>

class TeleopMecanum
{
public:
  TeleopMecanum();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::ServiceClient vel_pub_;
  ros::Subscriber joy_sub_;

};


TeleopMecanum::TeleopMecanum():
  linear_(1),
  angular_(2)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);


  vel_pub_ = nh_.serviceClient<mecanum::control>("mecanum");


  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopMecanum::joyCallback, this);

}

void TeleopMecanum::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  mecanum::control srv;
  std::string cmd;
  int index = 100;
  for (int i = 0; i < 10; i++)
  {
    if (joy->buttons[i] == 1)
    {
      index = i;
      break;
    }
  }
  switch (index)
  {
  case 0:
    cmd = "right";
    break;
  case 1:
    cmd = "left";
    break;
  case 2:
    cmd = "backward";
    break;
  case 3:
    cmd = "forward";
    break;
  default:
    break;
  }
  srv.request.cmd = cmd;
  if (vel_pub_.call(srv))
  {
    ROS_INFO("Mecanum 1: %ld", (long int)srv.response.mecanum1);
  }
  else
  {
    ROS_ERROR("Failed to call service mecanum");
    return;
  }
  
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_client");
  TeleopMecanum teleop_mecanum;

  ros::spin();
}