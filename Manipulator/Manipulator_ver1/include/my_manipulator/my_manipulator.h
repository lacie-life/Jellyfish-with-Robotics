//
// Created by lacie on 29/08/2019.
//

#ifndef MANIPULATOR_VER1_MY_MANIPULATOR_H
#define MANIPULATOR_VER1_MY_MANIPULATOR_H

#include "dynamixel.h"
#include "custom_trajectory.h"
#include "kinematics.h"
#include "../../include/manipulator_base/manipulator.h"

#define CUSTOM_TRAJECTORY_SIZE 4
#define CUSTOM_TRAJECTORY_LINE    "custom_trajectory_line"
#define CUSTOM_TRAJECTORY_CIRCLE  "custom_trajectory_circle"
#define CUSTOM_TRAJECTORY_RHOMBUS "custom_trajectory_rhombus"
#define CUSTOM_TRAJECTORY_HEART   "custom_trajectory_heart"

#define JOINT_DYNAMIXEL "joint_dxl"
#define TOOL_DYNAMIXEL  "tool_dxl"

#define X_AXIS manipulator_base::math::vector3(1.0, 0.0, 0.0)
#define Y_AXIS manipulator_base::math::vector3(0.0, 1.0, 0.0)
#define Z_AXIS manipulator_base::math::vector3(0.0, 0.0, 1.0)

class superManipulator : public manipulator_base::myManipulator
{

private:
    manipulator_base::Kinematics *kinematics_;
    manipulator_base::JointActuator *actuator_;
    manipulator_base::ToolActuator *tool_;
    manipulator_base::CustomTaskTrajectory *custom_trajectory_[CUSTOM_TRAJECTORY_SIZE];

public:
    superManipulator();
    virtual ~superManipulator();

    void initSuperManipulator(bool using_actual_robot_state, STRING usb_port = "/dev/ttyUSB0", STRING baud_rate = "1000000", float control_loop_time = 0.010);
    void processSuperManipulator(double present_time);
};

#endif //MANIPULATOR_VER1_MY_MANIPULATOR_H
