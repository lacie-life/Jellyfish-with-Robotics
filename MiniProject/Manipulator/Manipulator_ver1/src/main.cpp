#include <iostream>
#include <string>
#include <vector>
#include <ctime>
#include "my_manipulator.h"

#define DXL_SIZE 5

superManipulator super_manipulator;
double control_time = 0.010;
double present_time = 0.0;
double previous_time = 0.0;
bool platform_state = true;

typedef struct _MotionWaypoint
{
    std::vector<double> angle;
    double path_time;
} MotionWaypoint;

std::vector<MotionWaypoint> motion_way_point_buf;
bool processing_motion_state = false;
char hand_motion_cnt = 0;
bool hand_motion_repeat_state = false;
bool platform_state_processing = false;
std::string global_cmd[50];

double getCurrentTime();
void split(std::string data, char separator, std::string* temp);
std::string* paraseDataFromProcessign(std::string get);
void sendAngleToProcessing(JointWayPoint joint_states_vector);
void sendValueToProcessing(superManipulator *super_manipulator);
void fromProcessing(superManipulator *super_manipulator, std::string data);
void playProcessingMotion(superManipulator *super_manipulator);
void getData(uint32_t wait_time);

int main() {
    super_manipulator.initSuperManipulator(platform_state);
    super_manipulator.enableAllActuator();
    super_manipulator.enableAllJointActuator();
    super_manipulator.printManipulatorSetting();

    TaskWayPoint draw_line_arg;
    draw_line_arg.kinematic.position(0) = 0.02;
    draw_line_arg.kinematic.position(1) = 0.02;
    draw_line_arg.kinematic.position(2) = -0.02;
    void *p_draw_line_arg = &draw_line_arg;
    super_manipulator.makeCustomTrajectory(CUSTOM_TRAJECTORY_LINE, "joint4", p_draw_line_arg, 1.0);
    //playProcessingMotion(&super_manipulator);
    //std::cout << "Trajectory" << std::endl;

    super_manipulator.printManipulatorSetting();

    present_time = getCurrentTime()/1000.0;
    playProcessingMotion(&super_manipulator);


    if(present_time-previous_time >= control_time)
    {
        super_manipulator.processSuperManipulator(getCurrentTime()/1000.0);

        previous_time = getCurrentTime()/1000.0;
        sendValueToProcessing(&super_manipulator);
    }
    return 0;
}

double getCurrentTime()
{
    time_t timer;
    struct tm y2k = {0};
    double seconds;

    y2k.tm_hour = 0;   y2k.tm_min = 0; y2k.tm_sec = 0;
    y2k.tm_year = 100; y2k.tm_mon = 0; y2k.tm_mday = 1;

    time(&timer);  /* get current time; same as: timer = time(NULL)  */

    seconds = difftime(timer,mktime(&y2k));

    return seconds;
}

void split(std::string data, char separator, std::string* temp)
{
    int cnt = 0;
    int get_index = 0;

    std::string copy = data;

    for (int index = 0; index < copy.length(); index++)
    {
        if (copy[index] == separator)
        {
            for (int i = 0; i < index+1; i++)
            {
                temp[cnt] = temp[cnt] + copy[i];
            }
            std::string copy_ = "";

            for (int i = get_index+1; i < copy.length(); i++)
            {
                copy_ = copy_ + copy[i];
            }
            copy = copy_;
            continue;
        }
        else if (copy[index] != separator && (index = copy.length() - 1))
        {
            for ( int i = 0; i < copy.length(); i++)
            {
                temp[cnt] = temp[cnt] + copy[i];
            }
            break;
        }
    }

//    while(true)
//    {
//        get_index = copy.indexOf(separator);
//
//        if(-1 != get_index)
//        {
//            temp[cnt] = copy.substring(0, get_index);
//            copy = copy.substring(get_index + 1);
//        }
//        else
//        {
//            temp[cnt] = copy.substring(0, copy.length());
//            break;
//        }
//        ++cnt;
//    }
}

std::string* parseDataFromProcessing(std::string get)
{
    split(get, ',', global_cmd);

    return global_cmd;
}

void sendAngleToProcessing(JointWayPoint joint_states_vector)
{
    std::cout << "angle" << std::endl;
    for (int i = 0; i < (int)joint_states_vector.size(); i++)
    {
        std::cout << "," << joint_states_vector.at(i).position << "\t" ;
    }
    std::cout << "\n";
}

void sendValueToProcessing(superManipulator *super_manipulator)
{
    sendAngleToProcessing(super_manipulator->getAllActiveJointValue());
}

void fromProcessing(superManipulator *super_manipulator, std::string data)
{
    std::string *cmd = parseDataFromProcessing(data);
    std::cout << cmd[0] << "\t" << cmd[1] << std::endl;
    cmd[0] = "motion";
    cmd[1] = "1";
    if (cmd[0] == "opm")
    {
        if (cmd[1] == "ready")
        {
            if(platform_state_processing)
            {
                super_manipulator->enableAllActuator();
                sendValueToProcessing(super_manipulator);
            }
        }
        else if (cmd[1] == "end")
        {
            if(platform_state_processing)
            {
                super_manipulator->disableAllActuator();
            }
        }
    }
        ////////// joint space control tab
    else if (cmd[0] == "joint")
    {
        std::vector<double> goal_position;
        for (uint8_t index = 0; index < DXL_SIZE; index++)
        {
            std::string::size_type sz;     // alias of size_t

            double temp_ = std::stod (cmd[index+1],&sz);
            goal_position.push_back(temp_);
        }
        super_manipulator->makeJointTrajectory(goal_position, 1.0); // FIX TIME PARAM
    }
//    else if (cmd[0] == "gripper")
//    {
//        open_manipulator->makeToolTrajectory("gripper", (double)cmd[1].toFloat());
//    }
//    else if (cmd[0] == "grip")
//    {
//        if (cmd[1] == "on")
//            open_manipulator->makeToolTrajectory("gripper", -0.009);
//        else if (cmd[1] == "off")
//            open_manipulator->makeToolTrajectory("gripper", 0.009);
//    }
        ////////// task space control tab
//    else if (cmd[0] == "task")
//    {
//        if (cmd[1] == "forward")
//            open_manipulator->makeTaskTrajectoryFromPresentPose("gripper", math::vector3(0.010, 0.0, 0.0), 0.2);
//        else if (cmd[1] == "back")
//            open_manipulator->makeTaskTrajectoryFromPresentPose("gripper", math::vector3(-0.010, 0.0, 0.0), 0.2);
//        else if (cmd[1] == "left")
//            open_manipulator->makeTaskTrajectoryFromPresentPose("gripper", math::vector3(0.0, 0.010, 0.0), 0.2);
//        else if (cmd[1] == "right")
//            open_manipulator->makeTaskTrajectoryFromPresentPose("gripper", math::vector3(0.0, -0.010, 0.0), 0.2);
//        else if (cmd[1] == "up")
//            open_manipulator->makeTaskTrajectoryFromPresentPose("gripper", math::vector3(0.0, 0.0, 0.010), 0.2);
//        else if (cmd[1] == "down")
//            open_manipulator->makeTaskTrajectoryFromPresentPose("gripper", math::vector3(0.0, 0.0, -0.010), 0.2);
//        else
//            open_manipulator->makeTaskTrajectoryFromPresentPose("gripper", math::vector3(0.0, 0.0, 0.0), 0.2);
//    }
    else if (cmd[0] == "torque")
    {
        if(platform_state_processing)
        {
            if (cmd[1] == "on")
                super_manipulator->enableAllJointActuator();
            else if (cmd[1] == "off")
                super_manipulator->disableAllJointActuator();
        }
    }
        ////////// hand teaching tab
    else if (cmd[0] == "get")
    {
        if (cmd[1] == "clear")  // motion clear
        {
            processing_motion_state = false;
            motion_way_point_buf.clear();
            hand_motion_cnt = 0;
        }
        else if (cmd[1] == "pose")  // save pose
        {
            MotionWaypoint read_value;
            JointWayPoint present_states = super_manipulator->getAllActiveJointValue();
            for(uint32_t i = 0; i < present_states.size(); i ++)
                read_value.angle.push_back(present_states.at(i).position);
            read_value.path_time = 2.0; // FIX TIME PARAM
            //read_value.gripper_value = open_manipulator->getToolValue("gripper").position;
            motion_way_point_buf.push_back(read_value);
            hand_motion_cnt = 0;
        }
        else if (cmd[1] == "on")  // save gripper on
        {
            super_manipulator->makeToolTrajectory("gripper", -0.009);
        }
        else if (cmd[1] == "off")  // save gripper off
        {
            super_manipulator->makeToolTrajectory("gripper", 0.009);
        }
    }
    else if (cmd[0] == "hand")
    {
        if (cmd[1] == "once") // play motion (once)
        {
            processing_motion_state = true;
        }
        else if (cmd[1] == "repeat") // play motion (repeat)
        {
            hand_motion_repeat_state = true;
        }
        else if (cmd[1] == "stop") // play motion (stop)
        {
            hand_motion_repeat_state = false;
            processing_motion_state = false;
            hand_motion_cnt = 0;
        }
    }
        ////////// motion tab
    else if (cmd[0] == "motion")
    {
        if (cmd[1] == "1")
        {
            TaskWayPoint draw_line_arg;
            draw_line_arg.kinematic.position(0) = 0.02;
            draw_line_arg.kinematic.position(1) = 0.02;
            draw_line_arg.kinematic.position(2) = -0.02;
            void *p_draw_line_arg = &draw_line_arg;
            super_manipulator->makeCustomTrajectory(CUSTOM_TRAJECTORY_LINE, "joint4", p_draw_line_arg, 1.0);
        }
        else if (cmd[1] == "2")
        {
            double draw_circle_arg[3];
            draw_circle_arg[0] = 0.03; // radius (m)
            draw_circle_arg[1] = 2;    // revolution
            draw_circle_arg[2] = 0.0;  // start angle position (rad)
            void* p_draw_circle_arg = &draw_circle_arg;
            super_manipulator->makeCustomTrajectory(CUSTOM_TRAJECTORY_CIRCLE, "gripper", p_draw_circle_arg, 4.0);
        }
    }
}

void playProcessingMotion(superManipulator *super_manipulator)
{
    if(!super_manipulator->getMovingState() && processing_motion_state)
    {
        if(motion_way_point_buf.size() == 0)
            return;

        //open_manipulator->makeToolTrajectory("gripper", motion_way_point_buf.at(hand_motion_cnt).gripper_value);
        super_manipulator->makeJointTrajectory(motion_way_point_buf.at(hand_motion_cnt).angle, motion_way_point_buf.at(hand_motion_cnt).path_time);
        hand_motion_cnt ++;
        if(hand_motion_cnt >= motion_way_point_buf.size())
        {
            hand_motion_cnt = 0;
            if(!hand_motion_repeat_state)
                processing_motion_state = false;
        }
    }
}

void getData(uint32_t wait_time)
{
    static uint8_t state = 0;
    static uint32_t tick = 0;

    bool rc100_state = false;
    bool processing_state = false;

    uint16_t get_rc100_data = 0;
    std::string get_processing_data = "";

//    if (availableProcessing())
//    {
//        get_processing_data = readProcessingData();
//        processing_state = true;
//    }


    get_processing_data = "motion,1";
    processing_state = true;

    switch (state)
    {
        case 0:

            if (processing_state)
            {
                fromProcessing(&super_manipulator, get_processing_data);
                tick = getCurrentTime();
                state = 1;
            }
            break;

        case 1:
            if ((getCurrentTime() - tick) >= wait_time)
            {
                state = 0;
            }
            break;

        default:
            state = 0;
            break;
    }
}