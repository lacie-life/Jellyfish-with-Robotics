//
// Created by lacie on 29/08/2019.
//

#ifndef MANIPULATOR_VER1_MANIPULATOR_TRAJECTORY_GENERATOR_H
#define MANIPULATOR_VER1_MANIPULATOR_TRAJECTORY_GENERATOR_H

#include <math.h>
#include <vector>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/QR>

#include "manipulator_manager.h"

#define PI 3.141592

namespace manipulator_base
{
    class MinimumJerk
    {
    private:
        Eigen::VectorXd coefficient_;

    public:
         MinimumJerk();
         virtual ~MinimumJerk();

         void calcCoefficient(Point start,
                              Point goal,
                              double move_time);

         Eigen::VectorXd getCoefficient();

    };

    class JointTrajectory
    {
    private:
        uint8_t coefficient_size_;
        MinimumJerk minimum_jerk_trajectory_generator_;
        Eigen::MatrixXd minimum_jerk_coefficient_;

    public:
        JointTrajectory();
        virtual ~JointTrajectory();

        void makeJointTrajectory(double move_time,
                JointWayPoint start,
                JointWayPoint goal);

        Eigen::MatrixXd getMinimumJerkCoefficient();
        JointWayPoint getJointWayPoint(double tick);
    };

    class TaskTrajectory
    {
    private:
        uint8_t coefficient_size_;
        MinimumJerk minimum_jerk_trajectory_generator_;
        Eigen::MatrixXd minimum_jerk_coefficient_;

    public:
        TaskTrajectory();
        virtual ~TaskTrajectory();

        void makeTaskTrajectory(double move_time,
                                TaskWayPoint start,
                                TaskWayPoint goal);
        Eigen::MatrixXd getMinimumJerkCoefficient();
        TaskWayPoint getTaskWaypoint(double tick);
    };

    /*************************************************************
     *  Trajectory Class
     ************************************************************/
     class Trajectory
     {
     private:
         TrajectoryType trajectory_type_;
         Time trajectory_time_;
         Manipulator manipulator_;

         JointTrajectory joint_;
         TaskTrajectory task_;
         std::map<Name, CustomJointTrajectory *> cus_joint_;
         std::map<Name, CustomTaskTrajectory *> cus_task_;

         Name present_custom_trajectory_name_;
         Name present_control_tool_name_;

     public:
         Trajectory() {}
         ~Trajectory() {}

         // Time
         void setMoveTime(double move_time);
         void setPresentTime(double present_time);
         void setStartTimeToPresentTime();
         void setStartTime(double start_time);
         double getMoveTime();
         double getTickTime();

         // Manipulator
         void setManipulator(Manipulator manipulator);
         Manipulator* getManipulator();

         // Get Trajectory
         JointTrajectory getJointTrajectory();
         TaskTrajectory getTaskTrajectory();
         CustomJointTrajectory* getCustomJointTrajectory(Name name);
         CustomTaskTrajectory* getCustomTaskTrajectory(Name name);

         // Custom Trajectory Setting
         void addCustomTrajectory(Name trajectory_name, CustomJointTrajectory *custom_trajectory);
         void addCustomTrajectory(Name trajectory_name, CustomTaskTrajectory *custom_trajectory);
         void setCustomTrajectoryOption(Name trajectory_name, const void* arg);
         void setPresentControlToolName(Name present_control_tool_name);
         Name getPresentCustomTrajectoryName();
         Name getPresentControlToolName();

         // First Waypoint
         void initTrajectoryWayPoint(Manipulator actual_manipulator, Kinematics *kinematics);

         // Present Waypoint
         void updatePresentWaypoint(Kinematics* kinematics); //forward kinematics,dynamics
         void setPresentJointWaypoint(JointWayPoint joint_value_vector);
         void setPresentTaskWaypoint(Name tool_name, TaskWayPoint tool_position_value_vector);
         JointWayPoint getPresentJointWaypoint();
         TaskWayPoint getPresentTaskWaypoint(Name tool_name);

         JointWayPoint removeWaypointDynamicData(JointWayPoint value);
         TaskWayPoint removeWaypointDynamicData(TaskWayPoint value);

         // Trajectory
         void setTrajectoryType(TrajectoryType trajectory_type);
         bool checkTrajectoryType(TrajectoryType trajectory_type);
         void makeJointTrajectory(JointWayPoint start_way_point, JointWayPoint goal_way_point);
         void makeTaskTrajectory(TaskWayPoint start_way_point, TaskWayPoint goal_way_point);
         void makeCustomTrajectory(Name trajectory_name, JointWayPoint start_way_point, const void *arg);
         void makeCustomTrajectory(Name trajectory_name, TaskWayPoint start_way_point, const void *arg);

         // Tool
         void setToolGoalPosition(Name tool_name, double tool_goal_position);
         void setToolGoalValue(Name tool_name, JointValue tool_goal_value);
         double getToolGoalPosition(Name tool_name);
         JointValue getToolGoalValue(Name tool_name);
     };
}

#endif //MANIPULATOR_VER1_MANIPULATOR_TRAJECTORY_GENERATOR_H
