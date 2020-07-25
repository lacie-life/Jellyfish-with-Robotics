//
// Created by lacie on 17/07/2019.
//

#ifndef MANIPULATOR_CUSTOM_TRAJECTORY_H
#define MANIPULATOR_CUSTOM_TRAJECTORY_H



#if defined(__OPENCR__)
#include <RobotisManipulator.h>
#else
#include "robotis_manipulator.h"
#endif

using namespace robotis_manipulator;
using namespace Eigen;

namespace custom_trajectory
{

    enum AXIS{
        X_AXIS,
        Y_AXIS,
        Z_AXIS,
    };


/*****************************************************************************
** Line
*****************************************************************************/
    class Line : public robotis_manipulator::CustomTaskTrajectory
    {
    private:
        TaskWaypoint start_pose_;
        TaskWaypoint goal_pose_;

        double acc_dec_time_;
        double move_time_;
        std::vector<double> vel_max_;

    public:
        Line() {}
        virtual ~Line() {}

        void initLine(double move_time, TaskWaypoint start, TaskWaypoint delta);
        TaskWaypoint drawLine(double time_var);

        virtual void setOption(const void *arg);
        virtual void makeTaskTrajectory(double move_time, TaskWaypoint start, const void *arg);
        virtual TaskWaypoint getTaskWaypoint(double tick);
    };


/*****************************************************************************
** Circle
*****************************************************************************/
    class Circle : public robotis_manipulator::CustomTaskTrajectory
    {
    private:
        robotis_manipulator::MinimumJerk path_generator_;
        VectorXd coefficient_;

        TaskWaypoint start_pose_;
        TaskWaypoint goal_pose_;

        double radius_;
        double start_angular_position_;
        double revolution_;

    public:
        Circle() {}
        virtual ~Circle() {}

        void initCircle(double move_time, TaskWaypoint start, double radius, double revolution, double start_angular_position);
        TaskWaypoint drawCircle(double time_var);

        virtual void setOption(const void *arg);
        virtual void makeTaskTrajectory(double move_time, TaskWaypoint start, const void *arg);
        virtual TaskWaypoint getTaskWaypoint(double tick);
    };


/*****************************************************************************
** Rhombus
*****************************************************************************/
    class Rhombus : public robotis_manipulator::CustomTaskTrajectory
    {
    private:
        robotis_manipulator::MinimumJerk path_generator_;
        VectorXd coefficient_;

        TaskWaypoint start_pose_;
        TaskWaypoint goal_pose_;

        double radius_;
        double start_angular_position_;
        double revolution_;

    public:
        Rhombus() {}
        virtual ~Rhombus() {}

        void initRhombus(double move_time, TaskWaypoint start, double radius, double revolution, double start_angular_position);
        TaskWaypoint drawRhombus(double time_var);

        virtual void setOption(const void *arg);
        virtual void makeTaskTrajectory(double move_time, TaskWaypoint start, const void *arg);
        virtual TaskWaypoint getTaskWaypoint(double tick);
    };


/*****************************************************************************
** Heart
*****************************************************************************/
    class Heart : public robotis_manipulator::CustomTaskTrajectory
    {
    private:
        robotis_manipulator::MinimumJerk path_generator_;
        VectorXd coefficient_;

        TaskWaypoint start_pose_;
        TaskWaypoint goal_pose_;

        double radius_;
        double start_angular_position_;
        double revolution_;

    public:
        Heart() {}
        virtual ~Heart() {}

        void initHeart(double move_time, TaskWaypoint start, double radius, double revolution, double start_angular_position);
        TaskWaypoint drawHeart(double tick);

        virtual void setOption(const void *arg);
        virtual void makeTaskTrajectory(double move_time, TaskWaypoint start, const void *arg);
        virtual TaskWaypoint getTaskWaypoint(double tick);
    };


} // namespace CUSTOM_TRAJECTORY
#endif // CUSTOM_TRAJECTORY_H_




