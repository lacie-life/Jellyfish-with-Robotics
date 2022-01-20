//
// Created by lacie on 29/08/2019.
//

#ifndef MANIPULATOR_VER1_CUSTOM_TRAJECTORY_H
#define MANIPULATOR_VER1_CUSTOM_TRAJECTORY_H

#include "../../include/manipulator_base/manipulator.h"

using namespace manipulator_base;
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
    class Line : public manipulator_base::CustomTaskTrajectory
    {
    private:
        TaskWayPoint start_pose_;
        TaskWayPoint goal_pose_;

        double acc_dec_time_;
        double move_time_;
        std::vector<double> vel_max_;

    public:
        Line() {}
        virtual ~Line() {}

        void initLine(double move_time, TaskWayPoint start, TaskWayPoint delta);
        TaskWayPoint drawLine(double time_var);

        virtual void setOption(const void *arg);
        virtual void makeTaskTrajectory(double move_time, TaskWayPoint start, const void *arg);
        virtual TaskWayPoint getTaskWaypoint(double tick);
    };


/*****************************************************************************
** Circle
*****************************************************************************/
    class Circle : public manipulator_base::CustomTaskTrajectory
    {
    private:
        manipulator_base::MinimumJerk path_generator_;
        VectorXd coefficient_;

        TaskWayPoint start_pose_;
        TaskWayPoint goal_pose_;

        double radius_;
        double start_angular_position_;
        double revolution_;

    public:
        Circle() {}
        virtual ~Circle() {}

        void initCircle(double move_time, TaskWayPoint start, double radius, double revolution, double start_angular_position);
        TaskWayPoint drawCircle(double time_var);

        virtual void setOption(const void *arg);
        virtual void makeTaskTrajectory(double move_time, TaskWayPoint start, const void *arg);
        virtual TaskWayPoint getTaskWaypoint(double tick);
    };


/*****************************************************************************
** Rhombus
*****************************************************************************/
    class Rhombus : public manipulator_base::CustomTaskTrajectory
    {
    private:
        manipulator_base::MinimumJerk path_generator_;
        VectorXd coefficient_;

        TaskWayPoint start_pose_;
        TaskWayPoint goal_pose_;

        double radius_;
        double start_angular_position_;
        double revolution_;

    public:
        Rhombus() {}
        virtual ~Rhombus() {}

        void initRhombus(double move_time, TaskWayPoint start, double radius, double revolution, double start_angular_position);
        TaskWayPoint drawRhombus(double time_var);

        virtual void setOption(const void *arg);
        virtual void makeTaskTrajectory(double move_time, TaskWayPoint start, const void *arg);
        virtual TaskWayPoint getTaskWaypoint(double tick);
    };


/*****************************************************************************
** Heart
*****************************************************************************/
    class Heart : public manipulator_base::CustomTaskTrajectory
    {
    private:
        manipulator_base::MinimumJerk path_generator_;
        VectorXd coefficient_;

        TaskWayPoint start_pose_;
        TaskWayPoint goal_pose_;

        double radius_;
        double start_angular_position_;
        double revolution_;

    public:
        Heart() {}
        virtual ~Heart() {}

        void initHeart(double move_time, TaskWayPoint start, double radius, double revolution, double start_angular_position);
        TaskWayPoint drawHeart(double tick);

        virtual void setOption(const void *arg);
        virtual void makeTaskTrajectory(double move_time, TaskWayPoint start, const void *arg);
        virtual TaskWayPoint getTaskWaypoint(double tick);
    };


} // namespace CUSTOM_TRAJECTORY


#endif //MANIPULATOR_VER1_CUSTOM_TRAJECTORY_H
