//
// Created by lacie on 29/08/2019.
//

#ifndef MANIPULATOR_VER1_MANIPULATOR_MANAGER_H
#define MANIPULATOR_VER1_MANIPULATOR_MANAGER_H

#include <eigen3/Eigen/Eigen>
#include "manipulator_common.h"

namespace manipulator_base
{
    class Kinematics
    {
    public:
        Kinematics() {}
        virtual ~Kinematics() {}

        virtual void setOption(const void *arg) = 0;
        virtual Eigen::MatrixXd jacobian(Manipulator *manipulator, Name tool_name) = 0;
        virtual void solveForwardKinematics(Manipulator *manipulator) = 0;
        virtual bool solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue>* goal_joint_position) = 0;
    };

    class JointActuator
    {
    public:
        bool enabled_state_;

        JointActuator() : enabled_state_(false) {}
        virtual ~JointActuator() {}

        virtual void init(std::vector<uint8_t> actuator_id, const void *arg) = 0;
        virtual void setMode(std::vector<uint8_t> actuator_id, const void *arg) = 0;
        virtual std::vector<uint8_t> getId() = 0;

        virtual void enable() = 0;
        virtual void disable() = 0;

        virtual bool sendJointActuatorValue(std::vector<uint8_t> actuator_id, std::vector<ActuatorValue> value_vector) = 0;
        virtual std::vector<ActuatorValue> receiveJointActuatorValue(std::vector<uint8_t> actuator_id) = 0;

        bool findId(uint8_t actuator_id);
        bool getEnabledState();
    };

    class ToolActuator
    {
    public:
        bool enabled_state_;

        ToolActuator():enabled_state_(false){}
        virtual ~ToolActuator() {}

        virtual void init(uint8_t actuator_id, const void *arg) = 0;
        virtual void setMode(const void *arg) = 0;
        virtual uint8_t getId() = 0;

        virtual void enable() = 0;
        virtual void disable() = 0;

        virtual bool sendToolActuatorValue(ActuatorValue value) = 0;
        virtual ActuatorValue receiveToolActuatorValue() = 0;

        bool findId(uint8_t actuator_id);
        bool getEnabledState();
    };


    class CustomJointTrajectory
    {
    public:
        CustomJointTrajectory() {}
        virtual ~CustomJointTrajectory() {}

        virtual void makeJointTrajectory(double move_time, JointWayPoint start, const void *arg) = 0;
        virtual void setOption(const void *arg) = 0;
        virtual JointWayPoint getJointWayPoint(double tick) = 0;
    };

    class CustomTaskTrajectory
    {
    public:
        CustomTaskTrajectory() {}
        virtual ~CustomTaskTrajectory() {}

        virtual void makeTaskTrajectory(double move_time, TaskWayPoint start, const void *arg) = 0;
        virtual void setOption(const void *arg) = 0;
        virtual TaskWayPoint getTaskWaypoint(double tick) = 0;
    };
}
#endif //MANIPULATOR_VER1_MANIPULATOR_MANAGER_H
