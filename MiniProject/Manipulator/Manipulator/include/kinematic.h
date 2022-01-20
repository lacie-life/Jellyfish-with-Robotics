//
// Created by lacie on 17/07/2019.
//

#ifndef MANIPULATOR_KINEMATIC_H
#define MANIPULATOR_KINEMATIC_H



#if defined(__OPENCR__)
#include <RobotisManipulator.h>
#else
#include "robotis_manipulator.h"
#endif

//#define KINEMATICS_DEBUG

using namespace Eigen;
using namespace robotis_manipulator;

namespace kinematics
{

/*****************************************************************************
** Kinematics Solver Using Chain Rule and Jacobian
*****************************************************************************/
    class SolverUsingCRAndJacobian : public robotis_manipulator::Kinematics
    {
    private:
        void forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name);
        bool inverseSolverUsingJacobian(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue>* goal_joint_value);

    public:
        SolverUsingCRAndJacobian(){}
        virtual ~SolverUsingCRAndJacobian(){}

        virtual void setOption(const void *arg);
        virtual MatrixXd jacobian(Manipulator *manipulator, Name tool_name);
        virtual void solveForwardKinematics(Manipulator *manipulator);
        virtual bool solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue>* goal_joint_value);
    };


/*****************************************************************************
** Kinematics Solver Using Chain Rule and Singularity Robust Jacobian
*****************************************************************************/
    class SolverUsingCRAndSRJacobian : public robotis_manipulator::Kinematics
    {
    private:
        void forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name);
        bool inverseSolverUsingSRJacobian(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue>* goal_joint_value);

    public:
        SolverUsingCRAndSRJacobian(){}
        virtual ~SolverUsingCRAndSRJacobian(){}

        virtual void setOption(const void *arg);
        virtual MatrixXd jacobian(Manipulator *manipulator, Name tool_name);
        virtual void solveForwardKinematics(Manipulator *manipulator);
        virtual bool solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue>* goal_joint_value);
    };


/*****************************************************************************
** Kinematics Solver Using Chain Rule and Singularity Robust Position Only Jacobian
*****************************************************************************/
    class SolverUsingCRAndSRPositionOnlyJacobian : public robotis_manipulator::Kinematics
    {
    private:
        void forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name);
        bool inverseSolverUsingPositionOnlySRJacobian(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue>* goal_joint_value);

    public:
        SolverUsingCRAndSRPositionOnlyJacobian(){}
        virtual ~SolverUsingCRAndSRPositionOnlyJacobian(){}

        virtual void setOption(const void *arg);
        virtual MatrixXd jacobian(Manipulator *manipulator, Name tool_name);
        virtual void solveForwardKinematics(Manipulator *manipulator);
        virtual bool solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue>* goal_joint_value);
    };


/*****************************************************************************
** Kinematics Solver Customized for OpenManipulator Chain
*****************************************************************************/
    class SolverCustomizedforOMChain : public robotis_manipulator::Kinematics
    {
    private:
        void forwardSolverUsingChainRule(Manipulator *manipulator, Name component_name);
        bool chainCustomInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue>* goal_joint_value);

    public:
        SolverCustomizedforOMChain(){}
        virtual ~SolverCustomizedforOMChain(){}

        virtual void setOption(const void *arg);
        virtual MatrixXd jacobian(Manipulator *manipulator, Name tool_name);
        virtual void solveForwardKinematics(Manipulator *manipulator);
        virtual bool solveInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose, std::vector<JointValue>* goal_joint_value);
    };

} // namespace KINEMATICS


#endif // KINEMATICS_H_
