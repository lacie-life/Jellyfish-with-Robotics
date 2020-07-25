//
// Created by lacie on 29/08/2019.
//

#ifndef MANIPULATOR_VER1_KINEMATICS_H
#define MANIPULATOR_VER1_KINEMATICS_H

#include "../../include/manipulator_base/manipulator.h"
#include "../../include/manipulator_base/manipulator_math.h"

//#define KINEMATICS_DEBUG

using namespace Eigen;
using namespace manipulator_base;

namespace kinematics
{

/*****************************************************************************
** Kinematics Solver Using Chain Rule and Jacobian
*****************************************************************************/
    class SolverUsingCRAndJacobian : public manipulator_base::Kinematics
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
    class SolverUsingCRAndSRJacobian : public manipulator_base::Kinematics
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
    class SolverUsingCRAndSRPositionOnlyJacobian : public manipulator_base::Kinematics
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
    class SolverCustomizedforOMChain : public manipulator_base::Kinematics
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



#endif //MANIPULATOR_VER1_KINEMATICS_H
