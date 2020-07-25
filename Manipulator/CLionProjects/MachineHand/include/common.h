//
// Created by lacie on 09/09/2019.
//

#ifndef MACHINEHAND_COMMON_H
#define MACHINEHAND_COMMON_H

#include <unistd.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/LU>
#include <math.h>
#include <vector>
#include <map>

#include "machine_math.h"

typedef std::string STRING;

namespace MahineHand
{
    typedef STRING Name;

    typedef struct _KinematicPose
    {
        Eigen::Vector3d position;
        Eigen::Matrix3d orientation;
    } KinematicPose;

    typedef struct _Limit
    {
        double maximum;
        double minimum;
    } Limit;

    typedef struct _Point
    {
        double position;
    } Point, ActuatorValue, JointValue, ToolValue;

    typedef std::vector<JointValue> JointWayPoint;

    typedef struct _TaskWayPoint
    {
        KinematicPose kinematic;
    } TaskWayPoint, Pose;

    typedef struct _ChainingName
    {
        Name parent;
        std::vector<Name> child;
    } ChainingName;

    typedef struct _Relative
    {
        KinematicPose pose_from_parent;
    } Relative;

    typedef struct _JointConstant
    {
        int8_t id;
        Eigen::Vector3d axis;
        double coefficient;       // joint angle over actuator angle
        Limit position_limit;
    } JointConstant;

    typedef struct _World
    {
        Name name;
        Name child;
        Pose pose;
    } World;

    typedef struct _Component
    {
        //constant
        ChainingName name;
        Relative relative;
        JointConstant joint_constant;

        //variable
        Pose pose_from_world;
        JointValue joint_value;

        //Actuator
        Name actuator_name;
    } Component;

    class Manipulator
    {
    private:
        int8_t dof_;
        World world_;
        std::map<Name, Component> component_;
    public:
        Manipulator();
        ~Manipulator() {}

        /*********************************************************************
         *  Add Function
         *********************************************************************/
        void addWorld(Name world_name,
                      Name child_name,
                      Eigen::Vector3d world_position = Eigen::Vector3d::Zero(),
                      Eigen::Matrix3d world_orientation = Eigen::Matrix3d::Identity());

        void addJoint(Name my_name,
                      Name parent_name,
                      Name child_name,
                      Eigen::Vector3d relative_position,
                      Eigen::Matrix3d relative_orientation,
                      Eigen::Vector3d axis_of_rotation = Eigen::Vector3d::Zero(),
                      int8_t joint_actuator_id = -1,
                      double max_position_limit = M_PI,
                      double min_position_limit = -M_PI,
                      double coefficient = 1.0);

        void addTool(Name my_name,
                     Name parent_name,
                     Eigen::Vector3d relative_position,
                     Eigen::Matrix3d relative_orientation,
                     int8_t tool_id = -1,
                     double max_position_limit = M_PI,
                     double min_position_limit = -M_PI,
                     double coefficient = 1.0);

        void addComponentChild(Name my_name, Name child_name);
        void printManipulatorSetting();

        /*****************************************************************************
          ** Set Function
         *****************************************************************************/
        void setWorldPose(Pose world_pose);
        void setWorldKinematicPose(KinematicPose world_kinematic_pose);
        void setWorldPosition(Eigen::Vector3d world_position);
        void setWorldOrientation(Eigen::Matrix3d world_orientation);
        void setComponent(Name component_name, Component component);
        void setComponentActuatorName(Name component_name, Name actuator_name);
        void setComponentPoseFromWorld(Name component_name, Pose pose_to_world);
        void setComponentKinematicPoseFromWorld(Name component_name, KinematicPose pose_to_world);
        void setComponentPositionFromWorld(Name component_name, Eigen::Vector3d position_to_world);
        void setComponentOrientationFromWorld(Name component_name, Eigen::Matrix3d orientation_to_wolrd);

        void setJointPosition(Name name, double position);
        void setJointValue(Name name, JointValue joint_value);

        void setAllActiveJointPosition(std::vector<double> joint_position_vector);
        void setAllActiveJointValue(std::vector<JointValue> joint_value_vector);
        void setAllJointPosition(std::vector<double> joint_position_vector);
        void setAllJointValue(std::vector<JointValue> joint_value_vector);
        void setAllToolPosition(std::vector<double> tool_position_vector);
        void setAllToolValue(std::vector<JointValue> tool_value_vector);


        /*****************************************************************************
         ** Get Function
         *****************************************************************************/
        int8_t getDOF();
        Name getWorldName();
        Name getWorldChildName();
        Pose getWorldPose();
        KinematicPose getWorldKinematicPose();
        Eigen::Vector3d getWorldPosition();
        Eigen::Matrix3d getWorldOrientation();
        int8_t getComponentSize();
        std::map<Name, Component> getAllComponent();std::map<Name, Component>::iterator getIteratorBegin();
        std::map<Name, Component>::iterator getIteratorEnd();
        Component getComponent(Name component_name);
        Name getComponentActuatorName(Name component_name);
        Name getComponentParentName(Name component_name);
        std::vector<Name> getComponentChildName(Name component_name);
        Pose getComponentPoseFromWorld(Name component_name);
        KinematicPose getComponentKinematicPoseFromWorld(Name component_name);
        Eigen::Vector3d getComponentPositionFromWorld(Name component_name);
        Eigen::Matrix3d getComponentOrientationFromWorld(Name component_name);
        KinematicPose getComponentRelativePoseFromParent(Name component_name);
        Eigen::Vector3d getComponentRelativePositionFromParent(Name component_name);
        Eigen::Matrix3d getComponentRelativeOrientationFromParent(Name component_name);

        int8_t getId(Name component_name);
        double getCoefficient(Name component_name);
        Eigen::Vector3d getAxis(Name component_name);
        double getJointPosition(Name component_name);
        JointValue getJointValue(Name component_name);


        std::vector<double> getAllJointPosition();
        std::vector<JointValue> getAllJointValue();
        std::vector<double> getAllActiveJointPosition();
        std::vector<JointValue> getAllActiveJointValue();
        std::vector<double> getAllToolPosition();
        std::vector<JointValue> getAllToolValue();

        std::vector<uint8_t> getAllJointID();
        std::vector<uint8_t> getAllActiveJointID();
        std::vector<Name> getAllToolComponentName();
        std::vector<Name> getAllActiveJointComponentName();


        /*****************************************************************************
         ** Check Function
         *****************************************************************************/
        bool checkJointLimit(Name Component_name, double value);

        /*****************************************************************************
         ** Find Function
        *****************************************************************************/

        Name findComponentNameUsingId(int8_t id);
    };
}


#endif //MACHINEHAND_COMMON_H
