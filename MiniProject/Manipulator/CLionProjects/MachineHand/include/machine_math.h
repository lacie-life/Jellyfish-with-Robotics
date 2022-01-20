//
// Created by lacie on 09/09/2019.
//

#ifndef MACHINEHAND_MACHINE_MATH_H
#define MACHINEHAND_MACHINE_MATH_H

#include <unistd.h>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/LU>
#include <math.h>

#define DEG2RAD 0.01745329252 //(M_PI / 180.0)
#define RAD2DEG 57.2957795131 //(180.0 / M_PI)

namespace MachineHand
{
    namespace math
    {
        /******************************************
         * Make a vector matrix
         ************************************************/
        Eigen::Vector3d vector3(double v1, double v2, double v3);
        Eigen::Matrix3d matrix3(double m11, double m12, double m13,
                                double m21, double m22, double m23,
                                double m31, double m32, double m33);
        Eigen::Matrix3d inertiaMatrix(double ixx, double ixy, double ixz , double iyy , double iyz, double izz);

        /*********************************************
         * Convert
         **********************************************/

        // Translation Vector
        Eigen::Vector3d convertXYZToVector(double x, double y, double z);

        // Rotation
        Eigen::Matrix3d convertRollAngleToRotationMatrix(double angle);
        Eigen::Matrix3d convertPitchAngleToRotationMatrix(double angle);
        Eigen::Matrix3d convertYawAngleToRotationMatrix(double angle);
        Eigen::Vector3d convertRotationMatrixToRPYVector(const Eigen::Matrix3d& rotation_matrix);
        Eigen::Matrix3d convertRPYToRotationMatrix(double roll, double pitch, double yaw);
        Eigen::Quaterniond convertRPYToQuaternion(double roll, double pitch, double yaw);
        Eigen::Quaterniond convertRotationMatrixToQuaternion(const Eigen::Matrix3d& rotation_matrix);
        Eigen::Vector3d convertQuaternionToRPYVector(const Eigen::Quaterniond& quaternion);
        Eigen::Matrix3d convertQuaternionToRotationMatrix(const Eigen::Quaterniond& quaternion);
        Eigen::Vector3d convertRotationMatrixToOmega(const Eigen::Matrix3d& rotation_matrix);

        // Transformation Matrix
        Eigen::Matrix4d convertXYZRPYToTransformationMatrix(double x, double y, double z , double roll, double pitch, double yaw);
        Eigen::Matrix4d convertXYZToTransformationMatrix(double x, double y, double z);
        Eigen::Matrix4d convertRPYToTransformationMatrix(double roll, double pitch, double yaw);

        // Dynamic Value
        Eigen::Vector3d convertOmegaToRPYVelocity(Eigen::Vector3d rpy_vector, Eigen::Vector3d omega);
        Eigen::Vector3d convertRPYVelocityToOmega(Eigen::Vector3d rpy_vector, Eigen::Vector3d rpy_velocity);
        Eigen::Vector3d convertOmegaDotToRPYAcceleration(Eigen::Vector3d rpy_vector, Eigen::Vector3d rpy_velocity, Eigen::Vector3d omega_dot);
        Eigen::Vector3d convertRPYAccelerationToOmegaDot(Eigen::Vector3d rpy_vector, Eigen::Vector3d rpy_velocity, Eigen::Vector3d rpy_acceleration);

        /*************************************************************
         *  Math
         *************************************************************/
        double sign(double value);
        // nghich dao ma tran
        Eigen::Matrix4d inverseTransformationMatrix(const Eigen::MatrixXd& transformation_matrix);
        Eigen::Vector3d matrixLogarithm(Eigen::Matrix3d rotation_matrix);
        // ma tran doi xung nghieng
        Eigen::Matrix3d skewSymmetricMatrix(Eigen::Vector3d v);
        // ma tran xoay vong
        Eigen::Matrix3d rodriguesRotationMatrix(Eigen::Vector3d axis, double angle);

        Eigen::Vector3d positionDifference(Eigen::Vector3d desired_position, Eigen::Vector3d present_position);
        Eigen::Vector3d orientationDifference(Eigen::Matrix3d desired_orientation, Eigen::Matrix3d present_orientation);
        Eigen::VectorXd poseDifference(Eigen::Vector3d desired_position, Eigen::Vector3d present_position,
                                       Eigen::Matrix3d desired_orientation, Eigen::Matrix3d present_orientation);
    }
}

#endif //MACHINEHAND_MACHINE_MATH_H
