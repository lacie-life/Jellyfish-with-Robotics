#include <iostream>
#include <vector>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/LU>
#include <math.h>

struct Node
{
    std::string my_name;
    std::string parent_name;
    std::string child_name;
    int8_t id;
    Eigen::Vector3d pose;
    Eigen::Matrix3d orientation;
    Eigen::Vector3d axis;

    double maximum_limit;
    double minimum_limit;

    double joint_position;

    Eigen::Vector3d pose_form_parent;
    Eigen::Matrix3d orientation_from_parent;
    Eigen::Vector3d pose_from_world;
    Eigen::Matrix3d orientation_from_world;

    Node* parent;
    Node* children;
};

struct World
{
    std::string name;
    std::string child;
    Eigen::Vector3d pose;
    Eigen::Matrix3d orientation;
};

struct Manipulator_
{
    int8_t dof;
    World world;
    std::vector <Node> component;
} ;

Eigen::Vector3d vector3(double v1, double v2, double v3)
{
    Eigen::Vector3d temp;
    temp << v1, v2, v3;
    return temp;
}

Eigen::Matrix3d matrix3(double m11, double m12, double m13,
                                                double m21, double m22, double m23,
                                                double m31, double m32, double m33)
{
    Eigen::Matrix3d temp;
    temp << m11, m12, m13, m21, m22, m23, m31, m32, m33;
    return temp;
}

Eigen::Matrix3d inertiaMatrix(double ixx, double ixy, double ixz , double iyy , double iyz, double izz)
{
    Eigen::Matrix3d inertia;
    inertia <<
            ixx, ixy, ixz,
            ixy, iyy, iyz,
            ixz, iyz, izz;

    return inertia;
}

/**********************************************************************
 *  Convert
 ************************************************************************/

// Translation Vector
Eigen::Vector3d convertXYZToVector(double x, double y, double z)
{
    Eigen::Vector3d position;
    position << x, y, z;

    return position;
}

// Rotation
Eigen::Matrix3d convertRollAngleToRotationMatrix(double angle)
{
    Eigen::Matrix3d rotation(3,3);
    rotation <<
             1.0, 0.0, 0.0,
            0.0, cos(angle), -sin(angle),
            0.0, sin(angle), cos(angle);

    return rotation;
}

Eigen::Matrix3d convertPitchAngleToRotationMatrix(double angle)
{
    Eigen::Matrix3d rotation(3,3);
    rotation <<
             cos(angle), 0.0, sin(angle),
            0.0, 1.0, 0.0,
            -sin(angle), 0.0, cos(angle);

    return rotation;
}

Eigen::Matrix3d convertYawAngleToRotationMatrix(double angle)
{
    Eigen::Matrix3d rotation(3,3);
    rotation <<
             cos(angle), -sin(angle), 0.0,
            sin(angle), cos(angle), 0.0,
            0.0, 0.0, 1.0;

    return rotation;
}

Eigen::Vector3d convertRotationMatrixToRPYVector(const Eigen::Matrix3d& rotation)
{
    Eigen::Vector3d rpy;// = Eigen::MatrixXd::Zero(3,1);
    rpy.coeffRef(0,0) = atan2(rotation.coeff(2,1), rotation.coeff(2,2));
    rpy.coeffRef(1,0) = atan2(-rotation.coeff(2,0), sqrt(pow(rotation.coeff(2,1), 2) + pow(rotation.coeff(2,2),2)));
    rpy.coeffRef(2,0) = atan2 (rotation.coeff(1,0), rotation.coeff(0,0));

    return rpy;
}

Eigen::Matrix3d convertRPYToRotationMatrix(double roll, double pitch, double yaw)
{
    Eigen::Matrix3d rotation = convertYawAngleToRotationMatrix(yaw)*convertPitchAngleToRotationMatrix(pitch)*convertRollAngleToRotationMatrix(roll);

    return rotation;
}

Eigen::Quaterniond convertRPYToQuaternion(double roll, double pitch, double yaw)
{
    Eigen::Quaterniond quaternion;
    quaternion = convertRPYToRotationMatrix(roll,pitch,yaw);

    return quaternion;
}

Eigen::Quaterniond convertRotationMatrixToQuaternion(const Eigen::Matrix3d& rotation)
{
    Eigen::Quaterniond quaternion;
    quaternion = rotation;

    return quaternion;
}

Eigen::Vector3d convertQuaternionToRPYVector(const Eigen::Quaterniond& quaternion)
{
    Eigen::Vector3d rpy = convertRotationMatrixToRPYVector(quaternion.toRotationMatrix());

    return rpy;
}

Eigen::Matrix3d convertQuaternionToRotationMatrix(const Eigen::Quaterniond& quaternion)
{
    Eigen::Matrix3d rotation = quaternion.toRotationMatrix();

    return rotation;
}



// Transformation Matrix


Eigen::Matrix4d convertXYZToTransformationMatrix(double position_x, double position_y, double position_z)
{
    Eigen::Matrix4d mat_translation;

    mat_translation <<
                    1, 0, 0, position_x,
            0, 1, 0, position_y,
            0, 0, 1, position_z,
            0, 0, 0,          1;

    return mat_translation;
}

Eigen::Matrix4d convertRPYToTransformationMatrix(double roll, double pitch, double yaw )
{
    double sr = sin(roll), cr = cos(roll);
    double sp = sin(pitch), cp = cos(pitch);
    double sy = sin(yaw), cy = cos(yaw);

    Eigen::Matrix4d mat_roll;
    Eigen::Matrix4d mat_pitch;
    Eigen::Matrix4d mat_yaw;

    mat_roll <<
             1, 0, 0, 0,
            0, cr, -sr, 0,
            0, sr, cr, 0,
            0, 0, 0, 1;

    mat_pitch <<
              cp, 0, sp, 0,
            0, 1, 0, 0,
            -sp, 0, cp, 0,
            0, 0, 0, 1;

    mat_yaw <<
            cy, -sy, 0, 0,
            sy, cy, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    Eigen::Matrix4d mat_rpy = (mat_yaw*mat_pitch)*mat_roll;

    return mat_rpy;
}

Eigen::Matrix4d inverseTransformationMatrix(const Eigen::MatrixXd& transform)
{
    // If T is Transform Matrix A from B, the BOA is translation component coordi. B to coordi. A

    Eigen::Vector3d vec_boa;
    Eigen::Vector3d vec_x, vec_y, vec_z;
    Eigen::Matrix4d inv_t;

    vec_boa(0) = -transform(0,3);
    vec_boa(1) = -transform(1,3);
    vec_boa(2) = -transform(2,3);

    vec_x(0) = transform(0,0); vec_x(1) = transform(1,0); vec_x(2) = transform(2,0);
    vec_y(0) = transform(0,1); vec_y(1) = transform(1,1); vec_y(2) = transform(2,1);
    vec_z(0) = transform(0,2); vec_z(1) = transform(1,2); vec_z(2) = transform(2,2);

    inv_t <<
          vec_x(0), vec_x(1), vec_x(2), vec_boa.dot(vec_x),
            vec_y(0), vec_y(1), vec_y(2), vec_boa.dot(vec_y),
            vec_z(0), vec_z(1), vec_z(2), vec_boa.dot(vec_z),
            0, 0, 0, 1;

    return inv_t;
}

Eigen::Matrix4d convertXYZRPYToTransformationMatrix(double position_x, double position_y, double position_z , double roll , double pitch , double yaw)
{
    Eigen::Matrix4d transformation = convertRPYToTransformationMatrix(roll, pitch, yaw);
    transformation.coeffRef(0,3) = position_x;
    transformation.coeffRef(1,3) = position_y;
    transformation.coeffRef(2,3) = position_z;

    return transformation;
}
// Dynamic value
Eigen::Vector3d convertOmegaToRPYVelocity(Eigen::Vector3d rpy_vector, Eigen::Vector3d omega)
{
    Eigen::Matrix3d c_inverse;
    Eigen::Vector3d rpy_velocity;

    c_inverse << 1, sin(rpy_vector(0))*tan(rpy_vector(1)), cos(rpy_vector(0))*tan(rpy_vector(1)),
            0, cos(rpy_vector(0)),                    -sin(rpy_vector(0)),
            0, sin(rpy_vector(0))/cos(rpy_vector(1)), cos(rpy_vector(0))/cos(rpy_vector(1));

    rpy_velocity = c_inverse * omega;
    return rpy_velocity;
}

Eigen::Vector3d convertRPYVelocityToOmega(Eigen::Vector3d rpy_vector, Eigen::Vector3d rpy_velocity)
{
    Eigen::Matrix3d c;
    Eigen::Vector3d omega;

    c << 1, 0,                     -sin(rpy_vector(1)),
            0, cos(rpy_vector(0)),    sin(rpy_vector(0))*cos(rpy_vector(1)),
            0, -sin(rpy_vector(0)), cos(rpy_vector(0))*cos(rpy_vector(1));

    omega = c * rpy_velocity;
    return omega;
}

Eigen::Vector3d convertOmegaDotToRPYAcceleration(Eigen::Vector3d rpy_vector, Eigen::Vector3d rpy_velocity, Eigen::Vector3d omega_dot)
{
    Eigen::Vector3d c_dot;
    Eigen::Matrix3d c_inverse;
    Eigen::Vector3d rpy_acceleration;

    c_dot << -cos(rpy_vector[1]) * rpy_velocity[1] * rpy_velocity[2],
            -sin(rpy_vector[0]) * rpy_velocity[0] * rpy_velocity[1] - sin(rpy_vector[0]) * sin(rpy_vector[1]) * rpy_velocity[1] * rpy_velocity[2] + cos(rpy_vector[0]) * cos(rpy_vector[1]) * rpy_velocity[0] * rpy_velocity[2],
            -cos(rpy_vector[0]) * rpy_velocity[0] * rpy_velocity[1] - sin(rpy_vector[0]) * cos(rpy_vector[1]) * rpy_velocity[0] * rpy_velocity[2] - cos(rpy_vector[0]) * sin(rpy_vector[1]) * rpy_velocity[1] * rpy_velocity[2];

    c_inverse << 1, sin(rpy_vector(0))*tan(rpy_vector(1)), cos(rpy_vector(0))*tan(rpy_vector(1)),
            0, cos(rpy_vector(0)),                    -sin(rpy_vector(0)),
            0, sin(rpy_vector(0))/cos(rpy_vector(1)), cos(rpy_vector(0))/cos(rpy_vector(1));

    rpy_acceleration = c_inverse * (omega_dot - c_dot);
    return rpy_acceleration;
}

Eigen::Vector3d convertRPYAccelerationToOmegaDot(Eigen::Vector3d rpy_vector, Eigen::Vector3d rpy_velocity, Eigen::Vector3d rpy_acceleration)
{
    Eigen::Vector3d c_dot;
    Eigen::Matrix3d c;
    Eigen::Vector3d omega_dot;

    c_dot << -cos(rpy_vector[1]) * rpy_velocity[1] * rpy_velocity[2],
            -sin(rpy_vector[0]) * rpy_velocity[0] * rpy_velocity[1] - sin(rpy_vector[0]) * sin(rpy_vector[1]) * rpy_velocity[1] * rpy_velocity[2] + cos(rpy_vector[0]) * cos(rpy_vector[1]) * rpy_velocity[0] * rpy_velocity[2],
            -cos(rpy_vector[0]) * rpy_velocity[0] * rpy_velocity[1] - sin(rpy_vector[0]) * cos(rpy_vector[1]) * rpy_velocity[0] * rpy_velocity[2] - cos(rpy_vector[0]) * sin(rpy_vector[1]) * rpy_velocity[1] * rpy_velocity[2];

    c << 1, 0,                     -sin(rpy_vector(1)),
            0, cos(rpy_vector(0)),    sin(rpy_vector(0))*cos(rpy_vector(1)),
            0, -sin(rpy_vector(0)), cos(rpy_vector(0))*cos(rpy_vector(1));

    omega_dot = c_dot + c * rpy_acceleration;
    return omega_dot;
}

/********************************************************************************
 *  Math
 ******************************************************************************/
double sign(double value)
{
    if (value >= 0.0)
    {
        return 1.0;
    }
    else
    {
        return -1.0;
    }
}

Eigen::Matrix3d skewSymmetricMatrix(Eigen::Vector3d v)
{
    Eigen::Matrix3d skew_symmetric_matrix = Eigen::Matrix3d::Zero();
    skew_symmetric_matrix << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
    return skew_symmetric_matrix;
}

Eigen::Vector3d matrixLogarithm(Eigen::Matrix3d rotation_matrix)
{
    Eigen::Matrix3d R = rotation_matrix;
    Eigen::Vector3d l = Eigen::Vector3d::Zero();
    Eigen::Vector3d rotation_vector = Eigen::Vector3d::Zero();

    double theta = 0.0;
    // double diag = 0.0;
    bool diagonal_matrix = R.isDiagonal();

    l << R(2, 1) - R(1, 2),
            R(0, 2) - R(2, 0),
            R(1, 0) - R(0, 1);
    theta = atan2(l.norm(), R(0, 0) + R(1, 1) + R(2, 2) - 1);
    // diag = R.determinant();

    if (R.isIdentity())
    {
        rotation_vector.setZero(3);
        return rotation_vector;
    }

    if (diagonal_matrix == true)
    {
        rotation_vector << R(0, 0) + 1, R(1, 1) + 1, R(2, 2) + 1;
        rotation_vector = rotation_vector * M_PI_2;
    }
    else
    {
        rotation_vector = theta * (l / l.norm());
    }
    return rotation_vector;
}

Eigen::Matrix3d rodriguesRotationMatrix(Eigen::Vector3d axis, double angle)
{
    Eigen::Matrix3d skew_symmetric_matrix = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d Identity_matrix = Eigen::Matrix3d::Identity();

    skew_symmetric_matrix = skewSymmetricMatrix(axis);
    rotation_matrix = Identity_matrix +
                      skew_symmetric_matrix * sin(angle) +
                      skew_symmetric_matrix * skew_symmetric_matrix * (1 - cos(angle));
    return rotation_matrix;
}

Eigen::Vector3d positionDifference(Eigen::Vector3d desired_position, Eigen::Vector3d present_position)
{
    Eigen::Vector3d position_difference;
    position_difference = desired_position - present_position;

    return position_difference;
}

Eigen::Vector3d orientationDifference(Eigen::Matrix3d desired_orientation, Eigen::Matrix3d present_orientation)
{
    Eigen::Vector3d orientation_difference;
    orientation_difference = present_orientation * matrixLogarithm(present_orientation.transpose() * desired_orientation);

    return orientation_difference;
}

Eigen::VectorXd poseDifference(Eigen::Vector3d desired_position, Eigen::Vector3d present_position,
                                                       Eigen::Matrix3d desired_orientation, Eigen::Matrix3d present_orientation)
{
    Eigen::Vector3d position_difference;
    Eigen::Vector3d orientation_difference;
    Eigen::VectorXd pose_difference(6);

    position_difference = positionDifference(desired_position, present_position);
    orientation_difference = orientationDifference(desired_orientation, present_orientation);
    pose_difference << position_difference(0), position_difference(1), position_difference(2),
            orientation_difference(0), orientation_difference(1), orientation_difference(2);

    return pose_difference;
}

Eigen::Vector3d  convertRotationMatrixToOmega(const Eigen::Matrix3d& rotation_matrix)
{
    return matrixLogarithm(rotation_matrix);
}

#define X_AXIS vector3(1.0, 0.0, 0.0)
#define Y_AXIS vector3(0.0, 1.0, 0.0)
#define Z_AXIS vector3(0.0, 0.0, 1.0)

Manipulator_ Manipulator;

void addJoint(std::string my_name,
              std::string parent_name,
              std::string child_name,
              Eigen::Vector3d pose_from_parent,
              Eigen::Matrix3d orientation_from_parent,
              Eigen::Vector3d axis_of_rotation = Eigen::Vector3d::Zero(),
              int8_t joint_actuator_id = -1,
              double max_position_limit = M_PI,
              double min_position_limit = -M_PI)
{
    Node temp_component;
    if (joint_actuator_id != -1)
    {
        Manipulator.dof++;
    }

    temp_component.parent_name = parent_name;
    temp_component.child_name = child_name;
    temp_component.pose_form_parent = pose_from_parent;
    temp_component.orientation_from_parent = orientation_from_parent;
    temp_component.id = joint_actuator_id;
    temp_component.axis = axis_of_rotation;
    temp_component.maximum_limit = max_position_limit;
    temp_component.minimum_limit = min_position_limit;

    temp_component.pose_from_world = Eigen::Vector3d::Zero();
    temp_component.orientation_from_world = Eigen::Matrix3d::Identity();

    temp_component.joint_position = 0.0;

    Manipulator.component.push_back(temp_component);
};

void addWorld(std::string world_name,
              std::string child_name,
              Eigen::Vector3d world_pose = Eigen::Vector3d::Zero(),
              Eigen::Matrix3d world_orientation = Eigen::Matrix3d::Identity())
{
    Manipulator.world.name = world_name;
    Manipulator.world.child = child_name;
    Manipulator.world.pose = world_pose;
    Manipulator.world.orientation = world_orientation;
};

void initManipulator()
{
    /*****************************************************************************
      ** Initialize Manipulator Parameter
      *****************************************************************************/
    addWorld("world",   // world name
             "joint1"); // child name

    addJoint("joint1",  // my name
             "world",   // parent name
             "joint2",  // child name
             vector3(0.012, 0.0, 0.017),                // relative position
             convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
             Z_AXIS,    // axis of rotation
             1,        // actuator id
             M_PI,      // max joint limit (3.14 rad)
             -M_PI     // min joint limit (-3.14 rad)
    );

    addJoint("joint2",  // my name
             "joint1",  // parent name
             "joint3",  // child name
             vector3(0.0, 0.0, 0.0595),                // relative position
             convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
             Y_AXIS,    // axis of rotation
             4,        // actuator id
             M_PI_2,    // max joint limit (1.67 rad)
             -2.05    // min joint limit (-2.05 rad)
    );

    addJoint("joint3",  // my name
             "joint2",  // parent name
             "joint4",  // child name
             vector3(0.024, 0.0, 0.128),               // relative position
             convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
             Y_AXIS,    // axis of rotation
             10,        // actuator id
             1.53,      // max joint limit (1.53 rad)
             -M_PI_2  // min joint limit (-1.67 rad)
    );

    addJoint("joint4",  // my name
             "joint3",  // parent name
             "gripper", // child name
             vector3(0.124, 0.0, 0.0),                 // relative position
             convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
             Y_AXIS,    // axis of rotation
             16,        // actuator id
             2.0,       // max joint limit (2.0 rad)
             -1.8      // min joint limit (-1.8 rad)
    );
}

std::string getComponentParentName(std::string my_name)
{
    std::string name = "";
    for(int8_t index = 0; index < Manipulator.component.size(); index++)
    {
        if (Manipulator.component.at(index).my_name == my_name)
        {
            return Manipulator.component.at(index).parent_name;
        }
    }
    return name;
}

Eigen::Vector3d getAxis (std::string my_name)
{
    for(int8_t index = 0; index < Manipulator.component.size(); index++)
    {
        if (Manipulator.component.at(index).my_name == my_name)
        {
            return Manipulator.component.at(index).axis;
        }
    }
}

Eigen::Matrix3d getComponentOrientationFromWorld(std::string my_name)
{
    for(int8_t index = 0; index < Manipulator.component.size(); index++)
    {
        if (Manipulator.component.at(index).my_name == my_name)
        {
            return Manipulator.component.at(index).orientation_from_world;
        }
    }
}

Eigen::Vector3d getComponentPoseFromWorld(std::string my_name)
{
    for(int8_t index = 0; index < Manipulator.component.size(); index++)
    {
        if (Manipulator.component.at(index).my_name == my_name)
        {
            return Manipulator.component.at(index).pose_from_world;
        }
    }
}

std::string getComponentChildName(std::string my_name)
{
    for(int8_t index = 0; index < Manipulator.component.size(); index++)
    {
        if (Manipulator.component.at(index).my_name == my_name)
        {
            return Manipulator.component.at(index).child_name;
        }
    }
}

Eigen::Vector3d getComponentRelativePoseFromParent(std::string my_name)
{
    for(int8_t index = 0; index < Manipulator.component.size(); index++)
    {
        if (Manipulator.component.at(index).my_name == my_name)
        {
            return Manipulator.component.at(index).pose_form_parent;
        }
    }
}

double getJointPosition(std::string my_name)
{
    for(int8_t index = 0; index < Manipulator.component.size(); index++)
    {
        if (Manipulator.component.at(index).my_name == my_name)
        {
            return Manipulator.component.at(index).joint_position;
        }
    }
}

void setComponentPoseFromWorld(std::string my_name, Eigen::Vector3d my_pose_value)
{
    for(int8_t index = 0; index < Manipulator.component.size(); index++)
    {
        if (Manipulator.component.at(index).my_name == my_name)
        {
            Manipulator.component.at(index).pose_from_world = my_pose_value;
        }
    }
}

void setComponentOrientationFromWorld(std::string my_name, Eigen::Matrix3d my_orientation_value)
{
    for(int8_t index = 0; index < Manipulator.component.size(); index++)
    {
        if (Manipulator.component.at(index).my_name == my_name)
        {
            Manipulator.component.at(index).orientation_from_world = my_orientation_value;
        }
    }
}

std::vector<double> getAllActiveJointPosition()
{
    std::vector<double> joint_position;
    for( int8_t index = 0; index < Manipulator.component.size(); index++)
    {
        joint_position.push_back(Manipulator.component.at(index).joint_position);
    }
    return joint_position;
}

void setAllActiveJointPosition (std::vector<double> set_angle)
{
    for( int8_t index = 0; index < Manipulator.component.size(); index++)
    {
       Manipulator.component.at(index).joint_position = set_angle.at(index);
    }
}

Eigen::MatrixXd _jacobian(std::string tool_name)
{
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, Manipulator.dof);

    Eigen::Vector3d joint_axis = Eigen::Vector3d::Zero(3);

    Eigen::Vector3d position_changed = Eigen::Vector3d::Zero(3);
    Eigen::Vector3d orientation_changed = Eigen::Vector3d::Zero(3);
    Eigen::VectorXd pose_changed = Eigen::VectorXd::Zero(6);

    //////////////////////////////////////////////////////////////////////////////////

    int8_t index = 0;
    std::string my_name =  Manipulator.world.child;

    for (int8_t size = 0; size < Manipulator.dof; size++)
    {
        std::string parent_name = getComponentParentName(my_name);
        if (parent_name == Manipulator.world.name)
        {
            joint_axis = Manipulator.world.orientation * getAxis(my_name);
        }
        else
        {
            joint_axis = getComponentOrientationFromWorld(parent_name) * getAxis(my_name);
        }

        position_changed = skewSymmetricMatrix(joint_axis) *
                   (getComponentPoseFromWorld(tool_name) - getComponentPoseFromWorld(my_name));
        orientation_changed = joint_axis;

        pose_changed << position_changed(0),
                        position_changed(1),
                        position_changed(2),
                        orientation_changed(0),
                        orientation_changed(1),
                        orientation_changed(2);

        jacobian.col(index) = pose_changed;
        index++;
        my_name = getComponentChildName(my_name); // Get Child name which has active joint
    }
    return jacobian;
}

void forwardSolverUsingChainRule(std::string component_name)
{
    std::string my_name = component_name;
    if ( my_name == "gripper")
        return;
    //std::cout << my_name << "  " << std::endl;
    std::string parent_name = getComponentParentName(my_name);

    int8_t number_of_child = 1;

    Eigen::Vector3d parent_pose_value;
    Eigen::Matrix3d parent_orientation_value;

    Eigen::Vector3d my_pose_value;
    Eigen::Matrix3d my_orientation_value;
   // Pose parent_pose_value;
   // Pose my_pose_value;

    //Get Parent Pose
    if (parent_name == Manipulator.world.name)
    {
        parent_pose_value = Manipulator.world.pose;
        parent_orientation_value = Manipulator.world.orientation;
    }
    else
    {
        parent_pose_value = getComponentPoseFromWorld(parent_name);
        parent_orientation_value = getComponentOrientationFromWorld(parent_name);
    }

    //position
    my_pose_value = parent_pose_value
                                   + (parent_orientation_value * getComponentRelativePoseFromParent(my_name));
    //orientation
    my_orientation_value = parent_orientation_value * rodriguesRotationMatrix(getAxis(my_name), getJointPosition(my_name));

    setComponentPoseFromWorld(my_name, my_pose_value);
    setComponentOrientationFromWorld(my_name, my_orientation_value);

    std::string child_name = getComponentChildName(my_name);
    //std::cout << "Hello, World!" << std::endl;
    forwardSolverUsingChainRule(child_name);
}

bool chainCustomInverseKinematics(std::string tool_name, Eigen::Vector3d target_pose, Eigen::Matrix3d target_orientation_, std::vector<double> *goal_joint_value)
{
    //solver parameter
    double lambda = 0.0;
    const double param = 0.002;
    const int8_t iteration = 10;

    const double gamma = 0.5;             //rollback delta

    //sr sovler parameter
    double wn_pos = 1 / 0.3;
    double wn_ang = 1 / (2 * M_PI);
    double pre_Ek = 0.0;
    double new_Ek = 0.0;

    Eigen::MatrixXd We(6, 6);
    We << wn_pos, 0, 0, 0, 0, 0,
          0, wn_pos, 0, 0, 0, 0,
          0, 0, wn_pos, 0, 0, 0,
          0, 0, 0, wn_ang, 0, 0,
          0, 0, 0, 0, wn_ang, 0,
          0, 0, 0, 0, 0, wn_ang;

    Eigen::MatrixXd Wn = Eigen::MatrixXd::Identity(Manipulator.dof, Manipulator.dof);

    //jacobian
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(6, Manipulator.dof);
    Eigen::MatrixXd sr_jacobian = Eigen::MatrixXd::Identity(Manipulator.dof, Manipulator.dof);

    //delta parameter
    Eigen::VectorXd pose_changed = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd angle_changed = Eigen::VectorXd::Zero(Manipulator.dof);    //delta angle (dq)
    Eigen::VectorXd gerr(Manipulator.dof);

    //angle parameter
    std::vector<double> present_angle;                                               //angle (q)
    std::vector<double> set_angle;                                                   //set angle (q + dq)

    ////////////////////////////solving//////////////////////////////////

    forwardSolverUsingChainRule(Manipulator.world.child);

    //////////////make target ori//////////  //only OpenManipulator Chain
    Eigen::Matrix3d present_orientation = getComponentOrientationFromWorld(tool_name);
    Eigen::Vector3d present_orientation_rpy = convertRotationMatrixToRPYVector(present_orientation);
    Eigen::Matrix3d target_orientation = target_orientation_;
    Eigen::Vector3d target_orientation_rpy = convertRotationMatrixToRPYVector(target_orientation);

    Eigen::Vector3d joint1_relative_position = getComponentRelativePoseFromParent(Manipulator.world.child);
    Eigen::Vector3d target_position_from_joint1 = target_pose - joint1_relative_position;

    target_orientation_rpy(0) = present_orientation_rpy(0);
    target_orientation_rpy(1) = target_orientation_rpy(1);
    target_orientation_rpy(2) = atan2(target_position_from_joint1(1) ,target_position_from_joint1(0));

    target_orientation_ = convertRPYToRotationMatrix(target_orientation_rpy(0), target_orientation_rpy(1), target_orientation_rpy(2));
    ///////////////////////////////////////

    //////////////checking dx///////////////
    pose_changed = poseDifference(target_pose, getComponentPoseFromWorld(tool_name), target_orientation_, getComponentOrientationFromWorld(tool_name));
    pre_Ek = pose_changed.transpose() * We * pose_changed;
    ///////////////////////////////////////

    //////////////////////////solving loop///////////////////////////////
    for (int8_t count = 0; count < iteration; count++)
    {
    //////////solve using jacobian//////////

        jacobian = _jacobian(tool_name);
        lambda = pre_Ek + param;

        sr_jacobian = (jacobian.transpose() * We * jacobian) + (lambda * Wn);     //calculate sr_jacobian (J^T*we*J + lamda*Wn)
        gerr = jacobian.transpose() * We * pose_changed;                          //calculate gerr (J^T*we) dx

        Eigen::ColPivHouseholderQR<Eigen::MatrixXd> dec(sr_jacobian);                    //solving (get dq)
        angle_changed = dec.solve(gerr);                                          //(J^T*we) * dx = (J^T*we*J + lamda*Wn) * dq

        present_angle = getAllActiveJointPosition();
        set_angle.clear();
        for (int8_t index = 0; index < Manipulator.dof; index++)
            set_angle.push_back(present_angle.at(index) + angle_changed(index));
        setAllActiveJointPosition(set_angle);
        forwardSolverUsingChainRule(Manipulator.world.child);
        ////////////////////////////////////////

        //////////////checking dx///////////////
        pose_changed = poseDifference(target_pose, getComponentPoseFromWorld(tool_name), target_orientation_, getComponentOrientationFromWorld(tool_name));
        new_Ek = pose_changed.transpose() * We * pose_changed;
        ////////////////////////////////////////

        if (new_Ek < 1E-12)
        {
            *goal_joint_value = getAllActiveJointPosition();
            return true;
        }
        else if (new_Ek < pre_Ek)
        {
            pre_Ek = new_Ek;
        }
        else{
            present_angle = getAllActiveJointPosition();
            for (int8_t index = 0; index < Manipulator.dof; index++)
                set_angle.push_back(present_angle.at(index) - (gamma * angle_changed(index)));
            setAllActiveJointPosition(set_angle);

            forwardSolverUsingChainRule(Manipulator.world.child);
        }
    }
    *goal_joint_value = {};
    return false;
}

int main() {

    std::cout << "Hello, World!" << std::endl;
    return 0;
}