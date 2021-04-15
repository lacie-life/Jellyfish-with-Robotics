#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <math.h>

Eigen::MatrixXd convertCoordiane(float x_angle, float y_angele, float z_angele, Eigen::MatrixXd Q)
{
    Eigen::Matrix3f R_z, R_x, R_y;
    R_z << cos(z_angele), -sin(z_angele), 0,
           sin(z_angele), cos(z_angele), 0,
           0, 0, 1;
    R_y << cos(y_angele), 0, sin(y_angele),
           0, 1, 0,
           -sin(y_angele), 0, cos(y_angele);
    R_x << 1, 0, 0,
           0, cos(x_angle), -sin(x_angle),
           0, sin(x_angle), cos(x_angle);

    Eigen::Matrix3f rotationMatrix = R_x*R_y*R_z;

    Eigen::MatrixXf transformMatrix(4, 4);

    transformMatrix << rotationMatrix(0,0), rotationMatrix(0,1), rotationMatrix(0,2), Q(0,0),
                       rotationMatrix(1,0), rotationMatrix(1,1), rotationMatrix(1,2), Q(1,0),
                       rotationMatrix(2,0), rotationMatrix(2,1), rotationMatrix(2,2), Q(2,0),
                       0, 0, 0, 1;
    
    return transformMatrix;

}

int main(){

    float x_angle, y_angle, z_angle;
    float x, y, z;
    Eigen::MatrixXf Q(3,1);

    std::cout << "X angle: ";
    std::cin >> x_angle;
    std::cout << "\n"; 

    std::cout << "Y angle: ";
    std::cin >> y_angle;
    std::cout << "\n"; 

    std::cout << "Z angle: ";
    std::cin >> z_angle;
    std::cout << "\n"; 

    std::cout << "Q :";
    std::cin >> x;
    std::cin >> y;
    std::cin >> z;
    Q << x,
         y,
         z;

    Eigen::MatrixXf m = convertCoordiane(x_angle, y_angle, z_angle, Q);
    std::cout << Q;

    return 0;
}