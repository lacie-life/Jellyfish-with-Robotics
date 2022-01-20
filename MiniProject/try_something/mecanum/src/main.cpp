#include <iostream>
#include <eigen3/Eigen/Eigen>

using namespace Eigen;
using namespace std;

#define L1 22.5/2
#define L2 24.5/2
#define R 0.33

int main() {
    MatrixXf velocity(4,1);
    MatrixXf setUp(4,3);
    MatrixXf pose(3,1);

    setUp << 1, 1, -(L1+L2),
             1, -1, (L1+L2),
             1, -1, -(L1+L2),
             1, 1, (L1+L2);

    cout << setUp << "\n";

    for (int index = 0; index < 3; index++)
    {
        cin >> pose(index, 0);
    }
    cout << pose << "\n";

    velocity = setUp*pose;

    velocity = R*velocity;

    cout << velocity << "\n";

    return 0;
}