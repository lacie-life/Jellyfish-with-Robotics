#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <getopt.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <dynamixel_sdk.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

using namespace Eigen;


#define jointNumber 5
#define pi          3.14159
#define L12         4
#define L23         10.5
#define L34         8.5
#define L45         11
#define Lp          4
#define scalar      300/360

#define DEVICENAME                      "/dev/ttyUSB1"      // Check which port is being used on your controller

// ID of Dynamixels
#define JOINT_1 1
#define JOINT_2 2
#define JOINT_3 3
#define JOINT_4 4
#define JOINT_5 5

// Original position of Joints
#define POSITION_JOINT_1 512
#define POSITION_JOINT_2 512
#define POSITION_JOINT_3 512
#define POSITION_JOINT_4 512
#define POSITION_JOINT_5 512
#define CW_ANGLE_LIMIT   0
#define CCW_ANGLE_LIMIT  1023



#define ADDRESS_SET_POSITION 30
#define ADDRESS_GET_POSITION 36
#define ADDRESS_CW           6
#define ADDRESS_CCW          8


// Protocol version
#define PROTOCOL_VERSION1               1.0                 // See which protocol version is used in the Dynamixel

int getch()
{
#if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
#elif defined(_WIN32) || defined(_WIN64)
    return _getch();
#endif
}

int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
#elif defined(_WIN32) || defined(_WIN64)
    return _kbhit();
#endif
}


void scan(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler1, dynamixel::PacketHandler *packetHandler2)
{
    uint8_t dxl_error;
    uint16_t dxl_model_num;

    fprintf(stderr, "\n");
    fprintf(stderr, "Scan Dynamixel Using Protocol 1.0\n");
    for (int id = 1; id < 253; id++)
    {
        if (packetHandler1-> ping(portHandler, id, &dxl_model_num, &dxl_error)== COMM_SUCCESS)
        {
            fprintf(stderr, "\n                                          ... SUCCESS \r");
            fprintf(stderr, " [ID:%.3d] Model No : %.5d \n", id, dxl_model_num);
        }
        else
            fprintf(stderr, ".");

        if (kbhit())
        {
            char c = getch();
            if (c == 0x1b)
                break;
        }
    }
    fprintf(stderr, "\n\n");

    fprintf(stderr, "Scan Dynamixel Using Protocol 2.0\n");
    for (int id = 1; id < 253; id++)
    {
        if (packetHandler2-> ping(portHandler, id, &dxl_model_num, &dxl_error)== COMM_SUCCESS)
        {
            fprintf(stderr, "\n                                          ... SUCCESS \r");
            fprintf(stderr, " [ID:%.3d] Model No : %.5d \n", id, dxl_model_num);
        }
        else
        {
            fprintf(stderr, ".");
        }

        if (kbhit())
        {
            char c = getch();
            if (c == 0x1b) break;
        }
    }
    fprintf(stderr, "\n\n");
}

void write(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint16_t length, uint32_t value)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    if (length == 1)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, addr, (uint8_t)value, &dxl_error);
    }
    else if (length == 2)
    {
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, addr, (uint16_t)value, &dxl_error);
    }
    else if (length == 4)
    {
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, addr, (uint32_t)value, &dxl_error);
    }

    if (dxl_comm_result == COMM_SUCCESS)
    {
        if (dxl_error != 0) printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        //fprintf(stderr, "\n Success to write\n\n");
    }
    else
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        fprintf(stderr, "\n Fail to write! \n\n");
    }
}

int read(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint16_t length)
{
    uint8_t dxl_error = 0;
    int     dxl_comm_result = COMM_TX_FAIL;

    int8_t  value8    = 0;
    int16_t value16   = 0;
    int32_t value32   = 0;


    if (length == 1)
    {
        dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, id, addr, (uint8_t*)&value8, &dxl_error);
    }
    else if (length == 2)
    {
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, addr, (uint16_t*)&value16, &dxl_error);
    }
    else if (length == 4)
    {
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, id, addr, (uint32_t*)&value32, &dxl_error);
    }

    if (dxl_comm_result == COMM_SUCCESS)
    {
        if (dxl_error != 0) printf("%s\n", packetHandler->getRxPacketError(dxl_error));

        if (length == 1)
        {
            //fprintf(stderr, "\n READ VALUE : (UNSIGNED) %u , (SIGNED) %d \n\n", (uint8_t)value8, value8);
            return value8;
        }
        else if (length == 2)
        {
            //fprintf(stderr, "\n READ VALUE : (UNSIGNED) %u , (SIGNED) %d \n\n", (uint16_t)value16, value16);
            return value16;
        }
        else if (length == 4)
        {
            //fprintf(stderr, "\n READ VALUE : (UNSIGNED) %u , (SIGNED) %d \n\n", (uint32_t)value32, value32);
            return value32;
        }
    }
    else
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        fprintf(stderr, "\n Fail to read! \n\n");
        return COMM_RX_FAIL;

    }
}

void dump(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint16_t len)
{
    uint8_t  dxl_error       = 0;
    int      dxl_comm_result = COMM_TX_FAIL;
    uint8_t *data            = (uint8_t*)calloc(len, sizeof(uint8_t));

    dxl_comm_result = packetHandler->readTxRx(portHandler, id, addr, len, data, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS)
    {
        if (dxl_error != 0)
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));

        if (id != BROADCAST_ID)
        {
            fprintf(stderr, "\n");
            for (int i = addr; i < addr+len; i++)
                fprintf(stderr, "ADDR %.3d [0x%.4X] :     %.3d [0x%.2X] \n", i, i, data[i-addr], data[i-addr]);
            fprintf(stderr, "\n");
        }
    }
    else
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        fprintf(stderr, "\n Fail to read! \n\n");
    }

    free(data);
}

void startDynamixel(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler1)
{
    // Set joint mode for Dynamixel motors
    write(portHandler, packetHandler1, JOINT_1, ADDRESS_CW, 2, CW_ANGLE_LIMIT);
    write(portHandler, packetHandler1, JOINT_1, ADDRESS_CCW, 2, CCW_ANGLE_LIMIT);

    write(portHandler, packetHandler1, JOINT_2, ADDRESS_CW, 2, CW_ANGLE_LIMIT);
    write(portHandler, packetHandler1, JOINT_2, ADDRESS_CCW, 2, CCW_ANGLE_LIMIT);

    write(portHandler, packetHandler1, JOINT_3, ADDRESS_CW, 2, CW_ANGLE_LIMIT);
    write(portHandler, packetHandler1, JOINT_3, ADDRESS_CCW, 2, CCW_ANGLE_LIMIT);

    write(portHandler, packetHandler1, JOINT_4, ADDRESS_CW, 2, CW_ANGLE_LIMIT);
    write(portHandler, packetHandler1, JOINT_4, ADDRESS_CCW, 2, CCW_ANGLE_LIMIT);

    write(portHandler, packetHandler1, JOINT_5, ADDRESS_CW, 2, CW_ANGLE_LIMIT);
    write(portHandler, packetHandler1, JOINT_5, ADDRESS_CCW, 2, CCW_ANGLE_LIMIT);

    // Send origin position to Dynamixel motors
    write(portHandler, packetHandler1, JOINT_1, ADDRESS_SET_POSITION, 2, POSITION_JOINT_1);
    write(portHandler, packetHandler1, JOINT_2, ADDRESS_SET_POSITION, 2, POSITION_JOINT_2);
    write(portHandler, packetHandler1, JOINT_3, ADDRESS_SET_POSITION, 2, POSITION_JOINT_3);
    write(portHandler, packetHandler1, JOINT_4, ADDRESS_SET_POSITION, 2, POSITION_JOINT_4);
    write(portHandler, packetHandler1, JOINT_5, ADDRESS_SET_POSITION, 2, POSITION_JOINT_5);
}

MatrixXd initRotationMatrix(float angle)
{
    MatrixXd rotationMatrix(3,3);
    rotationMatrix << 1, 0         ,  0         ,
                      0, cos(angle), -sin(angle),
                      0, sin(angle),  cos(angle);
    return rotationMatrix;
}

MatrixXd initLink(float length)
{
    MatrixXd link(3,1);
    link << 0,
            0,
            length;
    return link;
}

MatrixXd initTransMatrix(MatrixXd rotationMatrix, MatrixXd link)
{
    MatrixXd transMatrix (4,4);
//    std::cout << "xxx" << std::endl;
//    std::cout << rotationMatrix << std::endl;
//    std::cout << link << std::endl;
    transMatrix << rotationMatrix(0,0), rotationMatrix(0,1), rotationMatrix(0,2), link(0),
                   rotationMatrix(1,0), rotationMatrix(1,1), rotationMatrix(1,2), link(1),
                   rotationMatrix(2,0), rotationMatrix(2,1), rotationMatrix(2,2), link(2),
                   0                  , 0                  , 0                  , 1      ;
    return transMatrix;
}



int main() {

    char *dev_name = (char*)DEVICENAME;

    // Initialize Packethandler1 instance
    dynamixel::PacketHandler *packetHandler1 = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION1);

    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(dev_name);

    // Angles are entered by user
    float angle[jointNumber];
    float angleRadian[jointNumber];
    int position[jointNumber];

    // Links
    MatrixXd q0(3,1), q12(3,1), q23(3,1), q34(3,1), q45(3,1), qP(3,1);

    //Rotation Matrixs
    MatrixXd R0(3,3), R12(3,3), R23(3,3), R34(3,3), R45(3,3);

    //Trans Matrixs
    MatrixXd T0(4,4), T12(4,4), T23(4,4), T34(4,4), Tp(4,4), T45(4,4);

    // Open port
    if (portHandler->openPort())
    {
        printf("Succeeded to open the port!\n\n");
        printf(" - Device Name : %s\n", dev_name);
        portHandler->setBaudRate(1000000);
        printf(" - Baudrate    : %d\n\n", portHandler->getBaudRate());
    }
    else
    {
        printf("Failed to open the port! [%s]\n", dev_name);
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }

    startDynamixel(portHandler, packetHandler1);

    while (1)
    {
        // Enter angle by degree
        for (int i= 0; i < jointNumber; i++ )
        {
            std::cin >> angle[i];
        }

        // Trans angle from degree to radian
        for (int i = 0; i < jointNumber; i++)
        {
            angleRadian[i] = (angle[i]*pi)/180;
        }

        // Trans angle from degree to position in Dynamixels
        std::cout << "Dynamixel position :" << std::endl;
        for (int i = 0; i < jointNumber; i++)
        {
            if (i == 2 || i == 4 )
            {
                position[i] = 512 - (int)(angle[i]*scalar);
            }
            else
            {
                position[i] = 512 + (int)(angle[i]*scalar);
            }
            std::cout << position[i] << std::endl;
        }

        // Send position to Dynamixel motor
        write(portHandler, packetHandler1, JOINT_1, ADDRESS_SET_POSITION, 2, position[0]);
        write(portHandler, packetHandler1, JOINT_2, ADDRESS_SET_POSITION, 2, position[1]);
        write(portHandler, packetHandler1, JOINT_3, ADDRESS_SET_POSITION, 2, position[2]);
        write(portHandler, packetHandler1, JOINT_4, ADDRESS_SET_POSITION, 2, position[3]);
        write(portHandler, packetHandler1, JOINT_5, ADDRESS_SET_POSITION, 2, position[4]);

        // Read position from Dynamixel motors
        std::cout << " JOINT_1 :" << read(portHandler, packetHandler1, JOINT_1, ADDRESS_GET_POSITION, 2) << std::endl;
        std::cout << " JOINT_2 :" << read(portHandler, packetHandler1, JOINT_2, ADDRESS_GET_POSITION, 2) << std::endl;
        std::cout << " JOINT_3 :" << read(portHandler, packetHandler1, JOINT_3, ADDRESS_GET_POSITION, 2) << std::endl;
        std::cout << " JOINT_4 :" << read(portHandler, packetHandler1, JOINT_4, ADDRESS_GET_POSITION, 2) << std::endl;
        std::cout << " JOINT_5 :" << read(portHandler, packetHandler1, JOINT_5, ADDRESS_GET_POSITION, 2) << std::endl;

        //Init Rotation Matrix
        R12 = initRotationMatrix(angleRadian[1]);
        R23 = initRotationMatrix(angleRadian[2]);
        R34 = initRotationMatrix(angleRadian[3]);
        R45 = initRotationMatrix(angleRadian[4]);

        R0 << cos(angleRadian[0]), -sin(angleRadian[0]), 0,
                sin(angleRadian[0]), cos(angleRadian[0]) , 0,
                0                  , 0                   , 1;

        std::cout << "Rotation Matrix : \n" << std::endl;
        std::cout << "R0 :  \n"  << R0 << std::endl;
        std::cout << "R12 : \n" << R12 << std::endl;
        std::cout << "R23 : \n" << R23 << std::endl;
        std::cout << "R34 : \n" << R34 << std::endl;
        std::cout << "R45 : \n" << R45 << std::endl;


        // Init Links
        q0  = initLink(0);
        q12 = initLink(L12);
        q23 = initLink(L23);
        q34 = initLink(L34);
        q45 = initLink(L45);

        qP << 0,
                sin(angle[4])*Lp,
                cos(angle[4])*Lp;

        std::cout << "Links : \n" << std::endl;
        std::cout << "q0 :  \n"  << q0 << std::endl;
        std::cout << "q12 : \n" << q12 << std::endl;
        std::cout << "q23 : \n" << q23 << std::endl;
        std::cout << "q34 : \n" << q34 << std::endl;
        std::cout << "q45 : \n" << q45 << std::endl;

        // Init Trans Matrix
        T0  = initTransMatrix(R0,  q0);
        T12 = initTransMatrix(R12, q12);
        T23 = initTransMatrix(R23, q23);
        T34 = initTransMatrix(R34, q34);
        T45 = initTransMatrix(R45, q45);

        Tp = T0*T12*T23*T34*T45;

        std::cout << "Trans Matrix : \n" << std::endl;
        std::cout << "T0 :  \n"  << T0 << std::endl;
        std::cout << "T12 : \n" << T12 << std::endl;
        std::cout << "T23 : \n" << T23 << std::endl;
        std::cout << "T34 : \n" << T34 << std::endl;
        std::cout << "T45 : \n" << T45 << std::endl;
        std::cout << "Tp :  \n"  << Tp << std::endl;


        MatrixXd P(4,1);
        P << qP(0),
                qP(1),
                qP(2),
                1;

        std::cout << "qP :  \n" << qP << std::endl;

        std::cout << "P : \n" << Tp*P << std::endl;
    }

    return 0;
}