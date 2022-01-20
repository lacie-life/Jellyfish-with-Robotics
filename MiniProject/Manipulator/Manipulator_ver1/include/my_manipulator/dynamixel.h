//
// Created by lacie on 29/08/2019.
//

#ifndef MANIPULATOR_VER1_DYNAMIXEL_H
#define MANIPULATOR_VER1_DYNAMIXEL_H

#include "../../include/manipulator_base/manipulator.h"
#include "../../include/dynamixel_sdk/dynamixel_sdk.h"

namespace dynamixelManipulator
{
    // Motor Torque On/Off
    #define ADDRESS_TORQUE       24
    //Limit of angle 0-300 degree
    #define CW_ANGLE_LIMIT   0
    #define CCW_ANGLE_LIMIT  1023

    #define ADDRESS_SET_POSITION 30
    #define ADDRESS_GET_POSITION 36
    #define ADDRESS_CW           6
    #define ADDRESS_CCW          8

    #define CONTROL_LOOP_TIME 10;    //ms

    #define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller

    // Protocol 1.0
    #define PROTOCOL_VERSION1               1.0                 // See which protocol version is used in the Dynamixel

    typedef struct
    {
        std::vector<uint8_t> id;
        uint8_t num;
    } Joint;

    class JointDynamixelControl : public manipulator_base::JointActuator
    {
    private:
        dynamixel::PacketHandler *packetHandler1;
        dynamixel::PortHandler *portHandler;
        Joint dynamixel_;
        float control_loop_time_; // unit: ms
        std::map<uint8_t, manipulator_base::ActuatorValue> previous_goal_value_;

    public:
        JointDynamixelControl(float control_loop_time = 0.010);
        virtual ~JointDynamixelControl(){}


        /*****************************************************************************
        ** Joint Dynamixel Control Functions
        *****************************************************************************/
        virtual void init(std::vector<uint8_t> actuator_id, const void *arg);
        virtual void setMode(std::vector<uint8_t> actuator_id, const void *arg);
        virtual std::vector<uint8_t> getId();

        virtual void enable();
        virtual void disable();

        virtual bool sendJointActuatorValue(std::vector<uint8_t> actuator_id, std::vector<manipulator_base::ActuatorValue> value_vector);
        virtual std::vector<manipulator_base::ActuatorValue> receiveJointActuatorValue(std::vector<uint8_t> actuator_id);


        /*****************************************************************************
        ** Functions called in Joint Dynamixel Profile Control Functions
        *****************************************************************************/
        int32_t convertRadian2Value(float radian);
        float convertValue2Radian(int value);
        void write(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint16_t length, uint32_t value);
        int read(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint16_t length);
        bool initialize(std::vector<uint8_t> actuator_id, STRING dxl_device_name, STRING dxl_baud_rate);
        bool setOperatingMode(std::vector<uint8_t> actuator_id, STRING dynamixel_mode = "position_mode");
        bool writeGoalProfilingControlValue(std::vector<uint8_t> actuator_id, std::vector<manipulator_base::ActuatorValue> value_vector);
        std::vector<manipulator_base::ActuatorValue> receiveAllDynamixelValue(std::vector<uint8_t> actuator_id);
    };


    class GripperDynamixel : public manipulator_base::ToolActuator
    {
    private:
        dynamixel::PacketHandler *packetHandler1;
        dynamixel::PortHandler *portHandler;
        Joint dynamixel_;

    public:
        GripperDynamixel() {}
        virtual ~GripperDynamixel() {}


        /*****************************************************************************
        ** Tool Dynamixel Control Functions
        *****************************************************************************/
        virtual void init(uint8_t actuator_id, const void *arg);
        virtual void setMode(const void *arg);
        virtual uint8_t getId();

        virtual void enable();
        virtual void disable();

        virtual bool sendToolActuatorValue(manipulator_base::ActuatorValue value);
        virtual manipulator_base::ActuatorValue receiveToolActuatorValue();


        /*****************************************************************************
        ** Functions called in Tool Dynamixel Profile Control Functions
        *****************************************************************************/
        int32_t convertRadian2Value(float radian);
        float convertValue2Radian(int value);
        void write(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint16_t length, uint32_t value);
        int read(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint16_t length);
        bool initialize(uint8_t actuator_id, STRING dxl_device_name, STRING dxl_baud_rate);
        bool setOperatingMode(STRING dynamixel_mode = "position_mode");
        bool writeGoalPosition(double radian);
        double receiveDynamixelValue();
    };

}
#endif //MANIPULATOR_VER1_DYNAMIXEL_H
