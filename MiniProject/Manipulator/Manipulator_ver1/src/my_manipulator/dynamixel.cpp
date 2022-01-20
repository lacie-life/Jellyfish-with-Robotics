//
// Created by lacie on 29/08/2019.
//

#include "../../include/my_manipulator/dynamixel.h"
#include <iostream>

using namespace dynamixelManipulator;
using namespace manipulator_base;

/*****************************************************************************
** Joint Dynamixel Profile Control Functions
*****************************************************************************/


JointDynamixelControl::JointDynamixelControl(float control_loop_time)
{
    control_loop_time_ = control_loop_time;
}

void JointDynamixelControl::init(std::vector<uint8_t> actuator_id, const void *arg)
{
    STRING *get_arg_ = (STRING *)arg;

    bool result = JointDynamixelControl::initialize(actuator_id ,get_arg_[0], get_arg_[1]);

    if (result == false)
        return;
}

void JointDynamixelControl::setMode(std::vector<uint8_t> actuator_id, const void *arg)
{
    bool result = false;

    STRING *get_arg_ = (STRING *)arg;

    if (get_arg_[0] == "position_mode" || get_arg_[0] == "current_based_position_mode")
    {
        result = JointDynamixelControl::setOperatingMode(actuator_id, get_arg_[0]);
        if (result == false)
            return;

    }
    return;
}

std::vector<uint8_t> JointDynamixelControl::getId()
{
    return dynamixel_.id;
}

void JointDynamixelControl::enable()
{
    const char* log = NULL;
    bool result = false;

    for (uint32_t index = 0; index < dynamixel_.num; index++)
    {
        write(portHandler, packetHandler1, dynamixel_.id.at(index), ADDRESS_TORQUE, 2, 1);
    }
    enabled_state_ = true;
}

void JointDynamixelControl::disable()
{
    const char* log = NULL;
    bool result = false;

    for (uint32_t index = 0; index < dynamixel_.num; index++)
    {
        write(portHandler, packetHandler1, dynamixel_.id.at(index), ADDRESS_TORQUE, 2, 0);
    }
    enabled_state_ = false;
}

bool JointDynamixelControl::sendJointActuatorValue(std::vector<uint8_t> actuator_id, std::vector<manipulator_base::ActuatorValue> value_vector)
{
    bool result = false;
    double test_value[4] = {0, 0, 0, 0};
    for (int8_t index = 0; index < value_vector.size(); index++)
    {
        value_vector.at(index).position = test_value[index];
       // std::cout << value_vector.at(index).position << std::endl;
    }
    result = JointDynamixelControl::writeGoalProfilingControlValue(actuator_id, value_vector);
    if (result == false)
        return false;

    return true;
}

std::vector<manipulator_base::ActuatorValue> JointDynamixelControl::receiveJointActuatorValue(std::vector<uint8_t> actuator_id)
{
    return JointDynamixelControl::receiveAllDynamixelValue(actuator_id);
}


/*****************************************************************************
** Functions called in Joint Dynamixel Profile Control Functions
*****************************************************************************/
int32_t JointDynamixelControl::convertRadian2Value(float radian)
{
    int32_t position = 0;
    if (radian > 0)
    {
        position = (radian * (1023 - 512) / 2.61799) + 512;
    }
    else if (radian < 0)
    {
        position = (radian * (0 - 512) / (-2.61799)) + 512;
    }
    else
    {
        position = 512;
    }
    return position;
}

float JointDynamixelControl::convertValue2Radian(int value)
{
    float radian = 0.0;
    if (value > 512)
    {
        radian = (float)(value - 512) * 2.61799 / (float)(1023 - 512);
    }
    else if (value < 512)
    {
        radian = (float)(value - 512) * (-2.61799) / (float)(0 - 512);
    }

    return radian;
}

void JointDynamixelControl::write(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint16_t length, uint32_t value)
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

int JointDynamixelControl::read(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint16_t length)
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
bool JointDynamixelControl::initialize(std::vector<uint8_t> actuator_id, STRING dxl_device_name, STRING dxl_baud_rate)
{
    dynamixel_.id = actuator_id;
    dynamixel_.num = actuator_id.size();

    packetHandler1 = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION1);
    portHandler = dynamixel::PortHandler::getPortHandler(dxl_device_name.c_str());

    if (portHandler->openPort())
    {
        portHandler->setBaudRate(1000000);

    }

    for (uint8_t index = 0; index < dynamixel_.num; index++)
    {

        uint8_t id = dynamixel_.id.at(index);
        uint8_t dxl_error;
        uint16_t dxl_model_num;

        if (packetHandler1-> ping(portHandler, id, &dxl_model_num, &dxl_error) == COMM_SUCCESS)
        {

            fprintf(stderr, " [ID:%.3d] Model No : %.5d \n", id, dxl_model_num);
        }
    }
    return true;
}

bool JointDynamixelControl::setOperatingMode(std::vector<uint8_t> actuator_id, STRING dynamixel_mode)
{
    if (dynamixel_mode == "position_mode")
    {
        for (uint8_t num = 0; num < actuator_id.size(); num++)
        {
            // Set joint mode for Dynamixel motors
            write(portHandler, packetHandler1, actuator_id.at(num), ADDRESS_CW, 2, CW_ANGLE_LIMIT);
            write(portHandler, packetHandler1, actuator_id.at(num), ADDRESS_CCW, 2, CCW_ANGLE_LIMIT);
        }
    }
    return true;
}

bool JointDynamixelControl::writeGoalProfilingControlValue(std::vector<uint8_t> actuator_id, std::vector<manipulator_base::ActuatorValue> value_vector)
{
    uint8_t id_array[actuator_id.size()];
    int32_t goal_value[actuator_id.size()];

    //add tarajectory eq.
    for(uint8_t index = 0; index < actuator_id.size(); index++)
    {
        float result_position;
        float time_control = control_loop_time_;       //ms

        if(previous_goal_value_.find(actuator_id.at(index)) == previous_goal_value_.end())
        {
            previous_goal_value_.insert(std::make_pair(actuator_id.at(index), value_vector.at(index)));
        }

        result_position = value_vector.at(index).position + 3*(value_vector.at(index).velocity * (time_control))/2;

        id_array[index] = actuator_id.at(index);
        ///////////////////////////////////////////////////////////////////////////////////////////////////////
        goal_value[index] = convertRadian2Value(result_position);
        previous_goal_value_[actuator_id.at(index)] = value_vector.at(index);
    }
    for (uint8_t index = 0; index < actuator_id.size(); index++)
    {
        write(portHandler, packetHandler1, id_array[index], ADDRESS_SET_POSITION, 2, goal_value[index]);
    }

    return true;
}

std::vector<manipulator_base::ActuatorValue> JointDynamixelControl::receiveAllDynamixelValue(std::vector<uint8_t> actuator_id)
{
    std::vector<manipulator_base::ActuatorValue> all_actuator;

    uint8_t id_array[actuator_id.size()];
    for (uint8_t index = 0; index < actuator_id.size(); index++)
        id_array[index] = actuator_id.at(index);

    int32_t get_position[actuator_id.size()];

    for (uint8_t index = 0; index < actuator_id.size(); index++)
    {
        get_position[index] = read(portHandler, packetHandler1, actuator_id.at(index), ADDRESS_GET_POSITION, 2);
    }

    for (uint8_t index = 0; index < actuator_id.size(); index++)
    {
        manipulator_base::ActuatorValue actuator;
        actuator.position = convertValue2Radian(get_position[index]);

        all_actuator.push_back(actuator);
    }
    return all_actuator;
}

/*****************************************************************************
** Tool Dynamixel Control Functions
*****************************************************************************/
 void GripperDynamixel::init(uint8_t actuator_id, const void *arg)
 {
   STRING *get_arg_ = (STRING *)arg;

   bool result = GripperDynamixel::initialize(actuator_id ,get_arg_[0], get_arg_[1]);

   if (result == false)
     return;
 }

 void GripperDynamixel::setMode(const void *arg)
 {
   bool result = false;
 // const char* log = NULL;

   STRING *get_arg_ = (STRING *)arg;

   if (get_arg_[0] == "position_mode" || get_arg_[0] == "current_based_position_mode")
   {
     result = GripperDynamixel::setOperatingMode(get_arg_[0]);
     if (result == false)
       return;
   }
 }

 uint8_t GripperDynamixel::getId()
 {
   return dynamixel_.id.at(0);
 }

 void GripperDynamixel::enable()
 {
     write(portHandler, packetHandler1, dynamixel_.id.at(0), ADDRESS_TORQUE, 2, 1);
     enabled_state_ = true;
 }

 void GripperDynamixel::disable()
 {
     write(portHandler, packetHandler1, dynamixel_.id.at(0), ADDRESS_TORQUE, 2, 0);
     enabled_state_ = false;
 }

 bool GripperDynamixel::sendToolActuatorValue(manipulator_base::ActuatorValue value)
 {
   return GripperDynamixel::writeGoalPosition(value.position);
 }

 manipulator_base::ActuatorValue GripperDynamixel::receiveToolActuatorValue()
 {
   manipulator_base::ActuatorValue result;
   result.position = GripperDynamixel::receiveDynamixelValue();
   result.velocity = 0.0;
   result.acceleration = 0.0;
   result.effort = 0.0;
   return result;
 }


 /*****************************************************************************
 ** Functions called in Tool Dynamixel Profile Control Functions
 *****************************************************************************/
 int32_t GripperDynamixel::convertRadian2Value(float radian)
 {
     int32_t position = 0;
     if (radian > 0)
     {
         position = (radian * (1023 - 512) / 2.61799) + 512;
     }
     else if (radian < 0)
     {
         position = (radian * (0 - 512) / (-2.61799)) + 512;
     }
     else
     {
         position = 512;
     }
     return position;
 }

 float GripperDynamixel::convertValue2Radian(int value)
{
    float radian = 0.0;
    if (value > 512)
    {
        radian = (float)(value - 512) * 2.61799 / (float)(1023 - 512);
    }
    else if (value < 512)
    {
        radian = (float)(value - 512) * (-2.61799) / (float)(0 - 512);
    }

    return radian;
}

 void GripperDynamixel::write(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint16_t length, uint32_t value)
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

 int GripperDynamixel::read(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint16_t length)
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
 bool GripperDynamixel::initialize(uint8_t actuator_id, STRING dxl_device_name, STRING dxl_baud_rate)
 {
     dynamixel_.id.at(0) = actuator_id;
     dynamixel_.num = 1;

     packetHandler1 = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION1);
     portHandler = dynamixel::PortHandler::getPortHandler(dxl_device_name.c_str());

     if (portHandler->openPort())
     {
         portHandler->setBaudRate(1000000);
     }
     uint8_t id = dynamixel_.id.at(0);
     uint8_t dxl_error;
     uint16_t dxl_model_num;

     if (packetHandler1-> ping(portHandler, id, &dxl_model_num, &dxl_error) == COMM_SUCCESS)
     {

         fprintf(stderr, " [ID:%.3d] Model No : %.5d \n", id, dxl_model_num);
     }

   return true;
 }

 bool GripperDynamixel::setOperatingMode(STRING dynamixel_mode)
 {
   const char* log = NULL;
   bool result = false;

   const uint32_t velocity = 0;
   const uint32_t acceleration = 0;
   const uint32_t current = 200;

   if (dynamixel_mode == "position_mode")
   {
       // Set joint mode for Dynamixel motors
       write(portHandler, packetHandler1, dynamixel_.id.at(0), ADDRESS_CW, 2, CW_ANGLE_LIMIT);
       write(portHandler, packetHandler1, dynamixel_.id.at(0), ADDRESS_CCW, 2, CCW_ANGLE_LIMIT);
   }
   return true;
 }
 bool GripperDynamixel::writeGoalPosition(double radian)
 {

   int32_t goal_position = 0;

   goal_position = convertRadian2Value(radian);
     write(portHandler, packetHandler1, dynamixel_.id.at(0), ADDRESS_SET_POSITION, 2, goal_position);

   return true;
 }

 double GripperDynamixel::receiveDynamixelValue()
 {
   int32_t get_value = 0;
   get_value = read(portHandler, packetHandler1, dynamixel_.id.at(0), ADDRESS_GET_POSITION, 2);
   return convertValue2Radian(get_value);

 }






