//
// Created by lacie on 29/08/2019.
//

#include "../../include/manipulator_base/manipulator_manager.h"

using namespace manipulator_base;

bool JointActuator::findId(uint8_t actuator_id)
{
    std::vector<uint8_t> id = getId();
    for(uint32_t index = 0; index < id.size(); index++)
    {
        if(id.at(index) == actuator_id)
            return true;
    }
    return false;
}

bool JointActuator::getEnabledState()
{
    return enabled_state_;
}

bool ToolActuator::findId(uint8_t actuator_id)
{
    if(getId() == actuator_id)
    {
        return true;
    }
    return false;
}

bool ToolActuator::getEnabledState()
{
    return enabled_state_;
}