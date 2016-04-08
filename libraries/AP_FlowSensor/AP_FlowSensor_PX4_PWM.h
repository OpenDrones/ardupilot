// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#ifndef AP_FLOWSENSOR_PX4_PWM_H
#define AP_FLOWSENSOR_PX4_PWM_H

#include "AP_FlowSensor.h"
#include "AP_FlowSensor_Backend.h"

class AP_FlowSensor_PX4_PWM : public AP_FlowSensor_Backend
{
public:
    // constructor
    AP_FlowSensor_PX4_PWM(AP_FlowSensor &_ap_flowsensor, uint8_t instance, AP_FlowSensor::FlowSensor_State &_state);

    // destructor
    ~AP_FlowSensor_PX4_PWM(void);
    
    // update state
    void update(void);

private:
    int _fd = -1;
    uint64_t _last_timestamp = 0;
};

#endif // AP_FLOWSENSOR_PX4_PWM_H
