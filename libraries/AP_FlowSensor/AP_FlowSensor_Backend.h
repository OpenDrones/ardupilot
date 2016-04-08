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

#ifndef __AP_FLOWSENSOR_BACKEND_H__
#define __AP_FLOWSENSOR_BACKEND_H__

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_FlowSensor.h"

class AP_FlowSensor_Backend
{
public:
    // constructor. This incorporates initialisation as well.
	  AP_FlowSensor_Backend(AP_FlowSensor &_ap_flowsensor, uint8_t instance, AP_FlowSensor::FlowSensor_State &_state) ;

    // we declare a virtual destructor so that RPM drivers can
    // override with a custom destructor if need be
    virtual ~AP_FlowSensor_Backend(void) {}

    // update the state structure. All backends must implement this.
    virtual void update() = 0;

protected:

    AP_FlowSensor &ap_flowsensor;
    AP_FlowSensor::FlowSensor_State &state;
};
#endif // __AP_FLOWSENSOR_BACKEND_H__
