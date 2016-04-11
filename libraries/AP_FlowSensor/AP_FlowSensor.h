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

#ifndef __AP_FLOWSENSOR_H__
#define __AP_FLOWSENSOR_H__

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

// Maximum number of FLOWSENSOR measurement instances available on this platform
#define FLOWSENSOR_MAX_INSTANCES 2

class AP_FlowSensor_Backend; 
 
class AP_FlowSensor
{
public:
    friend class AP_FlowSensor_Backend;

    AP_FlowSensor(void);

    // RPM driver types
    enum FlowSensor_Type {
        FLOWSENSOR_TYPE_NONE    = 0,
        FLOWSENSOR_TYPE_PX4_PWM = 1
    };

    // The FlowSensor_State structure is filled in by the backend driver
    struct FlowSensor_State {
        uint8_t                instance;        // the instance number of this RPM
        float                  rate_flowsensor;        // measured rate in revs per minute
        uint32_t               last_reading_ms; // time of last reading
    };

    // parameters for each instance
    AP_Int8  _type[FLOWSENSOR_MAX_INSTANCES];
    AP_Float _scaling[FLOWSENSOR_MAX_INSTANCES];
    AP_Float _maximum[FLOWSENSOR_MAX_INSTANCES];

    static const struct AP_Param::GroupInfo var_info[];
    
    // Return the number of flow sensor instances
    uint8_t num_sensors(void) const {
        return num_instances;
    }

    // detect and initialise any available flow sensors
    void init(void);

    // update state of all flow sensors. Should be called from main loop
    void update(void);

    /*
      return flow for a sensor. Return -1 if not healthy
     */
    float get_flow(uint8_t instance) const {
        if (!healthy(instance)) {
            return -1;
        }
        return state[instance].rate_flowsensor;
    }

    bool healthy(uint8_t instance) const;

private:
    FlowSensor_State state[FLOWSENSOR_MAX_INSTANCES];
    AP_FlowSensor_Backend *drivers[FLOWSENSOR_MAX_INSTANCES];
    uint8_t num_instances:2;

    void detect_instance(uint8_t instance);
    void update_instance(uint8_t instance);  
};
#endif // __FLOWSENSOR_H__
