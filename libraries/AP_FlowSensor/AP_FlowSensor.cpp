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

#include "AP_FlowSensor.h"
#include "AP_FlowSensor_PX4_PWM.h"

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_FlowSensor::var_info[] = {
    // @Param: _TYPE
    // @DisplayName: RPM type
    // @Description: What type of RPM sensor is connected
    // @Values: 0:None,1:PX4-PWM
    AP_GROUPINFO("_TYPE",    0, AP_FlowSensor, _type[0], 1),

    // @Param: _SCALING
    // @DisplayName: RPM scaling
    // @Description: Scaling factor between sensor reading and RPM.
    // @Increment: 0.001
    AP_GROUPINFO("_SCALING", 1, AP_FlowSensor, _scaling[0], 0.056f),

    // @Param: _MAX
    // @DisplayName: Maximum RPM
    // @Description: Maximum RPM to report
    // @Increment: 1
    AP_GROUPINFO("_MAX", 2, AP_FlowSensor, _maximum[0], 30),

#if RPM_MAX_INSTANCES > 1
    // @Param: 2_TYPE
    // @DisplayName: Second RPM type
    // @Description: What type of RPM sensor is connected
    // @Values: 0:None,1:PX4-PWM
    AP_GROUPINFO("2_TYPE",    10, AP_FlowSensor, _type[1], 0),

    // @Param: 2_SCALING
    // @DisplayName: RPM scaling
    // @Description: Scaling factor between sensor reading and RPM.
    // @Increment: 0.001
    AP_GROUPINFO("2_SCALING", 11, AP_FlowSensor, _scaling[1], 1.0f),
#endif

    AP_GROUPEND
};

AP_FlowSensor::AP_FlowSensor(void) :
    num_instances(0)
{
    AP_Param::setup_object_defaults(this, var_info);

    // init state and drivers
    memset(state,0,sizeof(state));
    memset(drivers,0,sizeof(drivers));
}

/*
  initialise the AP_FlowSensor class. 
 */
void AP_FlowSensor::init(void)
{
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }
    for (uint8_t i=0; i<FLOWSENSOR_MAX_INSTANCES; i++) {
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4  || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
        uint8_t type = _type[num_instances];
        uint8_t instance = num_instances;

        if (type == FLOWSENSOR_TYPE_PX4_PWM) {
            state[instance].instance = instance;
            drivers[instance] = new AP_FlowSensor_PX4_PWM(*this, instance, state[instance]);
        }
#endif
        if (drivers[i] != NULL) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1;
        }
    }
}

/*
  update RPM state for all instances. This should be called by main loop
 */
void AP_FlowSensor::update(void)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != NULL) {
            if (_type[i] == FLOWSENSOR_TYPE_NONE) {
                // allow user to disable a RPM sensor at runtime
                continue;
            }
            drivers[i]->update();
        }
    }
}
    
/*
  check if an instance is healthy
 */
bool AP_FlowSensor::healthy(uint8_t instance) const
{
    if (instance >= num_instances) {
        return false;
    }
    // assume we get readings at at least 1Hz
    if (AP_HAL::millis() - state[instance].last_reading_ms > 1000) {
        return false;
    }
    return true;
}
