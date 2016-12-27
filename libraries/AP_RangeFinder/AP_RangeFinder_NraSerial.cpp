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

#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_NraSerial.h"
#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_NraSerial::AP_RangeFinder_NraSerial(RangeFinder &_ranger, uint8_t instance,
                                                               RangeFinder::RangeFinder_State &_state,
                                                               AP_SerialManager &serial_manager) :
    AP_RangeFinder_Backend(_ranger, instance, _state)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Lidar, 0));
    }
}

/*
   detect if a NraSerial rangefinder is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool AP_RangeFinder_NraSerial::detect(RangeFinder &_ranger, uint8_t instance, AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar, 0) != nullptr;
}

// read - return last value measured by sensor
bool AP_RangeFinder_NraSerial::get_reading(uint16_t &reading_cm)
{
    if (uart == nullptr) {
        return false;
    }

    // parse a response message
    uint8_t data_buffer[DATA_BUFFER_MAX] = {0};
    uint32_t start_ms = hal.scheduler->millis();
    uint32_t len = 0;

    // read serial
    while (hal.scheduler->millis() - start_ms < 20) {
        uint32_t nbytes = uart->available();
        if (nbytes != 0) {
            for (int i=len; i < nbytes+len; i++) {
                if (i >= DATA_BUFFER_MAX) {
                    return false;
                }
                data_buffer[i] = uart->read();
            }
            len += nbytes;
        }
    }
    if (len > DATA_BUFFER_MAX) {
      return false;
    }
    nra_radar_packeg_valid radar_packeg_valid[20];
    uint8_t radar_packeg_valid_number = 0;
    for (int i = 0; i < DATA_BUFFER_MAX; ++i)
    {
      if (data_buffer[i] == NRA_START_ADDR && data_buffer[i+1] == NRA_START_ADDR 
        && data_buffer[i+12] == NRA_TAIL_ADDR && data_buffer[i+13] == NRA_TAIL_ADDR)
      {
        for (int j = 0; j < NRA_PKT_MESSAGE_SIZE; ++j)
        {
          radar_packeg_valid[radar_packeg_valid_number].data[j] = data_buffer[i+4+j];
        }
        radar_packeg_valid[radar_packeg_valid_number].message_id_low = data_buffer[i+2];
        radar_packeg_valid[radar_packeg_valid_number].message_id_high = data_buffer[i+3];
        radar_packeg_valid_number++;
        i = i + NRA_PKT_SIZE;
      }
    }

    if (radar_packeg_valid_number == 0) {
      return false;
    }

    uint32_t sum = 0;
    uint8_t count = 0;
    for (int i = 0; i < radar_packeg_valid_number; ++i)
    {
      uint16_t message_id = radar_packeg_valid[i].message_id_high * 256 + radar_packeg_valid[i].message_id_low;
      switch(message_id) {
        case NRA_TARGET_INFO:
        sum += radar_packeg_valid[i].data[2] * 256 + radar_packeg_valid[i].data[3];
        count++;
        break;

        default:
        break;
      }
    }
    if (sum == 0 || count == 0) {
      return false;
    }

    reading_cm = sum/count;
    return true;
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_NraSerial::update(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        last_reading_ms = hal.scheduler->millis();
        update_status();
    } else if (hal.scheduler->millis() - last_reading_ms > 200) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}