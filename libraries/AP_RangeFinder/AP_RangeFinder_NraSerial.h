// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"
#include <GCS_MAVLink/GCS.h>

// default slave address
#define NRA_START_ADDR       0xAA
#define NRA_TAIL_ADDR        0x55
#define NRA_TARGET_STATUS    0x70B
#define NRA_TARGET_INFO      0x70C
#define NRA_SENSOR_STATUS    0x60A

#define NRA_PKT_SIZE         0x0E
#define NRA_PKT_MESSAGE_SIZE 0x08
#define DATA_BUFFER_MAX      288

struct nra_radar_packeg_valid
{
    uint8_t message_id_high;
    uint8_t message_id_low;
    uint8_t data[NRA_PKT_MESSAGE_SIZE];
};

class AP_RangeFinder_NraSerial : public AP_RangeFinder_Backend
{

public:
    // constructor
    AP_RangeFinder_NraSerial(RangeFinder &ranger, uint8_t instance, RangeFinder::RangeFinder_State &_state,
                                   AP_SerialManager &serial_manager);

    // static detection function
    static bool detect(RangeFinder &ranger, uint8_t instance, AP_SerialManager &serial_manager);

    // update state
    void update(void);

private:
    // get a reading
    bool get_reading(uint16_t &reading_cm);

    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t last_reading_ms;

    uint8_t reading_snr;
};
