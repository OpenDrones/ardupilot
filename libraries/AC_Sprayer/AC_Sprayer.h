// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	AC_Sprayer.h
/// @brief	Crop sprayer library

/**
    The crop spraying functionality can be enabled in ArduCopter by doing the following:
        1. add this line to APM_Config.h
            #define SPRAYER ENABLED
        2. set CH7_OPT or CH8_OPT parameter to 15 to allow turning the sprayer on/off from one of these channels
        3. set RC10_FUNCTION to 22 to enable the servo output controlling the pump speed on RC10
        4. set RC11_FUNCTION to 23 to enable the spervo output controlling the spinner on RC11
        5. ensure the RC10_MIN, RC10_MAX, RC11_MIN, RC11_MAX accurately hold the min and maximum servo values you could possibly output to the pump and spinner
        6. set the SPRAY_SPINNER to the pwm value the spinner should spin at when on
        7. set the SPRAY_PUMP_RATE to the value the pump should servo should move to when the vehicle is travelling 1m/s expressed as a percentage (i.e. 0 ~ 100) of the full servo range.  I.e. 0 = the pump will not operate, 100 = maximum speed at 1m/s.  50 = 1/2 speed at 1m/s, full speed at 2m/s
        8. set the SPRAY_PUMP_MIN to the minimum value that the pump servo should move to while engaged expressed as a percentage (i.e. 0 ~ 100) of the full servo range
        9. set the SPRAY_SPEED_MIN to the minimum speed (in cm/s) the vehicle should be moving at before the pump and sprayer are turned on.  0 will mean the pump and spinner will always be on when the system is enabled with ch7/ch8 switch
**/

#ifndef AC_SPRAYER_H
#define AC_SPRAYER_H

#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_InertialNav/AP_InertialNav.h>     // Inertial Navigation library
#include <AP_FlowSensor/AP_FlowSensor.h>     // flow sensor library
#include <AP_AHRS/AP_AHRS_NavEKF.h>
#include <AP_Motors/AP_Motors.h>

#define AC_SPRAYER_DEFAULT_PUMP_RATE        10.0f   // default quantity of spray per meter travelled
#define AC_SPRAYER_DEFAULT_PUMP_MIN         40.0f       // default minimum pump speed expressed as a percentage from 0 to 100
#define AC_SPRAYER_DEFAULT_SPINNER_PWM      1300    // default speed of spinner (higher means spray is throw further horizontally
#define AC_SPRAYER_DEFAULT_SPEED_MIN        100     // we must be travelling at least 1m/s to begin spraying
#define AC_SPRAYER_DEFAULT_TURN_ON_DELAY    10      // delay between when we reach the minimum speed and we begin spraying.  This reduces the likelihood of constantly turning on/off the pump
#define AC_SPRAYER_DEFAULT_SHUT_OFF_DELAY   100     // shut-off delay in milli seconds.  This reduces the likelihood of constantly turning on/off the pump
#define AC_SPRAYER_DEFAULT_AREA             0       // spraying area
#define AC_SPRAYER_DEFAULT_WIDTH            500     // spraying width, cm

/// @class  AC_Sprayer
/// @brief  Object managing a crop sprayer comprised of a spinner and a pump both controlled by pwm
class AC_Sprayer {

public:

    /// Constructor
    AC_Sprayer(const AP_InertialNav* inav, const AP_FlowSensor* flowsensor, const AP_AHRS_NavEKF* ahrs, const AP_Motors* motors);

    // supported sprayer pump types
    enum SprayerPump_Type {
        Pump_Type_None = 0,
        Pump_Type_Spinner = 1,    // DAISCH Spinning pump control pwm range from 0 to 5000
        Pump_Type_Diaphragm = 2   // diaphragm pump  control pwm range from 1000 to 2000
    };

    void init();

    /// enable - allows sprayer to be enabled/disabled.  Note: this does not update the eeprom saved value
    void enable(bool true_false);

    /// enabled - returns true if sprayer is enabled
    bool enabled() const { return _enabled; }

    /// enabled - returns true if drain off detected
    bool get_drain_off() const { return _flags.drain_off; }

    /// test_pump - set to true to turn on pump as if travelling at 1m/s as a test
    void test_pump(bool true_false) { _flags.testing = true_false; }

    /// To-Do: add function to decode pilot input from channel 6 tuning knob

    /// set_pump_rate - sets desired quantity of spray when travelling at 1m/s as a percentage of the pumps maximum rate
    void set_pump_rate(float pct_at_1ms) { _pump_pct_1ms.set(pct_at_1ms); }

    float get_spray_area() const { return _current_total_area; }

    /// update - adjusts servo positions based on speed and requested quantity
    void update();

    static const struct AP_Param::GroupInfo var_info[]; 

private:
    // pointers to other objects we depend upon
    const AP_InertialNav* const _inav;
    const AP_FlowSensor* const _flowsensor;
    const AP_AHRS_NavEKF* const _ahrs;
    const AP_Motors* const _motors;

    // parameters
    AP_Int8         _enabled;               // top level enable/disable control
    AP_Float        _pump_pct_1ms;          // desired pump rate (expressed as a percentage of top rate) when travelling at 1m/s
    AP_Int8         _pump_min_pct;          // minimum pump rate (expressed as a percentage from 0 to 100)
    AP_Int16        _spinner_pwm;           // pwm rate of spinner
    AP_Float        _speed_min;             // minimum speed in cm/s above which the sprayer will be started
    AP_Int16        _turn_on_delay;
    AP_Int16        _shut_off_delay;
    AP_Int16        _width;
    AP_Float        _area;
    AP_Int16        _drain_off_delay;
    AP_Int8         _sprayer_pump_type;     // sprayer pump type -- 1-DAISCH spinning pump 2-Diaphragm pump

    // flag bitmask
    struct sprayer_flags_type {
        uint8_t spraying                : 1;            // true if we are currently spraying
        uint8_t testing                 : 1;            // true if we are testing the sprayer and should output a minimum value
        uint8_t drain_off               : 1;            // true if fluid drain off
        uint8_t drain_off_precheck      : 1;            // true if drain off pre check is ok
    } _flags;

    // internal variables
    uint32_t        _speed_over_min_time;   // time at which we reached speed minimum
    uint32_t        _speed_under_min_time;  // time at which we fell below speed minimum
    uint32_t        _drain_off_pre_time;  // time at which we fell below speed minimum
    uint32_t        _spraying_last_time;    // last time at which record system time
    bool            _armed;                 // moters arm status
    float           _current_total_area;    // current total spraying area

};

#endif /* AC_SPRAYER_H */
