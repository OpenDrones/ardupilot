// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>
#include "AC_Sprayer.h"

extern const AP_HAL::HAL& hal;

// ------------------------------

const AP_Param::GroupInfo AC_Sprayer::var_info[] PROGMEM = {
    // @Param: ENABLE
    // @DisplayName: Sprayer enable/disable
    // @Description: Allows you to enable (1) or disable (0) the sprayer
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLE",      0,  AC_Sprayer, _enabled, 0),

    // @Param: PUMP_RATE
    // @DisplayName: Pump speed
    // @Description: Desired pump speed when travelling 1m/s expressed as a percentage
    // @Units: percentage
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("PUMP_RATE",   1, AC_Sprayer, _pump_pct_1ms, AC_SPRAYER_DEFAULT_PUMP_RATE),

    // @Param: SPINNER
    // @DisplayName: Spinner rotation speed
    // @Description: Spinner's rotation speed in PWM (a higher rate will disperse the spray over a wider area horizontally)
    // @Units: ms
    // @Range: 1000 2000
    // @User: Standard
    AP_GROUPINFO("SPINNER",     2, AC_Sprayer, _spinner_pwm, AC_SPRAYER_DEFAULT_SPINNER_PWM),

    // @Param: SPEED_MIN
    // @DisplayName: Speed minimum
    // @Description: Speed minimum at which we will begin spraying
    // @Units: cm/s
    // @Range: 0 1000
    // @User: Standard
    AP_GROUPINFO("SPEED_MIN",   3, AC_Sprayer, _speed_min, AC_SPRAYER_DEFAULT_SPEED_MIN),

    // @Param: PUMP_MIN
    // @DisplayName: Pump speed minimum
    // @Description: Minimum pump speed expressed as a percentage
    // @Units: percentage
    // @Range: 0 100
    // @User: Standard
    AP_GROUPINFO("PUMP_MIN",   4, AC_Sprayer, _pump_min_pct, AC_SPRAYER_DEFAULT_PUMP_MIN),

    // @Param: TON_DLY
    // @DisplayName: Turn on delay
    // @Description: delay between when we reach the minimum speed and we begin spraying.
    // @Units: ms
    // @Range: 0 5000
    // @User: Standard
    AP_GROUPINFO("TON_DLY",   5, AC_Sprayer, _turn_on_delay, AC_SPRAYER_DEFAULT_TURN_ON_DELAY),

    // @Param: SOFF_DLY
    // @DisplayName: shut off  delay
    // @Description: delay between when we shut off spraying.
    // @Units: ms
    // @Range: 0 5000
    // @User: Standard
    AP_GROUPINFO("SOFF_DLY",   6, AC_Sprayer, _shut_off_delay, AC_SPRAYER_DEFAULT_SHUT_OFF_DELAY),

    // @Param: WIDTH
    // @DisplayName: spray width
    // @Description: spray width
    // @Units: cm
    // @Range: 0 10000
    // @User: Standard
    AP_GROUPINFO("WIDTH",   7, AC_Sprayer, _width, AC_SPRAYER_DEFAULT_WIDTH),

    // @Param: AREA
    // @DisplayName: spray area
    // @Description: accumulative total spray area
    // @Units: mu
    // @Range: 0 500000.00
    // @User: Standard
    AP_GROUPINFO("AREA",   8, AC_Sprayer, _area, AC_SPRAYER_DEFAULT_AREA),

    // @Param: DRAIN_DLY
    // @DisplayName: drain off delay
    // @Description: pre time for drain off detect
    // @Units: ms
    // @Range: 0 5000
    // @User: Standard
    AP_GROUPINFO("DRAIN_DLY",   9, AC_Sprayer, _drain_off_delay, 2000),

    // @Param: PUMP_TYPE
    // @DisplayName: sprayer pump type
    // @Description: sprayer pump type --1-DAISCH spinning pump 2-Diaphragm pump
    // @Units: 
    // @Range: 0 2
    // @User: Standard
    AP_GROUPINFO("PUMP_TYPE",   10, AC_Sprayer, _sprayer_pump_type, Pump_Type_Diaphragm),

    AP_GROUPEND
};

AC_Sprayer::AC_Sprayer(const AP_InertialNav* inav, const AP_FlowSensor* flowsensor, const AP_AHRS_NavEKF* ahrs, const AP_Motors* motors) :
    _inav(inav),
    _flowsensor(flowsensor),
    _ahrs(ahrs),
    _motors(motors),
    _speed_over_min_time(0),
    _speed_under_min_time(0),
    _drain_off_pre_time(0),
    _spraying_last_time(0),
    _armed(false)

{   
    AP_Param::setup_object_defaults(this, var_info);


    // initialise flags
    _flags.spraying = false;
    _flags.testing = false;
    _flags.drain_off = false;
    _flags.drain_off_precheck = false;

    // To-Do: ensure that the pump and spinner servo channels are enabled
}

void 
AC_Sprayer::init()
{
    // check for silly parameter values
    if (_pump_pct_1ms < 0.0f || _pump_pct_1ms > 100.0f) {
        _pump_pct_1ms.set_and_save(AC_SPRAYER_DEFAULT_PUMP_RATE);
    }
    if (_spinner_pwm < 0) {
        _spinner_pwm.set_and_save(AC_SPRAYER_DEFAULT_SPINNER_PWM);
    }
    
    _current_total_area = _area;
}

void 
AC_Sprayer::enable(bool true_false)
{
    // return immediately if no change
    if (true_false == _enabled) {
        return;
    }

    // set enabled/disabled parameter (in memory only)
    _enabled = true_false;

    // turn off the pump and spinner servos if necessary
    if (!_enabled) {
        // send output to pump channel
        // To-Do: change 0 below to radio_min of pump servo
        RC_Channel_aux::set_radio_to_min(RC_Channel_aux::k_sprayer_pump);

        // send output to spinner channel
        // To-Do: change 0 below to radio_min of spinner servo
        RC_Channel_aux::set_radio_to_min(RC_Channel_aux::k_sprayer_spinner);

        _spraying_last_time = 0;
    }
}

/// update - adjust pwm of servo controlling pump speed according to the desired quantity and our horizontal speed
void
AC_Sprayer::update()
{
    uint32_t now;
    float vel_fwd_abs;
    
    if ( (_motors->armed() != _armed) && (_motors->armed() == false) ) {
        _area.set_and_save(_current_total_area);  
    }
    _armed = _motors->armed();

    // exit immediately if we are disabled (perhaps set pwm values back to defaults)
    if (!_enabled) {
        _drain_off_pre_time = 0;
        _flags.drain_off_precheck = false;
        // ensure sprayer and spinner are off
        RC_Channel_aux::set_radio_to_min(RC_Channel_aux::k_sprayer_pump);
        RC_Channel_aux::set_radio_to_min(RC_Channel_aux::k_sprayer_spinner);
        return;
    }


    // exit immediately if the pump function has not been set-up for any servo
    if (!RC_Channel_aux::function_assigned(RC_Channel_aux::k_sprayer_pump)) {
        return;
    }

    // get horizontal velocity
    const Vector3f &velocity = _inav->get_velocity();

    // convert inertial nav earth-frame velocities_cm/s to body-frame
    vel_fwd_abs = abs(velocity.x * _ahrs->cos_yaw() + velocity.y * _ahrs->sin_yaw());

    // get the current time
    now = hal.scheduler->millis();

    //get spraying flow 
    const float flow = _flowsensor->get_flow(0);

    // check our speed vs the minimum
    if (vel_fwd_abs >= _speed_min) {
        // if we are not already spraying
        if (!_flags.spraying) {
            // set the timer if this is the first time we've surpassed the min speed
            if (_speed_over_min_time == 0) {
                _speed_over_min_time = now;
            }else{
                // check if we've been over the speed long enough to engage the sprayer
                if((now - _speed_over_min_time) > (uint32_t)_turn_on_delay) {
                    _flags.spraying = true;
                    _speed_over_min_time = 0;
                }
            }
        }
        // reset the speed under timer
        _speed_under_min_time = 0;
    }else{
        // we are under the min speed.  If we are spraying
        if (_flags.spraying) {
            // set the timer if this is the first time we've dropped below the min speed
            if (_speed_under_min_time == 0) {
                _speed_under_min_time = now;
            }else{
                // check if we've been over the speed long enough to engage the sprayer
                if((now - _speed_under_min_time) > (uint32_t)_shut_off_delay) {
                    _flags.spraying = false;
                    _speed_under_min_time = 0;
                }
            }
        }
        // reset the speed over timer
        _speed_over_min_time = 0;
    }

    // detect drain off
    if (_flags.spraying) {
        if ( flow == -1 && _flags.drain_off_precheck == true) {
            _flags.drain_off = true;
            _flags.drain_off_precheck = false;
            _drain_off_pre_time = 0;           
        } else {

            _flags.drain_off = false;
            if (flow > 0)
            {
                if (_drain_off_pre_time == 0)
                    _drain_off_pre_time = now;
                else if ((now - _drain_off_pre_time) > (uint32_t)_drain_off_delay) {
                            _flags.drain_off_precheck = true;
                        }
            } else {
                _drain_off_pre_time = 0;
            }
        }
    }
    else {
        _drain_off_pre_time = 0;
        _flags.drain_off = false;
        _flags.drain_off_precheck = false;
    }

    // calculate spray area
    if (_flags.spraying) {
        if (_spraying_last_time == 0) {
            _spraying_last_time = now;
        } else {
            _current_total_area += ( vel_fwd_abs * (float)(now - _spraying_last_time) * (float)_width )/ (1.0e7f * 666.7f);
            _spraying_last_time = now;
        }
    } else {
        _spraying_last_time = 0;
    }


    // if testing pump output speed as if travelling at 1m/s
    if (_flags.testing) {
        vel_fwd_abs = 100.0f;
    }

    // if spraying or testing update the pump rate percent, for radio direct pwm control（200Hz）, 100% duty -> 5000us
    if (_flags.spraying || _flags.testing) {
        switch (_sprayer_pump_type) {
            case Pump_Type_Spinner:
                RC_Channel_aux::set_radio(RC_Channel_aux::k_sprayer_pump, min((_pump_min_pct * 50 + 0.5 * (vel_fwd_abs - _speed_min) * _pump_pct_1ms), 5000));
                break;

            case Pump_Type_Diaphragm:
                RC_Channel_aux::set_radio(RC_Channel_aux::k_sprayer_pump, min((1000 + _pump_min_pct * 10 + 0.1 * (vel_fwd_abs - _speed_min) * _pump_pct_1ms), 2000));
                break;

            default:
                RC_Channel_aux::set_radio(RC_Channel_aux::k_sprayer_pump, min((1000 + _pump_min_pct * 10 + 0.1 * (vel_fwd_abs - _speed_min) * _pump_pct_1ms), 2000));
                break;
        }
        RC_Channel_aux::set_radio(RC_Channel_aux::k_sprayer_spinner, _spinner_pwm);
    }else{
        // ensure sprayer and spinner are off
        RC_Channel_aux::set_radio_to_min(RC_Channel_aux::k_sprayer_pump);
        RC_Channel_aux::set_radio_to_min(RC_Channel_aux::k_sprayer_spinner);
    }
}
