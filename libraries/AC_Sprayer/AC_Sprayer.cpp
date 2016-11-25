#include <AP_HAL/AP_HAL.h>
#include "AC_Sprayer.h"

extern const AP_HAL::HAL& hal;

// ------------------------------

const AP_Param::GroupInfo AC_Sprayer::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Sprayer enable/disable
    // @Description: Allows you to enable (1) or disable (0) the sprayer
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 0, AC_Sprayer, _enabled, 0, AP_PARAM_FLAG_ENABLE),

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

	// @Param: PUMP_TYPE
    // @DisplayName: sprayer pump type
    // @Description: sprayer pump type --1-DAISCH spinning pump 0,2-Diaphragm pump
    // @Units: 
    // @Range: 0 2
    // @User: Standard
    AP_GROUPINFO("PUMP_TYPE",   5, AC_Sprayer, _sprayer_pump_type, Pump_Type_Diaphragm),
	
	// @Param: DRAIN_DLY
    // @DisplayName: drain off delay
    // @Description: pre time for drain off detect
    // @Units: ms
    // @Range: 0 5000
    // @User: Standard
    AP_GROUPINFO("DRAIN_DLY",   6, AC_Sprayer, _drain_off_delay, 2000),

    AP_GROUPEND
};

AC_Sprayer::AC_Sprayer(const AP_InertialNav* inav, const AP_FlowSensor* flowsensor, const AP_AHRS_NavEKF* ahrs) :
    _inav(inav),
    _flowsensor(flowsensor),
    _ahrs(ahrs),
    _speed_over_min_time(0),
    _speed_under_min_time(0),
    _drain_off_pre_time(0)
{
    AP_Param::setup_object_defaults(this, var_info);

    _flags.drain_off = false;
    _flags.drain_off_precheck = false;
    // check for silly parameter values
    if (_pump_pct_1ms < 0.0f || _pump_pct_1ms > 100.0f) {
        _pump_pct_1ms.set_and_save(AC_SPRAYER_DEFAULT_PUMP_RATE);
    }
    if (_spinner_pwm < 0) {
        _spinner_pwm.set_and_save(AC_SPRAYER_DEFAULT_SPINNER_PWM);
    }

    // To-Do: ensure that the pump and spinner servo channels are enabled
}

void AC_Sprayer::run(const bool true_false)
{
    // return immediately if no change
    if (true_false == _flags.running) {
        return;
    }

    // set flag indicate whether spraying is permitted:
    // do not allow running to be set to true if we are currently not enabled
    _flags.running = true_false && _enabled;

    // turn off the pump and spinner servos if necessary
    if (!_flags.running) {
        stop_spraying();
    }
}

void AC_Sprayer::stop_spraying()
{
    // send output to pump channel
    RC_Channel_aux::set_radio_to_min(RC_Channel_aux::k_sprayer_pump);

    // send output to spinner channel
    RC_Channel_aux::set_radio_to_min(RC_Channel_aux::k_sprayer_spinner);

    _flags.spraying = false;
}

/// update - adjust pwm of servo controlling pump speed according to the desired quantity and our horizontal speed
void
AC_Sprayer::update()
{
    float vel_fwd_abs;
    // exit immediately if we are disabled or shouldn't be running
    if (!_enabled || !running()) {
		_drain_off_pre_time = 0;
        _flags.drain_off_precheck = false;
        run(false);
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
    const uint32_t now = AP_HAL::millis();
    //get spraying flow 
    const float flow = _flowsensor->get_flow(0);
	
    bool should_be_spraying = _flags.spraying;
    // check our speed vs the minimum
    if (vel_fwd_abs >= _speed_min) {
        // if we are not already spraying
        if (!_flags.spraying) {
            // set the timer if this is the first time we've surpassed the min speed
            if (_speed_over_min_time == 0) {
                _speed_over_min_time = now;
            }else{
                // check if we've been over the speed long enough to engage the sprayer
                if((now - _speed_over_min_time) > AC_SPRAYER_DEFAULT_TURN_ON_DELAY) {
                    should_be_spraying = true;
                    _speed_over_min_time = 0;
                }
            }
        }
        // reset the speed under timer
        _speed_under_min_time = 0;
    }else{
        // we are under the min speed.
        if (_flags.spraying) {
            // set the timer if this is the first time we've dropped below the min speed
            if (_speed_under_min_time == 0) {
                _speed_under_min_time = now;
            }else{
                // check if we've been over the speed long enough to engage the sprayer
                if((now - _speed_under_min_time) > AC_SPRAYER_DEFAULT_SHUT_OFF_DELAY) {
                    should_be_spraying = false;
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
    // if testing pump output speed as if travelling at 1m/s
    if (_flags.testing) {
        vel_fwd_abs = 100.0f;
        should_be_spraying = true;
    }

    // if spraying or testing update the pump servo position
    if (should_be_spraying) {
        switch (_sprayer_pump_type) {
            case Pump_Type_Spinner:
                RC_Channel_aux::set_radio(RC_Channel_aux::k_sprayer_pump, MIN((_pump_min_pct * 50 + 0.5 * (vel_fwd_abs - _speed_min) * _pump_pct_1ms), 5000));
                break;

            case Pump_Type_Diaphragm:
                RC_Channel_aux::set_radio(RC_Channel_aux::k_sprayer_pump, MIN((1000 + _pump_min_pct * 10 + 0.1 * (vel_fwd_abs - _speed_min) * _pump_pct_1ms), 2000));
                break;

            default:
                RC_Channel_aux::set_radio(RC_Channel_aux::k_sprayer_pump, MIN((1000 + _pump_min_pct * 10 + 0.1 * (vel_fwd_abs - _speed_min) * _pump_pct_1ms), 2000));
                break;
        }
		RC_Channel_aux::set_radio(RC_Channel_aux::k_sprayer_spinner, _spinner_pwm);
        _flags.spraying = true;
    }else{
        stop_spraying();
    }
}
