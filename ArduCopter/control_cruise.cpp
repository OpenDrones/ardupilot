/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

#if CRUISE_ENABLED == ENABLED

#define CRUISE_CURR_SPEED_MIN_DEFAULT   100.0f   // speed threshold to judge cruise forward or back
#define CRUISE_PILOT_SPEED_MAX_DEFAULT  1000.0f  // max speed that pilot can control in cruise
#define PILOT_ROLL_PITCH_THRESHOLD      300.0f   // threshold pilot pitch or roll to recalculate initial cruise velocity

static struct {
    // cruise PID controller
    uint8_t cruise_reset_I;                   // true the very first time to run cruise PID controller

    // cruise desired forward and right velocity without pilot input
    float vel_fwd;
    float vel_rgt;

    // cruise desired forward and right velocity with pilot input
    float target_vel_fwd;
    float target_vel_rgt;

    // cruise desired velocity NE
    Vector2f target_vel_ef;

    // final output
    int16_t roll;   // final roll angle sent to attitude controller
    int16_t pitch;  // final pitch angle sent to attitude controller
} cruise;

// cruise_init - initialise cruise controller
bool Copter::cruise_init(bool ignore_checks)
{
    if (position_ok() || ignore_checks) {

        // initialize vertical speed and acceleration
        pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
        pos_control.set_accel_z(g.pilot_accel_z);

        // initialise target position to stopping point
        pos_control.set_target_to_stopping_point_xy();
        pos_control.set_target_to_stopping_point_z();

        // init_xy_controller - initialise the xy controller
        pos_control.init_xy_controller();

        init_cruise_vel(cruise.vel_fwd, cruise.vel_rgt);


        return true;
    }else{
        return false;
    }
}

// cruise_run - runs the cruise controller
// should be called at 100hz or more

void Copter::cruise_run()
{
    float target_roll, target_pitch;  // pilot's roll and pitch angle inputs
    float target_yaw_rate = 0;          // pilot desired yaw rate in centi-degrees/sec
    float target_climb_rate = 0;      // pilot desired climb rate in centimeters/sec
    float takeoff_climb_rate = 0.0f;    // takeoff induced climb rate

    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if(!ap.auto_armed || !motors.get_interlock()) {
        wp_nav.init_loiter_target();
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(channel_throttle->control_in)-throttle_average);
        return;
    }

    // process pilot inputs
    if (!failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);

        // get pilot desired climb rate (for alt-hold mode and take-off)
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->control_in);
        target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

        // get takeoff adjusted pilot and takeoff climb rates
        takeoff_get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // check for take-off
        if (ap.land_complete && (takeoff_state.running || channel_throttle->control_in > get_takeoff_trigger_throttle())) {
            if (!takeoff_state.running) {
                takeoff_timer_start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            }

            // indicate we are taking off
            set_land_complete(false);
            // clear i term when we're taking off
            set_throttle_takeoff();
        }
    }

    // relax loiter target if we might be landed
    if (ap.land_complete_maybe) {
        wp_nav.loiter_soften_for_landing();
    }

    // if landed initialise loiter targets, set throttle to zero and exit
    if (ap.land_complete) {
        wp_nav.init_loiter_target();
        // move throttle to between minimum and non-takeoff-throttle to keep us on the ground
        attitude_control.set_throttle_out_unstabilized(get_throttle_pre_takeoff(channel_throttle->control_in),true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(channel_throttle->control_in)-throttle_average);
        return;
    }else{
        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(channel_roll->control_in, channel_pitch->control_in, target_roll, target_pitch);
            
        // calc target cruise velocity in body-frame according to pilot input roll and pitch
        get_pilot_cruise_vel(cruise.target_vel_fwd, cruise.vel_fwd, -target_pitch);
        get_pilot_cruise_vel(cruise.target_vel_rgt, cruise.vel_rgt, target_roll);

        // convert desired velocity from body frame reference to earth frame reference
        cruise.target_vel_ef.x = cruise.target_vel_fwd;
        cruise.target_vel_ef.y = cruise.target_vel_rgt;
        rotate_body_frame_to_NE(cruise.target_vel_ef.x, cruise.target_vel_ef.y);

        // set desired velocity
        pos_control.set_desired_velocity_xy(cruise.target_vel_ef.x, cruise.target_vel_ef.y);

        // calculate dt
        float dt = pos_control.time_since_last_xy_update();

        // update at poscontrol update rate
        if (dt >= pos_control.get_dt_xy()) {
            // sanity check dt
            if (dt >= 0.2f) {
                dt = 0.0f;
            }

            // call velocity controller which includes xy axis controller
            pos_control.update_xy_controller(AC_PosControl::XY_MODE_POS_AND_VEL_FF,ekfNavVelGainScaler);
        }
        
        // constrain target pitch/roll angles
        cruise.roll = constrain_int16(pos_control.get_roll(), -aparm.angle_max, aparm.angle_max);
        cruise.pitch = constrain_int16(pos_control.get_pitch(), -aparm.angle_max, aparm.angle_max);

        // update attitude controller targets
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(cruise.roll, cruise.pitch, target_yaw_rate);

        // throttle control
        if (sonar_enabled) {
            // if sonar is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
        }
        // update altitude target and call position controller
        pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control.add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control.update_z_controller();
    }
}

// calc initial cruise velocity in body-frame
void Copter::init_cruise_vel(float &cruise_vel_fwd, float &cruise_vel_rgt)
{
    float vel_fwd, vel_rgt;
    const Vector3f& curr_vel = inertial_nav.get_velocity();

    // convert inertial nav earth-frame velocities_cm/s to body-frame
    vel_fwd = curr_vel.x*ahrs.cos_yaw() + curr_vel.y*ahrs.sin_yaw();
    vel_rgt = -curr_vel.x*ahrs.sin_yaw() + curr_vel.y*ahrs.cos_yaw();
    
    // if unreasonable cruise parameter or copter fly to right or left in body-frame now, make cruise velocity equals to 0, to confirm only fly forward and back in cruise

    if (g.cruise_speed < 0 || g.cruise_speed > 1000 || fabsf(vel_rgt) > CRUISE_CURR_SPEED_MIN_DEFAULT) {
        cruise_vel_fwd = 0;
        cruise_vel_rgt = 0;
        return;
    }

        if (vel_fwd < -CRUISE_CURR_SPEED_MIN_DEFAULT) {
            cruise_vel_fwd = -g.cruise_speed;
        } else {
            cruise_vel_fwd = g.cruise_speed;
        }

    cruise_vel_rgt = 0;
}

// calc target cruise velocity in body-frame according to pilot input roll and pitch
void Copter::get_pilot_cruise_vel(float &target_vel, float vel, float pilot_in)
{
    float angle_max = constrain_float(aparm.angle_max,1000,8000);
    target_vel = vel + CRUISE_PILOT_SPEED_MAX_DEFAULT*pilot_in/angle_max;
    target_vel = constrain_int16(target_vel, -1500, 1500);
}

#endif  // CRUISE_ENABLED == ENABLED