/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

#if CRUISE_ENABLED == ENABLED

#define CRUISE_CURR_SPEED_MIN_DEFAULT       50.0f       // speed threshold to judge cruise forward or back
#define CRUISE_DESTINATION_DISTANCE_CM      5000.0f     // distance to calc next cruise destination
#define DISTANCE_TO_CALC_NEXT_CRUISE_DES_CM 1000.0f     // min distance to judge need to calc next cruise distination
#define PITCH_IN_SET_CRUISE_DIRECTION_THRESHOLD 0.5f

static struct {
    bool flag_reach_cruise_des;
    bool flag_init_cruise_des;
     // get current bearing when changing into cruise mode
    float cos_bearing, sin_bearing;
    // cruise destination
    Vector3f destination;
} cruise;

// cruise_init - initialise cruise controller
bool Copter::cruise_init()
{
    if (position_ok()) {
        const Vector3f& curr_vel = inertial_nav.get_velocity();
        pos_control.set_desired_velocity_xy(curr_vel.x,curr_vel.y);
        // initialise waypoint and spline controller
        wp_nav.wp_and_spline_init();

        // initialise target position to stopping point
        pos_control.set_target_to_stopping_point_xy();
        pos_control.set_target_to_stopping_point_z();

        // init cruise flags
        cruise.flag_init_cruise_des = true;
        cruise.flag_reach_cruise_des = false;
        return true;
    }else{
        return false;
    }
}

// cruise_run - runs the cruise controller
// should be called at 100hz or more

void Copter::cruise_run()
{
    float target_yaw_rate = 0;          // pilot desired yaw rate in centi-degrees/sec
    float target_climb_rate = 0;      // pilot desired climb rate in centimeters/sec

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
    } else {
        // calc cruise direction forward or backward
        if (cruise.flag_init_cruise_des) {
            // get pilot desired lean angles
            float target_roll, target_pitch;
            get_pilot_desired_lean_angles(channel_roll->control_in, channel_pitch->control_in, target_roll, target_pitch);
            // calc direction according to pilot inputs 
            if (target_pitch > aparm.angle_max * PITCH_IN_SET_CRUISE_DIRECTION_THRESHOLD) {
                cruise.cos_bearing = -ahrs.cos_yaw();
                cruise.sin_bearing = -ahrs.sin_yaw();
            } else if (target_pitch < -aparm.angle_max * PITCH_IN_SET_CRUISE_DIRECTION_THRESHOLD) {
                    cruise.cos_bearing = ahrs.cos_yaw();
                    cruise.sin_bearing = ahrs.sin_yaw();
                } else {
                    // run loiter controller
                    wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);
                    // update altitude target and call position controller
                    pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);

                    // call attitude controller
                    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
                    // call z-axis position controller
                    pos_control.update_z_controller();
                    return;
                }
        }
        // calc next cruise destination
        if (cruise.flag_init_cruise_des || cruise.flag_reach_cruise_des) {
            // calc next cruise destination
            update_cruise_des(cruise.destination);
            // set destination
            wp_nav.set_wp_xy_origin_and_destination(cruise.destination);
            
            cruise.flag_init_cruise_des = false;
        }
        // calc flag to reach next cruise destination
        cruise.flag_reach_cruise_des = reach_cruise_des(cruise.destination);

        // run waypoint controller
        wp_nav.update_wpnav_xy();
        // altitude control according to sonar
        if (sonar_enabled) {
            // if sonar is ok, use surface tracking
            target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
        }
        // update altitude target and call position controller
        pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);   
        // call z-axis position controller
        pos_control.update_z_controller();
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
    }
}

// update cruise destination
void Copter::update_cruise_des(Vector3f& destination)
{
    const Vector3f &curr_pos = inertial_nav.get_position();
    destination.x = curr_pos.x + CRUISE_DESTINATION_DISTANCE_CM * cruise.cos_bearing;
    destination.y = curr_pos.y + CRUISE_DESTINATION_DISTANCE_CM * cruise.sin_bearing;
    destination.z = curr_pos.z;
}

// flag to reach cruise destination
bool Copter::reach_cruise_des(Vector3f& destination)
{
    const Vector3f &curr_pos = inertial_nav.get_position();
    float distance_target;
    distance_target = pv_get_horizontal_distance_cm(destination, curr_pos);
    return distance_target < DISTANCE_TO_CALC_NEXT_CRUISE_DES_CM;
}

#endif  // CRUISE_ENABLED == ENABLED