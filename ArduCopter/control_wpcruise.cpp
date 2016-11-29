/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

#if WPCRUISE_ENABLED == ENABLED

#define ROLLIN_SET_WPCRUISE_DIRECTION_THRESHOLD 0.5f

static struct {
    // flag to set destination
    uint8_t flag_init_destination               : 1;                    // flag to initialize first destination when changing into WPCRUISE
    
} wpcruise;

// wpcruise_init - initialise Waypoint Cruise controller
bool Copter::wpcruise_init(bool ignore_checks)
{
    // fail to initialise Waypoint Cruise mode if no GPS lock or AB points not exist
    if ((position_ok() || ignore_checks) && (mission.num_commands() ==2 || mission.num_commands() == 3 || mission.num_commands() == 4)) {
    
        // initialise waypoint and spline controller
        wp_nav.wp_and_spline_init();
        // calc current position
        const Vector3f& curr_pos = inertial_nav.get_position();
        // set target position
        pos_control.set_xy_target(curr_pos.x, curr_pos.y);
        pos_control.set_alt_target(inertial_nav.get_altitude());
        // set the waypoint as "fast"
        wp_nav.set_fast_waypoint(true);

        if (mission.num_commands() == 2) {
            // return to breakpoint and loiter in mannual flight mode
            WpCruise_state = Return_Bp_Loiter;
         } else if (mission.num_commands() == 3) {
                // breakpoint don't exist, start AB cruise directly
                WpCruise_state = Waypoint_Nav;
            }  else {
                // breakpoint exist in WpCruise mode, return to breakpoint first and then continue AB cruise
                WpCruise_state = Return_Bp_Wp_Nav;
            }
        
        // set flag to init first destination
        wpcruise.flag_init_destination = true;
        return true;
    } else {
        return false;
    }
}

// wpcruise_run - runs the Waypoint Cruise controller
// should be called at 100hz or more
void Copter::wpcruise_run()
{
    float target_yaw_rate = 0;          // pilot desired yaw rate in centi-degrees/sec
    float target_climb_rate = 0;      // pilot desired climb rate in centimeters/sec

    // if not auto armed or motor interlock not enabled set throttle to zero and exit immediately
    if (!motors.armed() || !ap.auto_armed || !motors.get_interlock()) {
        motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        wp_nav.init_loiter_target();
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(0.0f);
        return;
    }

    // process pilot inputs
    if (!failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

        // get pilot desired climb rate (for alt-hold mode and take-off)
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

        // check for take-off
        if (ap.land_complete && (takeoff_state.running || target_climb_rate > 0.0f)) {
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
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            motors.set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }
        wp_nav.init_loiter_target();
        attitude_control.reset_rate_controller_I_terms();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0, get_smoothing_gain());
        pos_control.relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        pos_control.update_z_controller();
        return;
    } else {
            if (wpcruise.flag_init_destination && (WpCruise_state != Return_Bp_Loiter) && flag_recalc_wp_offset_direction)
            {
                // get pilot desired lean angles
                float target_roll, target_pitch;
                get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control.get_althold_lean_angle_max());
                // 
                if (target_roll > aparm.angle_max * ROLLIN_SET_WPCRUISE_DIRECTION_THRESHOLD) {
                    mission.set_wp_direction(1);
                } else if (target_roll < -aparm.angle_max * ROLLIN_SET_WPCRUISE_DIRECTION_THRESHOLD)
                {
                    mission.set_wp_direction(-1);
                } else {
                    // run loiter controller
                    wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);
                    // update altitude target and call position controller
                    pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);

                    // call attitude controller
                    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate, get_smoothing_gain());
                    // call z-axis position controller
                    pos_control.update_z_controller();
                    return;
                }
            }

            // calc destination position step by step
            if (wpcruise.flag_init_destination || (wp_nav.reached_wp_destination() && WpCruise_state != Return_Bp_Loiter))
            {
                Vector3f destination;
                switch(WpCruise_state) {
                    // return to breakpoint and loiter in mannual flight mode
                    case Return_Bp_Loiter:
                    calc_breakpoint_destination(destination);
                    break;
                    
                    // change state of waypoint cruise and delete breakpoint from storage
                    case Return_Bp_Wp_Nav:
                    // calculate break point position
                    calc_breakpoint_destination(destination);
                    WpCruise_state = Waypoint_Nav;
                    break;
                    
                    // update waypoint nav destination
                    case Waypoint_Nav:
                    if (mission.num_commands() == 4) {
                        mission.truncate(3);
                    }
                    update_waypoint_destination(destination);
                    break;
                    default:
                    // do nothing
                    break;
                }
                if (WpCruise_state != Wpcruise_loiter) {
                    // set destination
                    wp_nav.set_wp_xy_origin_and_destination(destination);
                }
                wpcruise.flag_init_destination = false;
            }
            // loiter and add way-point when flow break or low battery
            if (WpCruise_state != Wpcruise_loiter && (sprayer.get_drain_off() || failsafe.battery)) {
                if (WpCruise_state == Waypoint_Nav) {
                    // save current waypoint position
                    save_add_waypoint();
                }
                // set wpcruise state
                WpCruise_state = Wpcruise_loiter;
                // calc stopping point as destination
                Vector3f destination;
                pos_control.get_stopping_point_xy(destination);
                pos_control.get_stopping_point_z(destination);
                
                // set destination
                wp_nav.set_wp_xy_origin_and_destination(destination);
            }
            // run xy position controller of waypoint
            wp_nav.update_wpnav_xy();
            // call attitude controller
            attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate, get_smoothing_gain());


            // altitude control according to sonar
            if (rangefinder_alt_ok()) {
                // if sonar is ok, use surface tracking
                target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
            }
            // update altitude target and call position controller
            pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
            // call z-axis position controller
            pos_control.update_z_controller();
        }

}
// initilate first destination position during auto cruise
void Copter::calc_breakpoint_destination(Vector3f& destination)
{
    // read break point from storage
    AP_Mission::Mission_Command cmd;
    if (mission.read_cmd_from_storage((mission.num_commands()-1), cmd)) {
        // position llh to vector xyz
        destination = pv_location_to_vector(cmd.content.location);
    }
}
 void Copter::update_waypoint_destination(Vector3f& destination)
 {
    Location destination_loc;
    mission.update_wpcruise_target(destination_loc);
    // position llh to vector xyz
    destination = pv_location_to_vector(destination_loc);
 }
#endif   // WPCRUISE_ENABLED == ENABLED
