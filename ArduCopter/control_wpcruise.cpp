/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

#if WPCRUISE_ENABLED == ENABLED

#define ROLLIN_SET_WPCRUISE_DIRECTION_THRESHOLD 0.5f

enum state_wp_cruise {
    RETURN_TO_BREAKPOINT = 0,
    WAYPOINT_NAV
    };
static struct {
    // flag to set destination
    uint8_t flag_reach_destination_old;               // old flag of reaching destination
    uint8_t flag_init_destination;                    // flag to initialize first destination when changing into WPCRUISE
    state_wp_cruise state;
} wpcruise;

// wpcruise_init - initialise Waypoint Cruise controller
bool Copter::wpcruise_init()
{
    // fail to initialise Waypoint Cruise mode if no GPS lock or AB points not exist
    if (position_ok() && (mission.num_commands() == 3 || mission.num_commands() == 4)) {
    
        // initialise waypoint and spline controller
        wp_nav.wp_and_spline_init();
        // calc current position
        const Vector3f& curr_pos = inertial_nav.get_position();
        // set target position
        pos_control.set_xy_target(curr_pos.x, curr_pos.y);
        // set the waypoint as "fast"
        wp_nav.set_fast_waypoint(true);

        // init old flag
        wpcruise.flag_reach_destination_old = true;

        Vector3f destination;
        // calc destination according to number of commands, if break point exists, number of commands equal to 3, if not, equal to 2
        if (mission.num_commands() == 4) {
            //set waypoint cruise state as "return to break point"
            wpcruise.state = RETURN_TO_BREAKPOINT;
        } else { 
            // set state as "waypoint nav"
            wpcruise.state = WAYPOINT_NAV;
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
    if(!ap.auto_armed || !motors.get_interlock()) {
        wp_nav.init_loiter_target();
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        pos_control.relax_alt_hold_controllers(get_throttle_pre_takeoff(channel_throttle->control_in)-throttle_average);
        return;
    }

    // process pilot inputs
    if (!failsafe.radio) {

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
    }else{
            if (!failsafe.radio && wpcruise.flag_init_destination && (wpcruise.state != RETURN_TO_BREAKPOINT) && flag_recalc_wp_offset_direction)
            {
                // get pilot desired lean angles
                float target_roll, target_pitch;
                get_pilot_desired_lean_angles(channel_roll->control_in, channel_pitch->control_in, target_roll, target_pitch);
                // 
                if (target_roll > aparm.angle_max * ROLLIN_SET_WPCRUISE_DIRECTION_THRESHOLD) {
                    mission.set_wp_direction(1);
                } else if (target_roll < -aparm.angle_max * ROLLIN_SET_WPCRUISE_DIRECTION_THRESHOLD)
                {
                    mission.set_wp_direction(-1);
                } else {
                    // run loiter controller
                    wp_nav.update_loiter(ekfGndSpdLimit, ekfNavVelGainScaler);

                    // call attitude controller
                    attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
                    // call z-axis position controller
                    pos_control.update_z_controller();
                    return;
                }
            }
            // calc destination position step by step
            if (wpcruise.flag_init_destination || (wp_nav.reached_wp_destination() && (!wpcruise.flag_reach_destination_old)))
            {
                Vector3f destination;
                switch(wpcruise.state) {
                    // change state of waypoint cruise and delete breakpoint from storage
                    case RETURN_TO_BREAKPOINT:
                    // calculate break point position
                    calc_breakpoint_destination(destination);
                    wpcruise.state = WAYPOINT_NAV;
                    mission.truncate(3);
                    break;

                    // update waypoint nav destination
                    case WAYPOINT_NAV:
                    update_waypoint_destination(destination);
                    break;
                }
                // set destination
                if (g.sonar_alt_wp != 0 && sonar_enabled) {
                    wp_nav.set_wp_xy_origin_and_destination(destination);
                    target_sonar_alt = destination.z;
                }
                else {
                    wp_nav.set_wp_destination(destination);
                }

                wpcruise.flag_init_destination = false;
            }

            wpcruise.flag_reach_destination_old = wp_nav.reached_wp_destination();
        
        // run waypoint controller
        if (g.sonar_alt_wp == 0  || (!sonar_enabled)) {
            wp_nav.update_wpnav();
        } else {
            wp_nav.update_wpnav_xy();

            // calculate target sonar altitude
            // target_sonar_alt = wp_nav.calc_curr_target_z();

            // altitude control according to sonar
            if (sonar_enabled) {
                // if sonar is ok, use surface tracking
                target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control.get_alt_target(), G_Dt);
            }
    
            // update altitude target and call position controller
            pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        }
    
        // call z-axis position controller
        pos_control.update_z_controller();

        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw(wp_nav.get_roll(), wp_nav.get_pitch(), target_yaw_rate);
    }
}
// initilate first destination position during auto cruise
void Copter::calc_breakpoint_destination(Vector3f& destination)
{
    // read break point from storage
    AP_Mission::Mission_Command cmd;
    if (mission.read_cmd_from_storage(3, cmd))
    {
        const Vector3f &curr_pos = inertial_nav.get_position();
        destination = pv_location_to_vector_with_default(cmd.content.location, curr_pos);

        if (g.sonar_alt_wp != 0 && sonar_enabled) {
            destination.z = cmd.content.location.alt;
        }
    }
}
 void Copter::update_waypoint_destination(Vector3f& destination)
 {
    Location destination_loc;
    mission.update_wpcruise_target(destination_loc);

    const Vector3f &curr_pos = inertial_nav.get_position();
    destination = pv_location_to_vector_with_default(destination_loc, curr_pos);

    if (g.sonar_alt_wp != 0 && sonar_enabled) {
        destination.z = destination_loc.alt;
    }
 }
#endif   // WPCRUISE_ENABLED == ENABLED
