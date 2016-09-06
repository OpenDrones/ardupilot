/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

#if CRUISE_ENABLED == ENABLED

#define CRUISE_CURR_SPEED_MIN_DEFAULT       50.0f       // speed threshold to judge cruise forward or back
#define CRUISE_DESTINATION_DISTANCE_CM      5000.0f     // distance to calc next cruise destination
#define DISTANCE_TO_CALC_NEXT_CRUISE_DES_CM 1000.0f     // min distance to judge need to calc next cruise distination
#define PILOT_IN_SET_CRUISE_DIRECTION_THRESHOLD 0.5f

// cruise state
enum cruise_state_mode {
    CRUISE_LOITER = 0,         // loiter in cruise
    CRUISE_FORTH,              // fly to forthward in cruise
    CRUISE_BACK,               // fly to backward in cruise
    CRUISE_RIGHT,              // fly to right in cruise
    CRUISE_LEFT,               // fly to left in cruise
    };

static struct {
    bool flag_reach_cruise_des;        // flag to update cruise destination if cruise state is FORTH or BACK
    bool flag_init_cruise_des;         // flag to calc initial cruise destination
    bool flag_recalc_cruise_state;     // flag to recalc cruise state according to pilot input
    bool flag_cruise_brake;            // flag to brake during cruise forth or back
    cruise_state_mode state     :3;    // cruise state to calculate different destination
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
        cruise.flag_recalc_cruise_state = false;
        cruise.flag_cruise_brake = false;
        cruise.state = CRUISE_LOITER;
        cruise.cos_bearing = ahrs.cos_yaw();
        cruise.sin_bearing = ahrs.sin_yaw();
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
        // get pilot desired lean angles
        float target_roll, target_pitch;
        get_pilot_desired_lean_angles(channel_roll->control_in, channel_pitch->control_in, target_roll, target_pitch);

        // calc cruise state: forward or backward or left or right
        if (cruise.state == CRUISE_LOITER) {
            // // set flag to recalc cruise state according to pilot input
            if (is_zero(target_pitch) && is_zero(target_roll)) {
                cruise.flag_recalc_cruise_state = true;
            }
            // calc cruise state according to pilot inputs 
            if (target_pitch > aparm.angle_max * PILOT_IN_SET_CRUISE_DIRECTION_THRESHOLD && cruise.flag_recalc_cruise_state) {
                cruise.state = CRUISE_BACK;
                cruise.flag_init_cruise_des = true;
                cruise.flag_recalc_cruise_state = false;
            } else if (target_pitch < -aparm.angle_max * PILOT_IN_SET_CRUISE_DIRECTION_THRESHOLD && cruise.flag_recalc_cruise_state) {
                    cruise.state = CRUISE_FORTH;
                    cruise.flag_init_cruise_des = true;
                    cruise.flag_recalc_cruise_state = false;
                } else if (target_roll < -aparm.angle_max * PILOT_IN_SET_CRUISE_DIRECTION_THRESHOLD && cruise.flag_recalc_cruise_state) {
                        cruise.state = CRUISE_LEFT;
                        cruise.flag_init_cruise_des = true;
                        cruise.flag_recalc_cruise_state = false;
                    } else if (target_roll > aparm.angle_max * PILOT_IN_SET_CRUISE_DIRECTION_THRESHOLD && cruise.flag_recalc_cruise_state) {
                            cruise.state = CRUISE_RIGHT;
                            cruise.flag_init_cruise_des = true;
                            cruise.flag_recalc_cruise_state = false;
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
        
        // calc cruise destination
        if (cruise.flag_init_cruise_des || cruise.flag_reach_cruise_des) {
            switch (cruise.state) {
                case CRUISE_FORTH:
                    update_cruise_des_fwd(1, cruise.destination);
                    break;
                case CRUISE_BACK:
                    update_cruise_des_fwd(-1, cruise.destination);
                    break;
                case CRUISE_RIGHT:
                    update_cruise_des_rgt(1, cruise.destination);
                    break;
                case CRUISE_LEFT:
                    update_cruise_des_rgt(-1, cruise.destination);
                    break;
            }
            // set destination
            wp_nav.set_wp_xy_origin_and_destination(cruise.destination);
            
            cruise.flag_init_cruise_des = false;
        }
        // calc flag if reaching destination
        if (cruise.state == CRUISE_RIGHT || cruise.state == CRUISE_LEFT || cruise.flag_cruise_brake) {
            cruise.flag_reach_cruise_des = false;
        } else {
            // calc flag to reach next cruise destination
            cruise.flag_reach_cruise_des = reach_cruise_des(cruise.destination);
        }

        // switch cruise state according to pilot input
        if ((cruise.state == CRUISE_FORTH && (target_pitch > aparm.angle_max * PILOT_IN_SET_CRUISE_DIRECTION_THRESHOLD)) || 
            (cruise.state == CRUISE_BACK && (target_pitch < -aparm.angle_max * PILOT_IN_SET_CRUISE_DIRECTION_THRESHOLD))) {
            // set flag cruise brake
            cruise.flag_cruise_brake = true;
            // set stopping point as destination
            pos_control.get_stopping_point_xy(cruise.destination);
            pos_control.get_stopping_point_z(cruise.destination);
            // set destination
            wp_nav.set_wp_xy_origin_and_destination(cruise.destination);
        }

        // set cruise state as cruise_loiter if reaching stopping point during cruise brake
        if (cruise.flag_cruise_brake && wp_nav.reached_wp_destination()) {
            cruise.state = CRUISE_LOITER;
            cruise.flag_cruise_brake = false;
        }

        // switch cruise state as loiter when finishing right or left mission
        if ((cruise.state == CRUISE_LEFT || cruise.state == CRUISE_RIGHT) && wp_nav.reached_wp_destination()) {
            // reset cruise_state as loiter
            cruise.state = CRUISE_LOITER;
        }
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
void Copter::update_cruise_des_fwd(float direction, Vector3f& destination)
{
    const Vector3f &curr_pos = inertial_nav.get_position();
    destination.x = curr_pos.x + CRUISE_DESTINATION_DISTANCE_CM * cruise.cos_bearing * direction;
    destination.y = curr_pos.y + CRUISE_DESTINATION_DISTANCE_CM * cruise.sin_bearing * direction;
    destination.z = curr_pos.z;
}

// update cruise destination to left or right
void Copter::update_cruise_des_rgt(float direction, Vector3f& destination)
{
    const Vector3f &curr_pos = inertial_nav.get_position();
    float distance_offset = mission.get_distance_cm() + 100;
    destination.x = curr_pos.x - distance_offset * cruise.sin_bearing * direction;
    destination.y = curr_pos.y + distance_offset * cruise.cos_bearing * direction;
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