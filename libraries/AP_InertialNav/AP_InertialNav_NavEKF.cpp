#include <AP_HAL/AP_HAL.h>
#include "AP_InertialNav.h"

#if AP_AHRS_NAVEKF_AVAILABLE

/*
  A wrapper around the AP_InertialNav class which uses the NavEKF
  filter if available, and falls back to the AP_InertialNav filter
  when EKF is not available
 */

// table of user settable parameters
const AP_Param::GroupInfo AP_InertialNav_NavEKF::var_info[] = { 
    // @Param: ALT_CORR_FAC0
    // @DisplayName: 
    // @Description: altitude correction factor a
    // @Units:
    // @Range:
    // @Increment: 1
    AP_GROUPINFO("ALT_C0", 0, AP_InertialNav_NavEKF, _alt_corr_fac_a, 2),

    // @Param: ALT_CORR_FAC1
    // @DisplayName: 
    // @Description: altitude correction factor b
    // @Units: 
    // @Range:
    // @Increment: 1
    AP_GROUPINFO("ALT_C1", 1, AP_InertialNav_NavEKF, _alt_corr_fac_b, 4),

    // @Param: ALT_CORR_FAC0FW
    // @DisplayName: 
    // @Description: altitude correction factor a if flying to front
    // @Units:
    // @Range:
    // @Increment: 1
    AP_GROUPINFO("ALT_C0FW", 2, AP_InertialNav_NavEKF, _alt_corr_fac_af, 3),

    // @Param: ALT_CORR_FAC1FW
    // @DisplayName: 
    // @Description: altitude correction factor b if flying to front
    // @Units: 
    // @Range:
    // @Increment: 1
    AP_GROUPINFO("ALT_C1FW", 3, AP_InertialNav_NavEKF, _alt_corr_fac_bf, 6),

    // @Param: ALT_CORR_MAX
    // @DisplayName: 
    // @Description: maxinmun altitude correction
    // @Units: cm
    // @Range: 
    // @Increment: 1
    AP_GROUPINFO("ALT_CMAX", 4, AP_InertialNav_NavEKF, _alt_corr_max, 100),

    // @Param: ALT_C_FF
    // @DisplayName: 
    // @Description: altitude correction filter frequency
    // @Units: hz
    // @Range: 
    // @Increment: 1
    AP_GROUPINFO("ALT_C_FF", 5, AP_InertialNav_NavEKF, _alt_corr_ffreq, 10),

    // @Param: ALT_C_VFF
    // @DisplayName: 
    // @Description: filter frequency of velocity to calculate altitude correction
    // @Units: hz
    // @Range: 
    // @Increment: 
    AP_GROUPINFO("ALT_C_VFF", 6, AP_InertialNav_NavEKF, _alt_c_vel_ffreq, 20),
    
    AP_GROUPEND
};

/**
   update internal state
*/
void AP_InertialNav_NavEKF::update(float dt)
{
    // get the NE position relative to the local earth frame origin
    Vector2f posNE;
    if (_ahrs_ekf.get_relative_position_NE(posNE)) {
        _relpos_cm.x = posNE.x * 100; // convert from m to cm
        _relpos_cm.y = posNE.y * 100; // convert from m to cm
    }

    // get the D position relative to the local earth frame origin
    float posD;
    if (_ahrs_ekf.get_relative_position_D(posD)) {
        _relpos_cm.z = - posD * 100; // convert from m in NED to cm in NEU
    }
	
	float alt_corr;
	// calc altitude corretion
    alt_corr = calc_alt_corretion(dt);
	_relpos_cm.z -= alt_corr;

    // get the absolute WGS-84 position
    _haveabspos = _ahrs_ekf.get_position(_abspos);

    // get the velocity relative to the local earth frame
    Vector3f velNED;
    if (_ahrs_ekf.get_velocity_NED(velNED)) {
        _velocity_cm = velNED * 100; // convert to cm/s
        _velocity_cm.z = -_velocity_cm.z; // convert from NED to NEU
    }

    // Get a derivative of the vertical position which is kinematically consistent with the vertical position is required by some control loops.
    // This is different to the vertical velocity from the EKF which is not always consistent with the verical position due to the various errors that are being corrected for.
    if (_ahrs_ekf.get_vert_pos_rate(_pos_z_rate)) {
        _pos_z_rate *= 100; // convert to cm/s
        _pos_z_rate = - _pos_z_rate; // InertialNav is NEU
    }
}

/**
 * get_filter_status : returns filter status as a series of flags
 */
nav_filter_status AP_InertialNav_NavEKF::get_filter_status() const
{
    nav_filter_status status;
    _ahrs_ekf.get_filter_status(status);
    return status;
}

/**
 * get_origin - returns the inertial navigation origin in lat/lon/alt
 */
struct Location AP_InertialNav_NavEKF::get_origin() const
{
    struct Location ret;
     if (!_ahrs_ekf.get_origin(ret)) {
         // initialise location to all zeros if EKF1 origin not yet set
         memset(&ret, 0, sizeof(ret));
     }
    return ret;
}

/**
 * get_position - returns the current position relative to the home location in cm.
 *
 * @return
 */
const Vector3f &AP_InertialNav_NavEKF::get_position(void) const 
{
    return _relpos_cm;
}

/**
 * get_location - updates the provided location with the latest calculated location
 *  returns true on success (i.e. the EKF knows it's latest position), false on failure
 */
bool AP_InertialNav_NavEKF::get_location(struct Location &loc) const
{
    return _ahrs_ekf.get_location(loc);
}

/**
 * get_latitude - returns the latitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
 */
int32_t AP_InertialNav_NavEKF::get_latitude() const
{
    return _abspos.lat;
}

/**
 * get_longitude - returns the longitude of the current position estimation in 100 nano degrees (i.e. degree value multiplied by 10,000,000)
 * @return
 */
int32_t AP_InertialNav_NavEKF::get_longitude() const
{
    return _abspos.lng;
}

/**
 * get_velocity - returns the current velocity in cm/s
 *
 * @return velocity vector:
 *      		.x : latitude  velocity in cm/s
 * 				.y : longitude velocity in cm/s
 * 				.z : vertical  velocity in cm/s
 */
const Vector3f &AP_InertialNav_NavEKF::get_velocity() const
{
    return _velocity_cm;
}

/**
 * get_velocity_xy - returns the current horizontal velocity in cm/s
 *
 * @returns the current horizontal velocity in cm/s
 */
float AP_InertialNav_NavEKF::get_velocity_xy() const
{
    return norm(_velocity_cm.x, _velocity_cm.y);
}

/**
 * get_pos_z_derivative - returns the derivative of the z position in cm/s
*/
float AP_InertialNav_NavEKF::get_pos_z_derivative() const
{
    return _pos_z_rate;
}

/**
 * get_altitude - get latest altitude estimate in cm
 * @return
 */
float AP_InertialNav_NavEKF::get_altitude() const
{
    return _relpos_cm.z;
}

/**
 * getHgtAboveGnd - get latest height above ground level estimate in cm and a validity flag
 *
 * @return
 */
bool AP_InertialNav_NavEKF::get_hagl(float &height) const
{
    // true when estimate is valid
    bool valid = _ahrs_ekf.get_hagl(height);
    // convert height from m to cm
    height *= 100.0f;
    return valid;
}

/**
 * get_hgt_ctrl_limit - get maximum height to be observed by the control loops in cm and a validity flag
 * this is used to limit height during optical flow navigation
 * it will return invalid when no limiting is required
 * @return
 */
bool AP_InertialNav_NavEKF::get_hgt_ctrl_limit(float& limit) const
{
    // true when estimate is valid
    if (_ahrs_ekf.get_hgt_ctrl_limit(limit)) {
        // convert height from m to cm
        limit *= 100.0f;
        return true;
    }
    return false;
}

/**
 * get_velocity_z - returns the current climbrate.
 *
 * @see get_velocity().z
 *
 * @return climbrate in cm/s
 */
float AP_InertialNav_NavEKF::get_velocity_z() const
{
    return _velocity_cm.z;
}

// altitude correction calculation according to current velocity
float AP_InertialNav_NavEKF::calc_alt_corretion(float dt)
{
    float alt_corretion;
    float velocity,vel_fwd;
    bool flag_front;
    // fetch velocity in cm/s
    velocity = norm(_velocity_cm.x,_velocity_cm.y);
    
    // convert inertial nav earth-frame velocities_cm/s to body-frame
    vel_fwd = _velocity_cm.x*_ahrs_ekf.cos_yaw() + _velocity_cm.y*_ahrs_ekf.sin_yaw();
    flag_front = (vel_fwd >= velocity*0.6);
    
    // calculate velocity in m/s
    velocity /= 100;
    // lowpassfilter of current speed
    _alt_c_vel_filter.set_cutoff_frequency(_alt_c_vel_ffreq/100.0);
    velocity = _alt_c_vel_filter.apply(velocity,dt);

    // calculate alt correction according to velocity
    if (flag_front) {
        alt_corretion = _alt_corr_fac_af*velocity*velocity + _alt_corr_fac_bf*velocity;
    } else {
        alt_corretion = _alt_corr_fac_a*velocity*velocity + _alt_corr_fac_b*velocity;
    }

    // calculate alt correction filter
    _alt_corr_filter.set_cutoff_frequency(_alt_corr_ffreq/100.0);
    alt_corretion = _alt_corr_filter.apply(alt_corretion,dt);
    alt_corretion = constrain_float(alt_corretion,0,_alt_corr_max);

    return alt_corretion;
}


#endif // AP_AHRS_NAVEKF_AVAILABLE
