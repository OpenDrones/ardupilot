/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>
#include "AP_InertialNav.h"

#if AP_AHRS_NAVEKF_AVAILABLE

/*
  A wrapper around the AP_InertialNav class which uses the NavEKF
  filter if available, and falls back to the AP_InertialNav filter
  when EKF is not available
 */

// table of user settable parameters
const AP_Param::GroupInfo AP_InertialNav_NavEKF::var_info[] PROGMEM = { 
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
    float alt_corr;

    _ahrs_ekf.get_NavEKF().getPosNED(_relpos_cm);
    _relpos_cm *= 100; // convert to cm

    _haveabspos = _ahrs_ekf.get_position(_abspos);

    _ahrs_ekf.get_NavEKF().getVelNED(_velocity_cm);
    _velocity_cm *= 100; // convert to cm/s

    // calc altitude corretion
    alt_corr = calc_alt_corretion(dt);

    // InertialNav is NEU
    _relpos_cm.z = - _relpos_cm.z;
    _relpos_cm.z -= alt_corr;
    _velocity_cm.z = -_velocity_cm.z;
}

/**
 * get_filter_status : returns filter status as a series of flags
 */
nav_filter_status AP_InertialNav_NavEKF::get_filter_status() const
{
    nav_filter_status ret;
    _ahrs_ekf.get_NavEKF().getFilterStatus(ret);
    return ret;
}

/**
 * get_origin - returns the inertial navigation origin in lat/lon/alt
 */
struct Location AP_InertialNav_NavEKF::get_origin() const
{
    struct Location ret;
    if (!_ahrs_ekf.get_NavEKF().getOriginLLH(ret)) {
        // initialise location to all zeros if origin not yet set
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
 * get_location - updates the provided location with the latest calculated locatoin
 *  returns true on success (i.e. the EKF knows it's latest position), false on failure
 */
bool AP_InertialNav_NavEKF::get_location(struct Location &loc) const
{
    return _ahrs_ekf.get_NavEKF().getLLH(loc);
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
    return pythagorous2(_velocity_cm.x, _velocity_cm.y);
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
    bool valid = _ahrs_ekf.get_NavEKF().getHAGL(height);
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
    if (_ahrs_ekf.get_NavEKF().getHeightControlLimit(limit)) {
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
    velocity = pythagorous2(_velocity_cm.x,_velocity_cm.y);
    
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
