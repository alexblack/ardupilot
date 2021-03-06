/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

uint32_t Copter::get_ready_to_arm_mode_mask(void)
{
    uint32_t ret = 0;
    control_mode_t saved_control_mode = control_mode;
    for (uint8_t i=0; i < FLIGHT_MODE_MAX; i++) {
        if (is_valid_flight_mode(i)) {
            control_mode = (control_mode_t)i;
            if (pre_arm_checks(false) && arm_checks(false,true)) {
                ret |= 1UL << control_mode;
            }
        }
    }

    control_mode = saved_control_mode;
    return ret;
}

// performs pre-arm checks. expects to be called at 1hz.
void Copter::update_arming_checks(void)
{
    if (!motors.armed()) {
        // perform pre-arm checks & display failures every 30 seconds
        static uint8_t pre_arm_display_counter = PREARM_DISPLAY_PERIOD/2;
        pre_arm_display_counter++;
        bool display_fail = false;
        if (pre_arm_display_counter >= PREARM_DISPLAY_PERIOD) {
            display_fail = true;
            pre_arm_display_counter = 0;
        }

        if (pre_arm_checks(display_fail)) {
            set_pre_arm_check(true);
        }

        if (display_fail && ap.pre_arm_check) {
            // pre-arm checks have passed, run arming checks in order to display failures
            // this is a HACK that allows the GCS to display arming failures without attempting to arm
            arm_checks(true,true);
        }
    }
}


void Copter::update_ekf_innovation_check(void)
{
    const float dt = 1.0f/50.0f;
    const float tc = 5.0f;
    const float alpha = dt/(dt+tc);
    Vector3f velInnov, posInnov;
    ahrs.getPosVelInnovations(velInnov, posInnov);
    velInnov.z = 0;
    posInnov.z = 0;
    ekf_innovation_peak = MAX(ekf_innovation_peak, posInnov.length()/0.55f);
    ekf_innovation_peak = MAX(ekf_innovation_peak, velInnov.length()/0.7f);
    ekf_innovation_peak = MIN(ekf_innovation_peak, 10.0f);
    ekf_innovation_peak -= ekf_innovation_peak*alpha;
}

// performs pre-arm checks and arming checks
bool Copter::all_arming_checks_passing(bool arming_from_gcs)
{
    if (pre_arm_checks(true)) {
        set_pre_arm_check(true);
    }

    return ap.pre_arm_check && arm_checks(true, arming_from_gcs);
}

// perform pre-arm checks and set ap.pre_arm_check flag
//  return true if the checks pass successfully
bool Copter::pre_arm_checks(bool display_failure)
{
    // exit immediately if already armed
    if (motors.armed()) {
        return true;
    }

    // check if motor interlock and Emergency Stop aux switches are used
    // at the same time.  This cannot be allowed.
    if (check_if_auxsw_mode_used(AUXSW_MOTOR_INTERLOCK) && check_if_auxsw_mode_used(AUXSW_MOTOR_ESTOP)){
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Interlock/E-Stop Conflict");
        }
        return false;
    }

    // check if motor interlock aux switch is in use
    // if it is, switch needs to be in disabled position to arm
    // otherwise exit immediately.  This check to be repeated,
    // as state can change at any time.
    if (ap.using_interlock && ap.motor_interlock_switch){
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Motor Interlock Enabled");
        }
        return false;
    }

    // exit immediately if we've already successfully performed the pre-arm check
    if (ap.pre_arm_check) {
        // run gps checks because results may change and affect LED colour
        // no need to display failures because arm_checks will do that if the pilot tries to arm
        pre_arm_gps_checks(false);
        return true;
    }

    // succeed if pre arm checks are disabled
    if (g.arming_check == ARMING_CHECK_NONE) {
        set_pre_arm_check(true);
        set_pre_arm_rc_check(true);
        return true;
    }

    // check if sensors are calibrating
    if (compass.is_calibrating()) {
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Compass calibration running");
        }
        return false;
    }

    if (ins.calibrating_accel()) {
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Accelerometer calibration running");
        }
        return false;
    }

    // pre-arm rc checks a prerequisite
    pre_arm_rc_checks();
    if (!ap.pre_arm_rc_check) {
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: RC not calibrated");
        }
        return false;
    }
    // check Baro
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_BARO)) {
        // barometer health check
        if (!barometer.all_healthy()) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Barometer not healthy");
            }
            return false;
        }
        // Check baro & inav alt are within 1m if EKF is operating in an absolute position mode.
        // Do not check if intending to operate in a ground relative height mode as EKF will output a ground relative height
        // that may differ from the baro height due to baro drift.
        nav_filter_status filt_status = inertial_nav.get_filter_status();
        bool using_baro_ref = (!filt_status.flags.pred_horiz_pos_rel && filt_status.flags.pred_horiz_pos_abs);
        if (using_baro_ref) {
            if (fabsf(inertial_nav.get_altitude() - baro_alt) > PREARM_MAX_ALT_DISPARITY_CM) {
                if (display_failure) {
                    gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Altitude disparity");
                }
                return false;
            }
        }
    }

    // check Compass
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_COMPASS)) {
        //check if compass has calibrated and requires reboot
        if (compass.compass_cal_requires_reboot()) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL, "PreArm: Compass calibrated requires reboot");
            }
            return false;
        }

        // check the primary compass is healthy
        if (!compass.healthy()) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Compass not healthy");
            }
            return false;
        }

        // check compass learning is on or offsets have been set
        if (!compass.configured()) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Compass not calibrated");
            }
            return false;
        }

        // check for unreasonable compass offsets
        Vector3f offsets = compass.get_offsets();
        if (offsets.length() > COMPASS_OFFSETS_MAX) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Compass offsets too high");
            }
            return false;
        }

        // check for unreasonable mag field length
        float mag_field = compass.get_field().length();
        if (mag_field > COMPASS_MAGFIELD_EXPECTED*1.65f || mag_field < COMPASS_MAGFIELD_EXPECTED*0.35f) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Check mag field");
            }
            return false;
        }

        // check all compasses point in roughly same direction
        if (!compass.consistent()) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: inconsistent compasses");
            }
            return false;
        }

    }

    #if AC_FENCE == ENABLED
    // check fence is initialised
    if (!fence.pre_arm_check()) {
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: check fence");
        }
        return false;
    }
    #endif

    // check INS
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_INS)) {
        // check accelerometers have been calibrated
        if (!ins.accel_calibrated_ok_all()) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: INS not calibrated");
            }
            return false;
        }

        // check accels are healthy
        if (!ins.get_accel_health_all()) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Accelerometers not healthy");
            }
            return false;
        }

        //check if accelerometers have calibrated and require reboot
        if (ins.accel_cal_requires_reboot()) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL, "PreArm: Accelerometers calibrated requires reboot");
            }
            return false;
        }

        // check all accelerometers point in roughly same direction
        if (ins.get_accel_count() > 1) {
            const Vector3f &prime_accel_vec = ins.get_accel();
            for(uint8_t i=0; i<ins.get_accel_count(); i++) {
                // get next accel vector
                const Vector3f &accel_vec = ins.get_accel(i);
                Vector3f vec_diff = accel_vec - prime_accel_vec;
                float threshold = PREARM_MAX_ACCEL_VECTOR_DIFF;
                if (i >= 2) {
                    /*
                     * for boards with 3 IMUs we only use the first two
                     * in the EKF. Allow for larger accel discrepancy
                     * for IMU3 as it may be running at a different temperature
                     */
                    threshold *= 2;
                }
                if (vec_diff.length() > GRAVITY_MSS) {
                    // more than 1G of discrepancy means we should report a failed accelerometer
                    if (display_failure) {
                        gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Accelerometers not healthy");
                    }
                    return false;
                }
                // EKF is less sensitive to Z-axis error and Z-axis is more
                // likely to be temperature-sensitive on our hardware
                vec_diff.z *= 0.5f;
                if (vec_diff.length() > threshold) {
                    if (display_failure) {
                        gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: inconsistent Accelerometers");
                    }
                    return false;
                }
            }
        }

        // check lean angle
        if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_INS)) {
            if (degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch()))*100.0f > aparm.angle_max) {
                if (display_failure) {
                    gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Leaning");
                }
                return false;
            }
        }

        // check gyros are healthy
        if (!ins.get_gyro_health_all()) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Gyros not healthy");
            }
            return false;
        }

        // check all gyros are consistent
        if (ins.get_gyro_count() > 1) {
            for(uint8_t i=0; i<ins.get_gyro_count(); i++) {
                // get rotation rate difference between gyro #i and primary gyro
                Vector3f vec_diff = ins.get_gyro(i) - ins.get_gyro();
                if (vec_diff.length() > PREARM_MAX_GYRO_VECTOR_DIFF) {
                    if (display_failure) {
                        gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: inconsistent Gyros");
                    }
                    return false;
                }
            }
        }

        // get ekf attitude (if bad, it's usually the gyro biases)
        if (!pre_arm_ekf_attitude_check()) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Waiting for Nav Checks");
            }
            return false;
        }
    }

    // check GPS
    if (!pre_arm_gps_checks(display_failure)) {
        return false;
    }

    #if CONFIG_HAL_BOARD != HAL_BOARD_VRBRAIN
    #ifndef CONFIG_ARCH_BOARD_PX4FMU_V1
    // check board voltage
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_VOLTAGE)) {
        if (hal.analogin->board_voltage() < BOARD_VOLTAGE_MIN || hal.analogin->board_voltage() > BOARD_VOLTAGE_MAX) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Check Board Voltage");
            }
            return false;
        }
    }
    #endif
    #endif

    // check battery voltage
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_VOLTAGE)) {
        if (failsafe.battery || (!ap.usb_connected && battery.exhausted(g.fs_batt_voltage, g.fs_batt_mah))) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Check Battery");
            }
            return false;
        }
    }

    // check various parameter values
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_PARAMETERS)) {

        // ensure ch7 and ch8 have different functions
        if (check_duplicate_auxsw()) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Duplicate Aux Switch Options");
            }
            return false;
        }

        // failsafe parameter checks
        if (g.failsafe_throttle) {
            // check throttle min is above throttle failsafe trigger and that the trigger is above ppm encoder's loss-of-signal value of 900
            if (channel_throttle->get_radio_min() <= g.failsafe_throttle_value+10 || g.failsafe_throttle_value < 910) {
                if (display_failure) {
                    gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Check FS_THR_VALUE");
                }
                return false;
            }
        }

        // lean angle parameter check
        if (aparm.angle_max < 1000 || aparm.angle_max > 8000) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Check ANGLE_MAX");
            }
            return false;
        }

        // acro balance parameter check
        if ((g.acro_balance_roll > attitude_control.get_angle_roll_p().kP()) || (g.acro_balance_pitch > attitude_control.get_angle_pitch_p().kP())) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: ACRO_BAL_ROLL/PITCH");
            }
            return false;
        }

        #if RANGEFINDER_ENABLED == ENABLED && OPTFLOW == ENABLED
        // check range finder if optflow enabled
        if (optflow.enabled() && !rangefinder.pre_arm_check()) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: check range finder");
            }
            return false;
        }
        #endif
        #if FRAME_CONFIG == HELI_FRAME
        // check helicopter parameters
        if (!motors.parameter_check(display_failure)) {
            return false;
        }
        #endif // HELI_FRAME

        // check for missing terrain data
        if (!pre_arm_terrain_check(display_failure)) {
            return false;
        }
    }

    // check throttle is above failsafe throttle
    // this is near the bottom to allow other failures to be displayed before checking pilot throttle
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_RC)) {
        if (g.failsafe_throttle != FS_THR_DISABLED && channel_throttle->get_radio_in() < g.failsafe_throttle_value) {
            if (display_failure) {
                #if FRAME_CONFIG == HELI_FRAME
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Collective below Failsafe");
                #else
                gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Throttle below Failsafe");
                #endif
            }
            return false;
        }
    }

    return true;
}

// perform pre_arm_rc_checks checks and set ap.pre_arm_rc_check flag
void Copter::pre_arm_rc_checks()
{
    // exit immediately if we've already successfully performed the pre-arm rc check
    if (ap.pre_arm_rc_check) {
        return;
    }

    // set rc-checks to success if RC checks are disabled
    if ((g.arming_check != ARMING_CHECK_ALL) && !(g.arming_check & ARMING_CHECK_RC)) {
        set_pre_arm_rc_check(true);
        return;
    }

    // check if radio has been calibrated
    if (!channel_throttle->min_max_configured()) {
        return;
    }

    // check channels 1 & 2 have min <= 1300 and max >= 1700
    if (channel_roll->get_radio_min() > 1300 || channel_roll->get_radio_max() < 1700 || channel_pitch->get_radio_min() > 1300 || channel_pitch->get_radio_max() < 1700) {
        return;
    }

    // check channels 3 & 4 have min <= 1300 and max >= 1700
    if (channel_throttle->get_radio_min() > 1300 || channel_throttle->get_radio_max() < 1700 || channel_yaw->get_radio_min() > 1300 || channel_yaw->get_radio_max() < 1700) {
        return;
    }

    // check channels 1 & 2 have trim >= 1300 and <= 1700
    if (channel_roll->get_radio_trim() < 1300 || channel_roll->get_radio_trim() > 1700 || channel_pitch->get_radio_trim() < 1300 || channel_pitch->get_radio_trim() > 1700) {
        return;
    }

    // check channel 4 has trim >= 1300 and <= 1700
    if (channel_yaw->get_radio_trim() < 1300 || channel_yaw->get_radio_trim() > 1700) {
        return;
    }

    // if we've gotten this far rc is ok
    set_pre_arm_rc_check(true);
}

// performs pre_arm gps related checks and returns true if passed
bool Copter::pre_arm_gps_checks(bool display_failure)
{
    // always check if inertial nav has started and is ready
    if (!ahrs.healthy()) {
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Waiting for Nav Checks");
        }
        return false;
    }

    if (position_ok() && ekf_innovation_peak > 1.0f) {
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Waiting for Nav Checks");
        }
        return false;
    }

    // check if flight mode requires GPS
    bool gps_required = mode_requires_GPS(control_mode);

    #if AC_FENCE == ENABLED
    // if circular fence is enabled we need GPS
    if ((fence.get_enabled_fences() & AC_FENCE_TYPE_CIRCLE) != 0) {
        gps_required = true;
    }
    #endif

    // return true if GPS is not required
    if (!gps_required) {
        AP_Notify::flags.pre_arm_gps_check = true;
        return true;
    }

    // ensure GPS is ok
    if (!position_ok()) {
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Need 3D Fix");
        }
        AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // check EKF compass variance is below failsafe threshold
    float vel_variance, pos_variance, hgt_variance, tas_variance;
    Vector3f mag_variance;
    Vector2f offset;
    ahrs.get_variances(vel_variance, pos_variance, hgt_variance, mag_variance, tas_variance, offset);
    if (mag_variance.x>1.0f || mag_variance.y>1.0f || mag_variance.z>1.0f) {
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Waiting for Nav Checks");
        }
        return false;
    }

    // check home and EKF origin are not too far
    if (far_from_EKF_origin(ahrs.get_home())) {
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: EKF-home variance");
        }
        AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // return true immediately if gps check is disabled
    if (!(g.arming_check == ARMING_CHECK_ALL || g.arming_check & ARMING_CHECK_GPS)) {
        AP_Notify::flags.pre_arm_gps_check = true;
        return true;
    }

    // warn about hdop separately - to prevent user confusion with no gps lock
    if (gps.get_hdop() > g.gps_hdop_good) {
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Need 3D Fix");
        }
        AP_Notify::flags.pre_arm_gps_check = false;
        return false;
    }

    // if we got here all must be ok
    AP_Notify::flags.pre_arm_gps_check = true;
    return true;
}

// check ekf attitude is acceptable
bool Copter::pre_arm_ekf_attitude_check()
{
    // get ekf filter status
    nav_filter_status filt_status = inertial_nav.get_filter_status();

    return filt_status.flags.attitude;
}

// check we have required terrain data
bool Copter::pre_arm_terrain_check(bool display_failure)
{
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    // succeed if not using terrain data
    if (!terrain_use()) {
        return true;
    }

    // check if terrain following is enabled, using a range finder but RTL_ALT is higher than rangefinder's max range
    // To-Do: modify RTL return path to fly at or above the RTL_ALT and remove this check
    if ((rangefinder.num_sensors() > 0) && (g.rtl_altitude > rangefinder.max_distance_cm())) {
        gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: RTL_ALT above rangefinder max range");
        return false;
    }

    // show terrain statistics
    uint16_t terr_pending, terr_loaded;
    terrain.get_statistics(terr_pending, terr_loaded);
    bool have_all_data = (terr_pending <= 0);
    if (!have_all_data && display_failure) {
        gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Waiting for Terrain data");
    }
    return have_all_data;
#else
    return true;
#endif
}

// arm_checks - perform final checks before arming
//  always called just before arming.  Return true if ok to arm
//  has side-effect that logging is started
bool Copter::arm_checks(bool display_failure, bool arming_from_gcs)
{
    #if LOGGING_ENABLED == ENABLED
    // start dataflash
    start_logging();
    #endif

    // check if sensors are calibrating
    if (compass.is_calibrating()) {
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Compass calibration running");
        }
        return false;
    }

    if (ins.calibrating_accel()) {
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Accelerometer calibration running");
        }
        return false;
    }

    // check accels and gyro are healthy
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_INS)) {
        // check accelerometers have been calibrated
        if (!ins.accel_calibrated_ok_all()) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: INS not calibrated");
            }
            return false;
        }

        //check if accelerometers have calibrated and require reboot
        if (ins.accel_cal_requires_reboot()) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL, "PreArm: Accelerometers calibrated requires reboot");
            }
            return false;
        }

        if (!ins.get_accel_health_all()) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Accelerometers not healthy");
            }
            return false;
        }
        if (!ins.get_gyro_health_all()) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Gyros not healthy");
            }
            return false;
        }
        // get ekf attitude (if bad, it's usually the gyro biases)
        if (!pre_arm_ekf_attitude_check()) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Waiting for Nav Checks");
            }
            return false;
        }
    }

    // check lean angle
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_INS)) {
        if (degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch()))*100.0f > aparm.angle_max) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Leaning");
            }
            return false;
        }
    }

    // always check if inertial nav has started and is ready
    if (!ahrs.healthy()) {
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Waiting for Nav Checks");
        }
        return false;
    }


    //check if compass has calibrated and requires reboot
    if (compass.compass_cal_requires_reboot()) {
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL, "PreArm: Compass calibrated requires reboot");
        }
        return false;
    }

    // always check if the current mode allows arming
    if (!mode_allows_arming(control_mode, arming_from_gcs)) {
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Mode not armable");
        }
        return false;
    }

    // always check gps
    if (!pre_arm_gps_checks(display_failure)) {
        return false;
    }

    // if we are using motor interlock switch and it's enabled, fail to arm
    if (ap.using_interlock && motors.get_interlock()){
        gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Motor Interlock Enabled");
        return false;
    }

    // if we are not using Emergency Stop switch option, force Estop false to ensure motors
    // can run normally
    if (!check_if_auxsw_mode_used(AUXSW_MOTOR_ESTOP)){
        set_motor_emergency_stop(false);
        // if we are using motor Estop switch, it must not be in Estop position
    } else if (check_if_auxsw_mode_used(AUXSW_MOTOR_ESTOP) && ap.motor_emergency_stop){
        gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Motor Emergency Stopped");
        return false;
    }

    // succeed if arming checks are disabled
    if (g.arming_check == ARMING_CHECK_NONE) {
        return true;
    }

    // baro checks
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_BARO)) {
        // baro health check
        if (!barometer.all_healthy()) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: Barometer not healthy");
            }
            return false;
        }
        // Check baro & inav alt are within 1m if EKF is operating in an absolute position mode.
        // Do not check if intending to operate in a ground relative height mode as EKF will output a ground relative height
        // that may differ from the baro height due to baro drift.
        nav_filter_status filt_status = inertial_nav.get_filter_status();
        bool using_baro_ref = (!filt_status.flags.pred_horiz_pos_rel && filt_status.flags.pred_horiz_pos_abs);
        if (using_baro_ref && (fabsf(inertial_nav.get_altitude() - baro_alt) > PREARM_MAX_ALT_DISPARITY_CM)) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Altitude disparity");
            }
            return false;
        }
    }

    #if AC_FENCE == ENABLED
    // check vehicle is within fence
    if (!fence.pre_arm_check()) {
        if (display_failure) {
            gcs_send_text(MAV_SEVERITY_CRITICAL,"PreArm: check fence");
        }
        return false;
    }
    #endif

    // check battery voltage
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_VOLTAGE)) {
        if (failsafe.battery || (!ap.usb_connected && battery.exhausted(g.fs_batt_voltage, g.fs_batt_mah))) {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Check Battery");
            }
            return false;
        }
    }

    // check for missing terrain data
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_PARAMETERS)) {
        if (!pre_arm_terrain_check(display_failure)) {
            return false;
        }
    }

    // check throttle
    if ((g.arming_check == ARMING_CHECK_ALL) || (g.arming_check & ARMING_CHECK_RC)) {
        // check throttle is not too low - must be above failsafe throttle
        if (g.failsafe_throttle != FS_THR_DISABLED && channel_throttle->get_radio_in() < g.failsafe_throttle_value) {
            if (display_failure) {
                #if FRAME_CONFIG == HELI_FRAME
                gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Collective below Failsafe");
                #else
                gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Throttle below Failsafe");
                #endif
            }
            return false;
        }

        // check throttle is not too high - skips checks if arming from GCS in Guided
        if (!(arming_from_gcs && control_mode == GUIDED)) {
            // above top of deadband is too always high
            if (channel_throttle->get_control_in() > get_takeoff_trigger_throttle()) {
                if (display_failure) {
                    #if FRAME_CONFIG == HELI_FRAME
                    gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Collective too high");
                    #else
                    gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Throttle too high");
                    #endif
                }
                return false;
            }
            // in manual modes throttle must be at zero
            if ((mode_has_manual_throttle(control_mode) || control_mode == DRIFT) && channel_throttle->get_control_in() > 0) {
                if (display_failure) {
                    #if FRAME_CONFIG == HELI_FRAME
                    gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Collective too high");
                    #else
                    gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Throttle too high");
                    #endif
                }
                return false;
            }
        }
    }

    if (hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED) {
        if (!BoardConfig.get_safety_enable()) {
            hal.rcout->force_safety_off();
        } else {
            if (display_failure) {
                gcs_send_text(MAV_SEVERITY_CRITICAL,"Arm: Safety Switch");
            }
            return false;
        }
    }

    // if we've gotten this far all is ok
    return true;
}
