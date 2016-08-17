/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

#include "AP_NavEKF2.h"
#include "AP_NavEKF2_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;

static bool target_pos_set;
/********************************************************
*                   RESET FUNCTIONS                     *
********************************************************/

/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/

// select fusion of Visual Position measurements
void NavEKF2_core::SelectVisPosFusion()
{
    if(core_index != 0) {
        return;
    }
    // Check if the magnetometer has been fused on that time step and the filter is running at faster than 200 Hz
    // If so, don't fuse measurements on this time step to reduce frame over-runs
    // Only allow one time slip to prevent high rate magnetometer data preventing fusion of other measurements
    if (magFusePerformed && dtIMUavg < 0.005f && !visPosFusionDelayed) {
        visPosFusionDelayed = true;
        return;
    } else {
        visPosFusionDelayed = false;
    }

    // start performance timer
    hal.util->perf_begin(_perf_FuseVisPos);
    // Perform Data Checks
    // Check if the visPos data is still valid
    // visPosDataValid = ((imuSampleTime_ms - visPosValidMeaTime_ms) < 1000);
    // Check if the visPos sensor has timed out
    bool visPosSensorTimeout = ((imuSampleTime_ms - visPosValidMeaTime_ms) > 500);
    // Check if the fusion has timed out (visPos measurements have been rejected for too long)
    bool visPosFusionTimeout = ((imuSampleTime_ms - prevVisPosFuseTime_ms) > 500);

    visPosDataValid = !(visPosSensorTimeout || visPosFusionTimeout) && target_pos_set;

    // Reset Position if vispos data is not available for sometime
    if ((visPosSensorTimeout || visPosFusionTimeout) && target_pos_set) {
        target_pos_set = false;
    }

    // If the visPos measurements have been rejected for too long and we are relying on them, then revert to constant position mode
    if ((visPosSensorTimeout || visPosFusionTimeout) && PV_AidingMode == AID_VISPOS) {
        PV_AidingMode = AID_NONE;
        // store the current position to be used to as a sythetic position measurement
        lastKnownPositionNE.x = stateStruct.position.x;
        lastKnownPositionNE.y = stateStruct.position.y;
        // reset the position
        ResetPosition();
    }
    // Fuse visPos data into the main filter if not excessively tilted and we are in the correct mode
    if (visPosDataToFuse && (PV_AidingMode == AID_VISPOS || PV_AidingMode == AID_ABSOLUTE))
    {
        // Set the visPos noise used by the fusion processes
        R_LPOS = sq(MAX(frontend->_visPosNoise, 0.001f));
        // Fuse the visPos X and Y axis data into the main filter sequentially
        FuseVisPos();
        // reset flag to indicate that no new vispos data is available for fusion
        visPosDataToFuse = false;
    }

    // stop the performance timer
    hal.util->perf_end(_perf_FuseVisPos);
}

/*
*/
void NavEKF2_core::FuseVisPos()
{
    Vector24 H_LPOS;
    Vector3 lpos;
    // Copy required states to local variable names
    float q0  = stateStruct.quat[0];
    float q1 = stateStruct.quat[1];
    float q2 = stateStruct.quat[2];
    float q3 = stateStruct.quat[3];
    float pn = stateStruct.position.x;
    float pe = stateStruct.position.y;
    float pd = stateStruct.position.z;
    debug_pos = Tnb_vispos*visPosDataDelayed.pos;
    // calculate relative position in sensor frame
    stateStruct.quat.rotation_matrix(Tnb_vispos);
    if(!target_pos_set) {
        //if(stateStruct.position.z > -1.0f) {     //do not initialise if altitude is below half meter
        //    return;
        //}
        target_pos_ef.zero();
        hal.console->printf("Reset VPS Pos\n");
        target_pos_ef = Tnb_vispos*visPosDataDelayed.pos+stateStruct.position;
        target_pos_set = true;
    } /*else {
        //gps pos reset
        Vector2f gps_pos;
        float baroalt, gpsalt; 
        gps_pos.x = gpsDataNew.pos.x;
        gps_pos.y = gpsDataNew.pos.y;
        visPosGPSAltReset = gpsDataNew.hgt + stateStruct.position.z;
        visPosBaroAltReset = baroDataNew.hgt - baroHgtOffset + stateStruct.position.z;
        visPosGPSReset = gps_pos - Vector2f(stateStruct.position.x,stateStruct.position.y);
    }*/
    //Vector3f meas_abs_pos = target_pos_ef-Tnb_vispos*visPosDataDelayed.pos;
    Vector3f pos_bf = (Tnb_vispos.transposed())*(target_pos_ef-stateStruct.position);
    lpos[0] = pos_bf.x;
    lpos[1] = pos_bf.y;
    lpos[2] = pos_bf.z;
    //hal.console->printf("\n");
    // Fuse X and Y axis measurements sequentially assuming observation errors are uncorrelated
    for (int8_t obsIndex=0; obsIndex<=2; obsIndex++) { // fuse X axis data first
        // calculate observation jacobians and Kalman gains
        memset(&H_LPOS[0], 0, sizeof(H_LPOS));
        if (obsIndex == 0) {
            float t2 = q0*q0;
            float t3 = q1*q1;
            float t4 = q2*q2;
            float t5 = q3*q3;
            float t6 = q0*q1*2.0f;
            float t7 = q0*q3*2.0f;
            float t8 = q0*q2*2.0f;
            float t9 = q1*q3*2.0f;
            H_LPOS[1] = pn*(t8+t9)+pd*(t2-t3-t4+t5)-pe*(t6-q2*q3*2.0f);
            H_LPOS[2] = -pe*(t2-t3+t4-t5)-pd*(t6+q2*q3*2.0f)+pn*(t7-q1*q2*2.0f);
            H_LPOS[6] = -t2-t3+t4+t5;
            H_LPOS[7] = -t7-q1*q2*2.0f;
            H_LPOS[8] = t8-t9;

            t6 = q0*q2*2.0f;
            t7 = q0*q1*2.0f;
            t8 = q0*q3*2.0f;
            t9 = q1*q2*2.0f;
            float t10 = t8+t9;
            float t11 = t2+t3-t4-t5;
            float t12 = q1*q3*2.0f;
            float t13 = t2-t3-t4+t5;
            float t14 = pd*t13;
            float t15 = q2*q3*2.0f;
            float t16 = t6+t12;
            float t17 = pn*t16;
            float t18 = t2-t3+t4-t5;
            float t19 = pe*t18;
            float t20 = t7+t15;
            float t21 = pd*t20;
            float t22 = t8-t9;
            float t27 = pn*t22;
            float t23 = t19+t21-t27;
            float t24 = t6-t12;
            float t25 = t7-t15;
            float t28 = pe*t25;
            float t26 = t14+t17-t28;
            float t29 = P[6][7]*t11;
            float t30 = P[7][7]*t10;
            float t31 = P[2][7]*t23;
            float t55 = P[8][7]*t24;
            float t56 = P[1][7]*t26;
            float t32 = t29+t30+t31-t55-t56;
            float t33 = t10*t32;
            float t34 = P[6][8]*t11;
            float t35 = P[7][8]*t10;
            float t36 = P[2][8]*t23;
            float t57 = P[8][8]*t24;
            float t58 = P[1][8]*t26;
            float t37 = t34+t35+t36-t57-t58;
            float t38 = P[6][1]*t11;
            float t39 = P[7][1]*t10;
            float t40 = P[2][1]*t23;
            float t54 = P[1][1]*t26;
            float t60 = P[8][1]*t24;
            float t41 = t38+t39+t40-t54-t60;
            float t42 = P[6][2]*t11;
            float t43 = P[7][2]*t10;
            float t44 = P[2][2]*t23;
            float t62 = P[8][2]*t24;
            float t63 = P[1][2]*t26;
            float t45 = t42+t43+t44-t62-t63;
            float t46 = t23*t45;
            float t47 = P[6][6]*t11;
            float t48 = P[7][6]*t10;
            float t49 = P[2][6]*t23;
            float t64 = P[8][6]*t24;
            float t65 = P[1][6]*t26;
            float t50 = t47+t48+t49-t64-t65;
            float t51 = t11*t50;
            float t59 = t24*t37;
            float t61 = t26*t41;
            float t52 = R_LPOS+t33+t46+t51-t59-t61;
            float t53 = 1.0f/t52;

            // calculate innovation variance for X axis observation and protect against a badly conditioned calculation
            if (t52 > R_LPOS) {
                t53 = 1.0f/t52;
                faultStatus.bad_xvispos = false;
            } else {
                t52= 0.0f;
                t53 = 1.0f/R_LPOS;
                faultStatus.bad_xvispos = true;
                return;
            }
            varInnovVisPos[0] = t52;

            // calculate innovation for X axis observation
            innovVisPos[0] = lpos[0] - visPosDataDelayed.pos.x;

            // calculate Kalman gains for X-axis observation
            Kfusion[0] = -t53*(P[0][6]*t11+P[0][7]*t10+P[0][2]*t23-P[0][1]*t26-P[0][8]*t24);
            Kfusion[1] = -t53*(-t54+P[1][6]*t11+P[1][7]*t10+P[1][2]*t23-P[1][8]*t24);
            Kfusion[2] = -t53*(t44+P[2][6]*t11+P[2][7]*t10-P[2][1]*t26-P[2][8]*t24);
            Kfusion[3] = -t53*(P[3][6]*t11+P[3][7]*t10+P[3][2]*t23-P[3][1]*t26-P[3][8]*t24);
            Kfusion[4] = -t53*(P[4][6]*t11+P[4][7]*t10+P[4][2]*t23-P[4][1]*t26-P[4][8]*t24);
            Kfusion[5] = -t53*(P[5][6]*t11+P[5][7]*t10+P[5][2]*t23-P[5][1]*t26-P[5][8]*t24);
            Kfusion[6] = -t53*(t47+P[6][7]*t10+P[6][2]*t23-P[6][1]*t26-P[6][8]*t24);
            Kfusion[7] = -t53*(t30+P[7][6]*t11+P[7][2]*t23-P[7][1]*t26-P[7][8]*t24);
            Kfusion[8] = -t53*(-t57+P[8][6]*t11+P[8][7]*t10+P[8][2]*t23-P[8][1]*t26);
            Kfusion[9] = -t53*(P[9][6]*t11+P[9][7]*t10+P[9][2]*t23-P[9][1]*t26-P[9][8]*t24);
            Kfusion[10] = -t53*(P[10][6]*t11+P[10][7]*t10+P[10][2]*t23-P[10][1]*t26-P[10][8]*t24);
            Kfusion[11] = -t53*(P[11][6]*t11+P[11][7]*t10+P[11][2]*t23-P[11][1]*t26-P[11][8]*t24);
            Kfusion[12] = -t53*(P[12][6]*t11+P[12][7]*t10+P[12][2]*t23-P[12][1]*t26-P[12][8]*t24);
            Kfusion[13] = -t53*(P[13][6]*t11+P[13][7]*t10+P[13][2]*t23-P[13][1]*t26-P[13][8]*t24);
            Kfusion[14] = -t53*(P[14][6]*t11+P[14][7]*t10+P[14][2]*t23-P[14][1]*t26-P[14][8]*t24);
            Kfusion[15] = -t53*(P[15][6]*t11+P[15][7]*t10+P[15][2]*t23-P[15][1]*t26-P[15][8]*t24);
            if (!inhibitWindStates) {
                Kfusion[22] = -t53*(P[22][6]*t11+P[22][7]*t10+P[22][2]*t23-P[22][1]*t26-P[22][8]*t24);
                Kfusion[23] = -t53*(P[23][6]*t11+P[23][7]*t10+P[23][2]*t23-P[23][1]*t26-P[23][8]*t24);
            } else {
                Kfusion[22] = 0.0f;
                Kfusion[23] = 0.0f;
            }
            if (!inhibitMagStates) {
                Kfusion[16] = -t53*(P[16][6]*t11+P[16][7]*t10+P[16][2]*t23-P[16][1]*t26-P[16][8]*t24);
                Kfusion[17] = -t53*(P[17][6]*t11+P[17][7]*t10+P[17][2]*t23-P[17][1]*t26-P[17][8]*t24);
                Kfusion[18] = -t53*(P[18][6]*t11+P[18][7]*t10+P[18][2]*t23-P[18][1]*t26-P[18][8]*t24);
                Kfusion[19] = -t53*(P[19][6]*t11+P[19][7]*t10+P[19][2]*t23-P[19][1]*t26-P[19][8]*t24);
                Kfusion[20] = -t53*(P[20][6]*t11+P[20][7]*t10+P[20][2]*t23-P[20][1]*t26-P[20][8]*t24);
                Kfusion[21] = -t53*(P[21][6]*t11+P[21][7]*t10+P[21][2]*t23-P[21][1]*t26-P[21][8]*t24);
            } else {
                for (uint8_t i = 16; i <= 21; i++) {
                    Kfusion[i] = 0.0f;
                }
            }

        } else if (obsIndex == 1) {
            float t2 = q0*q0;
            float t3 = q1*q1;
            float t4 = q2*q2;
            float t5 = q3*q3;
            float t6 = q0*q2*2.0f;
            float t7 = q1*q3*2.0f;
            float t8 = q0*q3*2.0f;
            float t9 = q1*q2*2.0f;
            float t10 = q0*q1*2.0f;
            H_LPOS[0] = -pn*(t6+t7)-pd*(t2-t3-t4+t5)+pe*(t10-q2*q3*2.0d);
            H_LPOS[2] = pe*(t8+t9)-pd*(t6-t7)+pn*(t2+t3-t4-t5);
            H_LPOS[6] = t8-t9;
            H_LPOS[7] = -t2+t3-t4+t5;
            H_LPOS[8] = -t10-q2*q3*2.0f;

            t6 = q0*q1*2.0f;
            t7 = q2*q3*2.0f;
            t8 = q0*q2*2.0f;
            t9 = q1*q3*2.0f;
            t10 = q0*q3*2.0f;
            float t11 = q1*q2*2.0f;
            float t12 = t2-t3+t4-t5;
            float t13 = t10-t11;
            float t14 = t6+t7;
            float t15 = t2-t3-t4+t5;
            float t16 = pd*t15;
            float t17 = t6-t7;
            float t18 = t8+t9;
            float t19 = pn*t18;
            float t27 = pe*t17;
            float t20 = t16+t19-t27;
            float t21 = t2+t3-t4-t5;
            float t22 = pn*t21;
            float t23 = t8-t9;
            float t24 = t10+t11;
            float t25 = pe*t24;
            float t28 = pd*t23;
            float t26 = t22+t25-t28;
            float t29 = P[0][0]*t20;
            float t30 = P[7][6]*t12;
            float t31 = P[8][6]*t14;
            float t32 = P[0][6]*t20;
            float t55 = P[6][6]*t13;
            float t56 = P[2][6]*t26;
            float t33 = t30+t31+t32-t55-t56;
            float t34 = P[7][8]*t12;
            float t35 = P[8][8]*t14;
            float t36 = P[0][8]*t20;
            float t58 = P[6][8]*t13;
            float t59 = P[2][8]*t26;
            float t37 = t34+t35+t36-t58-t59;
            float t38 = t14*t37;
            float t39 = P[7][0]*t12;
            float t40 = P[8][0]*t14;
            float t60 = P[6][0]*t13;
            float t61 = P[2][0]*t26;
            float t41 = t29+t39+t40-t60-t61;
            float t42 = t20*t41;
            float t43 = P[7][2]*t12;
            float t44 = P[8][2]*t14;
            float t45 = P[0][2]*t20;
            float t54 = P[2][2]*t26;
            float t62 = P[6][2]*t13;
            float t46 = t43+t44+t45-t54-t62;
            float t47 = P[7][7]*t12;
            float t48 = P[8][7]*t14;
            float t49 = P[0][7]*t20;
            float t64 = P[6][7]*t13;
            float t65 = P[2][7]*t26;
            float t50 = t47+t48+t49-t64-t65;
            float t51 = t12*t50;
            float t57 = t13*t33;
            float t63 = t26*t46;
            float t52 = R_LPOS+t38+t42+t51-t57-t63;
            float t53 = 1.0f/t52;

            // calculate innovation variance for X axis observation and protect against a badly conditioned calculation
            if (t52 > R_LPOS) {
                t53 = 1.0f/t52;
                faultStatus.bad_yvispos = false;
            } else {
                t52 = 0.0f;
                t53 = 1.0f/R_LPOS;
                faultStatus.bad_yvispos = true;
                return;
            }
            varInnovVisPos[1] = t52;

            // calculate innovation for Y observation
            innovVisPos[1] = lpos[1] - visPosDataDelayed.pos.y;

            // calculate Kalman gains for the Y-axis observation
            Kfusion[0] = -t53*(t29-P[0][6]*t13+P[0][7]*t12+P[0][8]*t14-P[0][2]*t26);
            Kfusion[1] = -t53*(-P[1][6]*t13+P[1][7]*t12+P[1][0]*t20+P[1][8]*t14-P[1][2]*t26);
            Kfusion[2] = -t53*(-t54-P[2][6]*t13+P[2][7]*t12+P[2][0]*t20+P[2][8]*t14);
            Kfusion[3] = -t53*(-P[3][6]*t13+P[3][7]*t12+P[3][0]*t20+P[3][8]*t14-P[3][2]*t26);
            Kfusion[4] = -t53*(-P[4][6]*t13+P[4][7]*t12+P[4][0]*t20+P[4][8]*t14-P[4][2]*t26);
            Kfusion[5] = -t53*(-P[5][6]*t13+P[5][7]*t12+P[5][0]*t20+P[5][8]*t14-P[5][2]*t26);
            Kfusion[6] = -t53*(-t55+P[6][7]*t12+P[6][0]*t20+P[6][8]*t14-P[6][2]*t26);
            Kfusion[7] = -t53*(t47-P[7][6]*t13+P[7][0]*t20+P[7][8]*t14-P[7][2]*t26);
            Kfusion[8] = -t53*(t35-P[8][6]*t13+P[8][7]*t12+P[8][0]*t20-P[8][2]*t26);
            Kfusion[9] = -t53*(-P[9][6]*t13+P[9][7]*t12+P[9][0]*t20+P[9][8]*t14-P[9][2]*t26);
            Kfusion[10] = -t53*(-P[10][6]*t13+P[10][7]*t12+P[10][0]*t20+P[10][8]*t14-P[10][2]*t26);
            Kfusion[11] = -t53*(-P[11][6]*t13+P[11][7]*t12+P[11][0]*t20+P[11][8]*t14-P[11][2]*t26);
            Kfusion[12] = -t53*(-P[12][6]*t13+P[12][7]*t12+P[12][0]*t20+P[12][8]*t14-P[12][2]*t26);
            Kfusion[13] = -t53*(-P[13][6]*t13+P[13][7]*t12+P[13][0]*t20+P[13][8]*t14-P[13][2]*t26);
            Kfusion[14] = -t53*(-P[14][6]*t13+P[14][7]*t12+P[14][0]*t20+P[14][8]*t14-P[14][2]*t26);
            Kfusion[15] = -t53*(-P[15][6]*t13+P[15][7]*t12+P[15][0]*t20+P[15][8]*t14-P[15][2]*t26);
            if (!inhibitWindStates) {
                Kfusion[22] = -t53*(-P[22][6]*t13+P[22][7]*t12+P[22][0]*t20+P[22][8]*t14-P[22][2]*t26);
                Kfusion[23] = -t53*(-P[23][6]*t13+P[23][7]*t12+P[23][0]*t20+P[23][8]*t14-P[23][2]*t26);
            } else {
                Kfusion[22] = 0.0f;
                Kfusion[23] = 0.0f;
            }
            if (!inhibitMagStates) {
                Kfusion[16] = -t53*(-P[16][6]*t13+P[16][7]*t12+P[16][0]*t20+P[16][8]*t14-P[16][2]*t26);
                Kfusion[17] = -t53*(-P[17][6]*t13+P[17][7]*t12+P[17][0]*t20+P[17][8]*t14-P[17][2]*t26);
                Kfusion[18] = -t53*(-P[18][6]*t13+P[18][7]*t12+P[18][0]*t20+P[18][8]*t14-P[18][2]*t26);
                Kfusion[19] = -t53*(-P[19][6]*t13+P[19][7]*t12+P[19][0]*t20+P[19][8]*t14-P[19][2]*t26);
                Kfusion[20] = -t53*(-P[20][6]*t13+P[20][7]*t12+P[20][0]*t20+P[20][8]*t14-P[20][2]*t26);
                Kfusion[21] = -t53*(-P[21][6]*t13+P[21][7]*t12+P[21][0]*t20+P[21][8]*t14-P[21][2]*t26);
            } else {
                for (uint8_t i = 16; i <= 21; i++) {
                    Kfusion[i] = 0.0f;
                }
            }
        }  else {
            float t2 = q0*q0;
            float t3 = q1*q1;
            float t4 = q2*q2;
            float t5 = q3*q3;
            float t6 = q0*q3*2.0;
            float t7 = q0*q2*2.0;
            float t8 = q0*q1*2.0;
            float t9 = q2*q3*2.0;
            H_LPOS[0] = pd*(t8+t9)+pe*(t2-t3+t4-t5)-pn*(t6-q1*q2*2.0);
            H_LPOS[1] = -pn*(t2+t3-t4-t5)+pd*(t7-q1*q3*2.0)-pe*(t6+q1*q2*2.0);
            H_LPOS[6] = -t7-q1*q3*2.0;
            H_LPOS[7] = t8-t9;
            H_LPOS[8] = -t2+t3+t4-t5;

            t2 = q0*q0;
            t3 = q1*q1;
            t4 = q2*q2;
            t5 = q3*q3;
            t6 = q0*q1*2.0;
            t7 = q0*q2*2.0;
            t8 = q1*q3*2.0;
            t9 = q0*q3*2.0;
            float t10 = t7+t8;
            float t11 = t2-t3-t4+t5;
            float t12 = q2*q3*2.0;
            float t13 = t2-t3+t4-t5;
            float t14 = pe*t13;
            float t15 = t6+t12;
            float t16 = pd*t15;
            float t17 = q1*q2*2.0;
            float t18 = t2+t3-t4-t5;
            float t19 = pn*t18;
            float t20 = t7-t8;
            float t21 = t9+t17;
            float t22 = pe*t21;
            float t27 = pd*t20;
            float t23 = t19+t22-t27;
            float t24 = t6-t12;
            float t25 = t9-t17;
            float t28 = pn*t25;
            float t26 = t14+t16-t28;
            float t29 = P[1][1]*t23;
            float t30 = P[8][6]*t11;
            float t31 = P[6][6]*t10;
            float t32 = P[1][6]*t23;
            float t54 = P[7][6]*t24;
            float t55 = P[0][6]*t26;
            float t33 = t30+t31+t32-t54-t55;
            float t34 = t10*t33;
            float t35 = P[8][7]*t11;
            float t36 = P[6][7]*t10;
            float t37 = P[1][7]*t23;
            float t56 = P[7][7]*t24;
            float t57 = P[0][7]*t26;
            float t38 = t35+t36+t37-t56-t57;
            float t39 = P[8][0]*t11;
            float t40 = P[6][0]*t10;
            float t41 = P[1][0]*t23;
            float t59 = P[7][0]*t24;
            float t60 = P[0][0]*t26;
            float t42 = t39+t40+t41-t59-t60;
            float t43 = P[8][1]*t11;
            float t44 = P[6][1]*t10;
            float t62 = P[7][1]*t24;
            float t63 = P[0][1]*t26;
            float t45 = t29+t43+t44-t62-t63;
            float t46 = t23*t45;
            float t47 = P[8][8]*t11;
            float t48 = P[6][8]*t10;
            float t49 = P[1][8]*t23;
            float t64 = P[7][8]*t24;
            float t65 = P[0][8]*t26;
            float t50 = t47+t48+t49-t64-t65;
            float t51 = t11*t50;
            float t58 = t24*t38;
            float t61 = t26*t42;
            float t52 = R_LPOS+t34+t46+t51-t58-t61;
            float t53 = 1.0/t52;
            // calculate innovation variance for X axis observation and protect against a badly conditioned calculation
            if (t52 > R_LPOS) {
                t53 = 1.0f/t52;
                faultStatus.bad_yvispos = false;
            } else {
                t52 = 0.0f;
                t53 = 1.0f/R_LPOS;
                faultStatus.bad_yvispos = true;
                return;
            }
            varInnovVisPos[2] = t52;

            // calculate innovation for Y observation
            innovVisPos[2] = lpos[2] - visPosDataDelayed.pos.z;

            // calculate Kalman gains for the Y-axis observation
            Kfusion[0] = -t53*(P[0][6]*t10+P[0][8]*t11+P[0][1]*t23-P[0][0]*t26-P[0][7]*t24);
            Kfusion[1] = -t53*(t29+P[1][6]*t10+P[1][8]*t11-P[1][0]*t26-P[1][7]*t24);
            Kfusion[2] = -t53*(P[2][6]*t10+P[2][8]*t11+P[2][1]*t23-P[2][0]*t26-P[2][7]*t24);
            Kfusion[3] = -t53*(P[3][6]*t10+P[3][8]*t11+P[3][1]*t23-P[3][0]*t26-P[3][7]*t24);
            Kfusion[4] = -t53*(P[4][6]*t10+P[4][8]*t11+P[4][1]*t23-P[4][0]*t26-P[4][7]*t24);
            Kfusion[5] = -t53*(P[5][6]*t10+P[5][8]*t11+P[5][1]*t23-P[5][0]*t26-P[5][7]*t24);
            Kfusion[6] = -t53*(t31+P[6][8]*t11+P[6][1]*t23-P[6][0]*t26-P[6][7]*t24);
            Kfusion[7] = -t53*(-t56+P[7][6]*t10+P[7][8]*t11+P[7][1]*t23-P[7][0]*t26);
            Kfusion[8] = -t53*(t47+P[8][6]*t10+P[8][1]*t23-P[8][0]*t26-P[8][7]*t24);
            Kfusion[9] = -t53*(P[9][6]*t10+P[9][8]*t11+P[9][1]*t23-P[9][0]*t26-P[9][7]*t24);
            Kfusion[10] = -t53*(P[10][6]*t10+P[10][8]*t11+P[10][1]*t23-P[10][0]*t26-P[10][7]*t24);
            Kfusion[11] = -t53*(P[11][6]*t10+P[11][8]*t11+P[11][1]*t23-P[11][0]*t26-P[11][7]*t24);
            Kfusion[12] = -t53*(P[12][6]*t10+P[12][8]*t11+P[12][1]*t23-P[12][0]*t26-P[12][7]*t24);
            Kfusion[13] = -t53*(P[13][6]*t10+P[13][8]*t11+P[13][1]*t23-P[13][0]*t26-P[13][7]*t24);
            Kfusion[14] = -t53*(P[14][6]*t10+P[14][8]*t11+P[14][1]*t23-P[14][0]*t26-P[14][7]*t24);
            Kfusion[15] = -t53*(P[15][6]*t10+P[15][8]*t11+P[15][1]*t23-P[15][0]*t26-P[15][7]*t24);
            if (!inhibitWindStates) {
                Kfusion[22] = -t53*(P[22][6]*t10+P[22][8]*t11+P[22][1]*t23-P[22][0]*t26-P[22][7]*t24);
                Kfusion[23] = -t53*(P[23][6]*t10+P[23][8]*t11+P[23][1]*t23-P[23][0]*t26-P[23][7]*t24);
            } else {
                Kfusion[22] = 0.0f;
                Kfusion[23] = 0.0f;
            }
            if (!inhibitMagStates) {
                Kfusion[16] = -t53*(P[16][6]*t10+P[16][8]*t11+P[16][1]*t23-P[16][0]*t26-P[16][7]*t24);
                Kfusion[17] = -t53*(P[17][6]*t10+P[17][8]*t11+P[17][1]*t23-P[17][0]*t26-P[17][7]*t24);
                Kfusion[18] = -t53*(P[18][6]*t10+P[18][8]*t11+P[18][1]*t23-P[18][0]*t26-P[18][7]*t24);
                Kfusion[19] = -t53*(P[19][6]*t10+P[19][8]*t11+P[19][1]*t23-P[19][0]*t26-P[19][7]*t24);
                Kfusion[20] = -t53*(P[20][6]*t10+P[20][8]*t11+P[20][1]*t23-P[20][0]*t26-P[20][7]*t24);
                Kfusion[21] = -t53*(P[21][6]*t10+P[21][8]*t11+P[21][1]*t23-P[21][0]*t26-P[21][7]*t24);
            } else {
                for (uint8_t i = 16; i <= 21; i++) {
                    Kfusion[i] = 0.0f;
                }
            }
        }
        if(obsIndex == 0){
            H_LPOS[0] = 0.0f;
            //for(uint8_t i=0; i<=2; i++)
                //hal.console->printf("%f ",H_LPOS[i]);
            //for(uint8_t i=6; i<=8; i++)
                //hal.console->printf("%f ",H_LPOS[i]);                
            //hal.console->printf("\n");
        } else if(obsIndex == 1){
            H_LPOS[1] = 0.0f;
        } else {
            H_LPOS[2] = 0.0f;
        }

        // calculate the innovation consistency test ratio
        visPosTestRatio[obsIndex] = sq(innovVisPos[obsIndex]) / (sq(MAX(0.01f * (float)frontend->_visPosInnovGate, 1.0f)) * varInnovVisPos[obsIndex]);
        // Check the innovation for consistency and don't fuse if out of bounds
        if (visPosTestRatio[obsIndex] < 1.0f) {
            // record the last time observations were accepted for fusion
            prevVisPosFuseTime_ms = imuSampleTime_ms;

            // correct the covariance P = (I - K*H)*P
            // take advantage of the empty columns in KH to reduce the
            // number of operations
            for (unsigned i = 0; i<=stateIndexLim; i++) {
                for (unsigned j = 0; j<=2; j++) {
                    KH[i][j] = Kfusion[i] * H_LPOS[j];
                }
                for (unsigned j = 3; j<=5; j++) {
                    KH[i][j] = 0.0f;
                }
                for (unsigned j = 6; j<=8; j++) {
                    KH[i][j] = Kfusion[i] * H_LPOS[j];
                }
                for (unsigned j = 9; j<=23; j++) {
                    KH[i][j] = 0.0f;
                }
            }
            for (unsigned j = 0; j<=stateIndexLim; j++) {
                for (unsigned i = 0; i<=stateIndexLim; i++) {
                    ftype res = 0;
                    res += KH[i][0] * P[0][j];
                    res += KH[i][1] * P[1][j];
                    res += KH[i][2] * P[2][j];
                    res += KH[i][6] * P[6][j];
                    res += KH[i][7] * P[7][j];
                    res += KH[i][8] * P[8][j];
                    KHP[i][j] = res;
                }
            }

            // Check that we are not going to drive any variances negative and skip the update if so
            bool healthyFusion = true;
            for (uint8_t i= 0; i<=stateIndexLim; i++) {
                if (KHP[i][i] > P[i][i]) {
                    healthyFusion = false;
                }
            }

            if (healthyFusion) {
                // update the covariance matrix
                for (uint8_t i= 0; i<=stateIndexLim; i++) {
                    for (uint8_t j= 0; j<=stateIndexLim; j++) {
                        P[i][j] = P[i][j] - KHP[i][j];
                    }
                }

                // force the covariance matrix to be symmetrical and limit the variances to prevent ill-condiioning.
                ForceSymmetry();
                ConstrainVariances();

                // zero the attitude error state - by definition it is assumed to be zero before each observaton fusion
                stateStruct.angErr.zero();

                // correct the state vector
                for (uint8_t j= 0; j<=stateIndexLim; j++) {
                    statesArray[j] = statesArray[j] - Kfusion[j] * innovVisPos[obsIndex];
                }
                // the first 3 states represent the angular misalignment vector. This is
                // is used to correct the estimated quaternion on the current time step
                stateStruct.quat.rotate(stateStruct.angErr);
            } else {
                // record bad axis
                if (obsIndex == 0) {
                    faultStatus.bad_xvispos = true;
                } else if (obsIndex == 1) {
                    faultStatus.bad_yvispos = true;
                }

            }
        }
    }
    //if(core_index == 0)
    //    hal.console->printf("VisPos Innov: %f %f \n", innovVisPos[0], innovVisPos[1]);

}

/********************************************************
*                   MISC FUNCTIONS                      *
********************************************************/

#endif // HAL_CPU_CLASS