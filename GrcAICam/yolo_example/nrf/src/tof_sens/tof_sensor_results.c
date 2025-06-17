/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mutex_wrapper.h"

#include "tof_sensor_results.h"

#define LOG_MODULE_NAME top_sensor_results
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

static tos_sens_result_t tos_sens_result_bigw;

/*const char* RangeStatusStr[16] = {
    "RNG_VALID",    // 0,RANGE_VALID-Ranging measurement is valid.
    "SIGM_FAIL",    // 1,SIGMA_FAIL-Raised if sigma estimator check is above the internal defined threshold.
    "SIGN_FAIL",    // 2,SIGNAL_FAIL-Raised in autonomous mode only if signal value is below the internally defined threshold.
    "RN_CLIPPED",   // 3,RANGE_VALID_MIN_RANG_E_CLIPPED-Raised in autonomous mode only if ranging is below a threshold defined in the tuning parameter.
    "OUT_BOUNDS",   // 4,OUTOFBOUNDS_FAIL-Raised when the range result is out of bounds.
    "HRDWR_FAIL",   // 5,HARDWARE_FAIL-Raised in case of hardware or VCSEL failure.
    "RV_NO_WRAP",   // 6,RANGE_VALID_NO_WRAP-No wraparound check has been done.
    "WRAPT_FAIL",   // 7,WRAP_TARGET_FAIL-Wrapped target not matching phases.
    "PROCS_FAIL",   // 8,PROCESSING_FAIL-Internal algorithm underflow or overflow in autonomous ranging.
    "XTALK_FAIL",   // 9,XTALK_SIGNAL_FAIL-Raised in autonomous modes when the signal is below the crosstalk threshold.
    "R_SYNC_INT",   // 10,RANGESTATUS_SYNCRONISATION_INT-Raised once after start, ranging value has to be ignored.
    "R_MERG_PLS",   // 11,RANGE_VALID_MERGED_PULSE-Raised in ranging and multizone scanning modes only. Range is OK, but the distance reported is the result of multiple targets merged together.
    "LACK_OF_SG",   // 12,TARGET_PRESENT_LACK_OF_SIGNAL-Indicates that there is a target, but the signal is too low to report ranging.
    "MIN_RN_FAIL",  // 13,MIN_RANGE_FAIL-Programmed ROI is not valid, selected ROI is out of the SPAD array.
    "INV_RNG_RES",  // 14,INVALID-The reported range is invalid, ignore it.
    "RNG_ST_NONE"   // 15,RANGESTATUS_NONE-Single ranging value: ranging is not updated, ignore reported value. Multiple ranging values: target not detected.
};*/

K_MUTEX_DEF(tof_sensor_results_mutex)

void tof_sens_results_init()
{
    K_MUTEX_INIT(tof_sensor_results_mutex)
    tos_sens_result_bigw.RangeMilliMeter = 0;
    tos_sens_result_bigw.RangeStatus = 15;
}

void tof_sens_results_bigw_get_distance(char* buf)
{
    K_MUTEX_LOCK(tof_sensor_results_mutex)
	sprintf(buf, "%d,%d", tos_sens_result_bigw.RangeStatus, tos_sens_result_bigw.RangeMilliMeter);
    K_MUTEX_UNLOCK(tof_sensor_results_mutex)
}

void tof_sens_results_bigw_update(const VL53L1_RangingMeasurementData_t* NewRangingData)
{
    K_MUTEX_LOCK(tof_sensor_results_mutex)

    //tos_sens_result_bigw.StreamCount = NewRangingData->StreamCount;
    //tos_sens_result_bigw.SignalRateRtnMegaCps = NewRangingData->SignalRateRtnMegaCps;
    //tos_sens_result_bigw.AmbientRateRtnMegaCps = NewRangingData->AmbientRateRtnMegaCps;
    //tos_sens_result_bigw.EffectiveSpadRtnCount = NewRangingData->EffectiveSpadRtnCount;
    //tos_sens_result_bigw.SigmaMilliMeter = NewRangingData->SigmaMilliMeter;
    tos_sens_result_bigw.RangeMilliMeter = NewRangingData->RangeMilliMeter;
    tos_sens_result_bigw.RangeStatus = NewRangingData->RangeStatus;     

    K_MUTEX_UNLOCK(tof_sensor_results_mutex)
}
