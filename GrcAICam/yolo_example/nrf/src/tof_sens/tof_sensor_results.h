/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TOF_SENSOR_RESULTS_H
#define TOF_SENSOR_RESULTS_H

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "vl53l1_def.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TARGET_RCGN_IN_PROCGRESS = 0,       // not all zones data have processed yet 
    TARGET_RCGN_NOT_FOUND,              // target was not found
    TARGET_RCGN_IS_FOUND                // target is found
} target_recognition_result_t;

typedef struct {
    //uint8_t StreamCount;              // This 8-bit integer gives a counter incremented at each range. The value first starts at 0, incrementing to 255, and then incrementing from 128 to 255.
    //uint32_t SignalRateRtnMegaCps;    // this value is the return signal rate in MegaCountPer second (MCPS). This is a  16.16 fixed point value. To obtain a real value, it should be divided by 65536.
    //uint32_t AmbientRateRtnMegaCps;   // This value is the return ambient rate (in MCPS). It is a 16.16 fixed point value, which is effectively a measure of the infrared light. To obtain a real value, it should be divided by 65536.
    //uint16_t EffectiveSpadRtnCount;   // This is a16-bit integer that returns the effective SPAD count for the current ranging. To obtain a real value, it should be divided by 256. This 16.16 fixed point value is an estimation of the standard deviation of the current ranging, expressed in millimeters.
    //uint32_t SigmaMilliMeter;         // To obtain a real value, it should be divided by 65536.
    uint16_t RangeMilliMeter;           // This is a 16-bit integer giving the range distance in millimeters.
    uint8_t  RangeStatus;               // This is an 8-bit integer giving the range status for the current measurement. Value = 0 means that the ranging is valid.
} tos_sens_result_t;

void tof_sens_results_init();

void tof_sens_results_bigw_get_distance(char* buf); // big windows

void tof_sens_results_bigw_update(const VL53L1_RangingMeasurementData_t* RangingData);

#ifdef __cplusplus
}
#endif

#endif // TOF_SENSOR_RESULTS_H
