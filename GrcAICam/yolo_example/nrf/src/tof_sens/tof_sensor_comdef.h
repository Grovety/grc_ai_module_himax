/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TOF_SENSOR_COMDEF_H
#define TOF_SENSOR_COMDEF_H

#include <stdint.h>

#define TOF_LOW_DEF_THRESHOLD_DEF 200
#define TOF_HIGH_DEF_THRESHOLD_DEF 400

#define TOF_AMODE_ROW_TOP_LEFT_X_DEF 0
#define TOF_AMODE_ROW_TOP_LEFT_Y_DEF 15
#define TOF_AMODE_ROW_BOTTOM_RIGHT_X_DEF 15
#define TOF_AMODE_ROW_BOTTOM_RIGHT_Y_DEF 0
#define TOF_TIMING_BUDGET_COMMON_DEF 25000
#define TOF_TIMING_BUDGET_AMODE_DEF 75000
#define TOF_AUTO_MODE_MSR_RERIOD_MS 1000

#define TOF_INIT_MODE TOF_MODE_AUTO_WAIT_FOR_OBJECTS
#define TOF_TMR_ZONE_ITER 4
#define TOF_TMR_TGLV_ITER 3
#define TOF_RES_NO_READY 0
#define TOF_RES_BAD_STAT 1
#define TOF_RES_DATA_OK 2

#define TOF_WORK_AUTO_MODE VL53L1_PRESETMODE_AUTONOMOUS
#define TOF_WORK_RANG_MODE VL53L1_PRESETMODE_LITE_RANGING
                                   ;
// SHORT: 0m ~ 1.3m, MEDIUM : 0m ~ 3m, LONG : 0m ~ 4m
#define TOF_WORK_DISTANCE VL53L1_DISTANCEMODE_LONG

#define TOF_PARAM_MAX_DISTANCE 8000
#define CHECK_RANGE(Val,Min,Max) ((Val >= Min) && (Val <= Max))

typedef struct {
    uint16_t low;
    uint16_t high;
} dist_t;

typedef struct {
    struct {
        uint8_t x;
        uint8_t y;
    } top_left;
    struct {
        uint8_t x;
        uint8_t y;
    } bottom_rigth;
} row_t;

typedef struct {
    dist_t dist_mm;
    row_t amode_row;
    unsigned iter_cnt;
    uint32_t timing_budget_common;
    uint32_t timing_budget_amode;
} tof_msr_param_t;

typedef struct {
    uint16_t low;
    uint16_t high;
} vl53l1_thresholds_t;

typedef enum {
    TOF_MODE_AUTO_WAIT_FOR_OBJECTS = 2,
    TOF_MODE_AUTO_WAIT_DISAPPERANCE = 3,
    TOF_MODE_MEAS_DISTANCE_TIMER = 4,
    TOF_MODE_DISABLED_BY_USER_CMD = 6,
    TOF_MODE_DISABLED_WHILE_RECOGN = 7,
    TOF_MODE_UNKNOWN
} tof_mode_t;

#endif // TOF_SENSOR_COMDEF_H
