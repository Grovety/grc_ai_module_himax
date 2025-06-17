/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TOF_SENSOR_CORE_H
#define TOF_SENSOR_CORE_H

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "himax_comdef.h"
#include "leds_ctrl.h"
#include "tof_sensor_results.h"

#include "tof_sensor_comdef.h"

void tof_sensor_wakeup_prepare();

void tof_sensor_gpio_int_enable();

void tof_sensor_gpio_int_disable();

void tof_sensor_himax_i2c_test();

int tof_sensor_init();

tof_mode_t tof_sensor_get_mode();

void tof_sensor_set_user_mode(tof_mode_t new_mode);

void tof_sensor_set_auto_mode(tof_mode_t new_mode);

void tof_sensor_get_measure_param(char* buf);

bool tof_sensor_set_distance(int low, int high);

bool tof_sensor_set_amode_roi(int top_left_x, int top_left_y, int bot_rigth_x, int bot_rigth_y);

bool tof_sensor_set_timing_budgets(int timing_budgets_common, int timing_budgets_amode);

#endif // TOF_SENSOR_CORE_H
