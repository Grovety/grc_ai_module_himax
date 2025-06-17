/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef HIMAX_CONTROL_H
#define HIMAX_CONTROL_H

#include <stdbool.h>

#include "himax_comdef.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HIMAX_MODE_NO_CHANGE = 0x00,
    HIMAX_MODE_CONT_WORK,
#ifdef USE_POWER_DOWN_MODES
    HIMAX_MODE_POWER_DOWN,
    HIMAX_MODE_DPOWER_DOWN,
#endif // USE_POWER_DOWN_MODES
    HIMAX_MODE_POWER_OFF
} himax_next_mode_t;

typedef enum {
    HIMAX_ADV_DO_NOTHING = 0x00,
    HIMAX_ADV_READ_IMAGE,
    HIMAX_ADV_STOP_RDIMG,
    HIMAX_ADV_RCGN_IMIT_ON,
    HIMAX_ADV_RCGN_IMIT_OFF
} himax_adv_action_t;

// #define HIMAX_IS_ALWAYS_ON

#ifdef HIMAX_IS_ALWAYS_ON // It's intended to be used in Scenario 3
    #define HIMAX_INIT_MODE HIMAX_MODE_CONT_WORK
    #define HIMAX_POFF_MODE HIMAX_MODE_CONT_WORK
#else
    #define HIMAX_INIT_MODE HIMAX_MODE_POWER_OFF
    #define HIMAX_POFF_MODE HIMAX_MODE_POWER_OFF
#endif

#define HIMAX_RESETN_TIME_MS 20
#define HIMAX_WAKEUP_TIME_MS 20
#define HIMAX_INIT_DELAY_MS 100
#define HIMAX_PCTL_DELAY_MS 500

int himax_ctrl_init();

bool himax_set_mode(himax_next_mode_t mode, himax_adv_action_t adv_action);

himax_next_mode_t himax_get_mode();

void himax_ctrl_spi_notify();

void himax_pwrctl(bool on);

#ifdef __cplusplus
}
#endif

#endif // HIMAX_CONTROL_H
