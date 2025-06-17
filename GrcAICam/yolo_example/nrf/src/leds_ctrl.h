/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LEDS_CTRL_H
#define LEDS_CTRL_H

#include <stdint.h>

// #define LED_TEST

#define LED_MIN 0
#define LED_MAX 255
#define LED_ON 64
#define LED_OFF 0

typedef enum {
    LEDR_UPDATE_EVENT = 1,
    LEDG_UPDATE_EVENT = 2,
    LEDB_UPDATE_EVENT = 4
} led_color_t;

typedef void (*led_ctrl_proc_t)(uint8_t);

int leds_init();

void leds_power_on();

void leds_power_off();

void led_red_ctrl(uint8_t level);

void led_green_ctrl(uint8_t level);

void led_blue_ctrl(uint8_t level);

#endif // LEDS_CTRL_H
