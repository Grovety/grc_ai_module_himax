/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

#include "detect_handler.h"
#include "leds_ctrl.h"

#define LOG_MODULE_NAME detect_handler
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define DETECT_HANDLER_THREAD_STACKSIZE 512
#define DETECT_HANDLER_THREAD_PRIORITY 7

static void detect_handler_thread_proc();

static K_THREAD_DEFINE(detect_handler_thread_id, DETECT_HANDLER_THREAD_STACKSIZE, detect_handler_thread_proc, NULL, NULL, NULL,
		DETECT_HANDLER_THREAD_PRIORITY, 0, 0);
static K_EVENT_DEFINE(detect_handler_start_event);

static void detect_handler_thread_proc()
{
    while (1) {
        if (k_event_wait(&detect_handler_start_event, 1, true, K_FOREVER)) {
            LOG_INF("!!!! EXPECTED OBJECT DETECTED !!!!");
            for (unsigned i = 0; i < 10; i++) {
                led_green_ctrl((i & 1) ? LED_OFF : LED_ON);
                k_sleep(K_MSEC(300));
            }
        }
    }
}

void detect_handler()
{
    k_event_set(&detect_handler_start_event, 1);
}


