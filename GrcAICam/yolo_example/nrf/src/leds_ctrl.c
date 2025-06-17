/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include "leds_ctrl.h"

LOG_MODULE_REGISTER(leds_ctrl);

#ifdef LED_TEST
static void led_test();
#endif

#define LEDS_CONTROL DT_PATH(zephyr_user)
const struct gpio_dt_spec led_enable = GPIO_DT_SPEC_GET(LEDS_CONTROL, led_enbl_gpios);

#define STRIP_NODE DT_CHOSEN(led_strip)
#define STRIP_NUM_PIXELS DT_PROP(STRIP_NODE, chain_length)
static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);
static struct led_rgb led_color;

static void leds_update_thread_proc();
static K_THREAD_DEFINE(leds_update_thread, 512, leds_update_thread_proc, 0, 0, 0, 10, 0, -1);
static K_EVENT_DEFINE(leds_update_event);

int leds_init()
{
    if (!device_is_ready(strip)) {
        LOG_ERR("LED strip device %s is not ready", strip->name);
        return -1;
    }
    gpio_pin_configure_dt(&led_enable, GPIO_OUTPUT_HIGH);
    led_strip_update_rgb(strip, &led_color, STRIP_NUM_PIXELS);
    k_thread_start(leds_update_thread);
#ifdef LED_TEST
    led_test();
#endif // LED_TEST
    return 0;
}

void leds_power_on()
{
    gpio_pin_set_dt(&led_enable, 1);
}

void leds_power_off()
{
    gpio_pin_set_dt(&led_enable, 0);
}

static void leds_update_thread_proc()
{
    while (1) {
        uint32_t events = k_event_wait(&leds_update_event, 7, false, K_FOREVER);
        led_strip_update_rgb(strip, &led_color, STRIP_NUM_PIXELS);
        k_event_clear(&leds_update_event, events);
    }
}

void led_red_ctrl(uint8_t level)
{
    led_color.r = level;
    k_event_post(&leds_update_event, LEDR_UPDATE_EVENT);
}

void led_green_ctrl(uint8_t level)
{
    led_color.g = level;
    k_event_post(&leds_update_event, LEDG_UPDATE_EVENT);
}

void led_blue_ctrl(uint8_t level)
{
    led_color.b = level;
    k_event_post(&leds_update_event, LEDB_UPDATE_EVENT);
}

#ifdef LED_TEST

static void led_test()
{
    led_ctrl_proc_t led_proc[3] = {led_red_ctrl, led_green_ctrl, led_blue_ctrl};
    int i,j,k;
    k_sleep(K_MSEC(100));
    while(1) {
        for (i=0; i<3; i++) {       // turn on-turn off
            for (j=0; j<3; j++) {
                for (k=0; k<3; k++) {
                    led_proc[i](k ? LED_OFF : LED_ON);
                    k_sleep(K_MSEC(750));
                }
            }
        }
        for (i=0; i<3; i++) {       // turn on all
            led_proc[i](LED_ON);
        }
        k_sleep(K_MSEC(1500));
        for (i=0; i<3; i++) {       // turn off all
            led_proc[i](LED_OFF);
        }
        for (i=0; i<3; i++) {       // change brightness smoothly
            for (j=0; j<10; j++) {
                for (k=0; k<LED_MAX; k++) {
                    led_proc[i](k);
                    k_sleep(K_MSEC(5));
                }
                for (; k>=0; k--) {
                    led_proc[i](k);
                    k_sleep(K_MSEC(5));
                }
            }
        }
    }
}

#endif // LED_TEST
