/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

#include "pwr_ctrl.h"
#include "leds_ctrl.h"

#define POWER_CONTROL DT_PATH(zephyr_user)
static const struct gpio_dt_spec common_pwron_gpios_3v3 = GPIO_DT_SPEC_GET_BY_IDX(POWER_CONTROL, common_pwron_gpios, 0);
static const struct gpio_dt_spec common_pwron_gpios_1v8 = GPIO_DT_SPEC_GET_BY_IDX(POWER_CONTROL, common_pwron_gpios, 1);

#define CHARGER_CONTROL DT_PATH(zephyr_user)
static const struct gpio_dt_spec charger_gpios_bqce = GPIO_DT_SPEC_GET_BY_IDX(CHARGER_CONTROL, charger_ctrl_gpios, 0);
static const struct gpio_dt_spec charger_gpios_bqchg = GPIO_DT_SPEC_GET_BY_IDX(CHARGER_CONTROL, charger_ctrl_gpios, 1);
static const struct gpio_dt_spec charger_gpios_bqpgood = GPIO_DT_SPEC_GET_BY_IDX(CHARGER_CONTROL, charger_ctrl_gpios, 2);

static void pm_ctrl_3v3_on(bool on)
{
	gpio_pin_set_dt(&common_pwron_gpios_3v3, on);
}

static void pm_ctrl_1v8_on(bool on)
{
    gpio_pin_set_dt(&common_pwron_gpios_1v8, on);
}

static bool pm_charger_power_good()
{
    return gpio_pin_get_dt(&charger_gpios_bqpgood); // Pulls to VSS when a valid inputs source is detected
}

static bool pm_charger_is_charging()
{
    return gpio_pin_get_dt(&charger_gpios_bqchg); // Pulls to VSS when the batteryis charging
}

static bool pm_charger_is_on;

void pm_charger_status(char* buf)
{
    sprintf(buf, "%s,%d,%d", pm_charger_is_on ? "ON" : "OFF", pm_charger_power_good(), pm_charger_is_charging());
}

void pm_charger_ctrl(bool on)
{
    pm_charger_is_on = on;
    gpio_pin_set_dt(&charger_gpios_bqce, on); //  Charge Enable - active-low input
}

void pm_ctrl_init()
{
    gpio_pin_configure_dt(&common_pwron_gpios_3v3, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);
    gpio_pin_configure_dt(&common_pwron_gpios_1v8, GPIO_OUTPUT | GPIO_OUTPUT_INIT_LOW);

    pm_ctrl_3v3_on(true);
    pm_ctrl_1v8_on(true);

    gpio_pin_configure_dt(&charger_gpios_bqce, GPIO_OUTPUT | GPIO_OUTPUT_INIT_HIGH);
    gpio_pin_configure_dt(&charger_gpios_bqchg, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure_dt(&charger_gpios_bqpgood, GPIO_INPUT | GPIO_PULL_UP);
}
