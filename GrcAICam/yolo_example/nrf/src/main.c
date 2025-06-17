/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <soc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

#include <bluetooth/services/nus.h>

#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>

#include <stdio.h>

#include "amode_proc.h"
#include "leds_ctrl.h"
#include "acm_uart.h"
#include "himax_spi.h"
#include "himax_control.h"
#include "tof_sensor_core.h"
#include "pwr_ctrl.h"
#include "bt_proc.h"
#include "image_proc.h"

#include "version.h"
#include "lists_common_defs.h"
#include "list_expected_objects.h"
#include "list_recognized_objects.h"
#include "command_decoder.h"
#include "himax_spi_data_proc.h"
#include "time_conv.h"

#define LOG_MODULE_NAME objdet_main
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

static void error_handler(int err)
{
	LOG_INF("Error %d", err);
	led_blue_ctrl(20);
	while (1) {
		led_red_ctrl(LED_ON);
		k_sleep(K_MSEC(100));
		led_red_ctrl(LED_OFF);
		k_sleep(K_MSEC(100));
	}
}

void scenario1_init()
{ // only turn on Himax power
	pm_ctrl_init();

	leds_init();

	himax_ctrl_init();

	himax_pwrctl(true);

	led_green_ctrl(50);
}

void scenario2_init()
{ // tof sensor is involved
	int err;

	pm_ctrl_init();

	leds_init();

	himax_ctrl_init();

	err = tof_sensor_init(false);
	if (err) {
		error_handler(err);
	}

	err = amode_proc_init();
	if (err) {
		error_handler(err);
	}

	led_green_ctrl(3);
}

void scenario3_init()
{ // all functional capabilities are involved
	int err;

	pm_ctrl_init();

	leds_init();

	init_time();

	list_expct_objects_init(NULL);
	list_recg_objects_init(NULL);

	err = himax_ctrl_init();
	if (err) {
		error_handler(err);
	}

	err = himax_spis_init(himax_spi_data_proc);
	if (err) {
		error_handler(err);
	}

	command_decoder_init(amode_timer_cmd_recv_notify);

	image_reader_init(amode_image_read_notify);

	err = usb_enable(NULL);
	if (err) {
		error_handler(err);
	}

	err = tof_sensor_init(false);
	if (err) {
		error_handler(err);
	}

	err = amode_proc_init();
	if (err) {
		error_handler(err);
	}

	err = acm_uart_init(command_decoder_run);
	if (err) {
		error_handler(err);
	}

	err = bt_init(command_decoder_run);
	if (err) {
		error_handler(err);
	}
}

int main()
{
#if APPL_SCENARIO == 1
	scenario1_init();
#elif APPL_SCENARIO == 2
	scenario2_init();
#elif APPL_SCENARIO == 3
	scenario3_init();
#else
	#error Scenario must be defined!
#endif
	k_sleep(K_FOREVER);
}
