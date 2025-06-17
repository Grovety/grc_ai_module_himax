/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

#include "himax_comdef.h"
#include "himax_control.h"
#include "himax_spi.h"

#define LOG_MODULE_NAME himax_spi
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define HIMAX_SPI_OPERATION ((SPI_TRANSFER_MSB | SPI_WORD_SET(8)) | SPI_OP_MODE_SLAVE)
#define HIMAX_SPI_THREAD_STACKSIZE 1024
#define HIMAX_SPI_THREAD_PRIORITY 7

static himax_spi_data_proc_t himax_spi_data_proc;

static uint8_t tx_data[SPI_TRANS_LEN];
static uint8_t rx_data[SPI_TRANS_LEN];

static struct spi_buf spi_tx_buf = {
    .buf = tx_data,
    .len = sizeof(tx_data),
};

static struct spi_buf spi_rx_buf = {
    .buf = rx_data,
    .len = sizeof(rx_data),
};

static const struct spi_buf_set tx_buf_set = {
    .buffers = &spi_tx_buf,
    .count = 1,
};

static const struct spi_buf_set rx_buf_set = {
    .buffers = &spi_rx_buf,
    .count = 1,
};

static const struct device *himax_spi_dev = DEVICE_DT_GET(DT_CHOSEN(himax_spi));

static const struct spi_config himax_spi_cfg = {
	.operation = HIMAX_SPI_OPERATION,
	.frequency = HIMAX_SPI_FREQUENCY
};

static void himax_spis_thread_proc();
static K_THREAD_DEFINE(himax_spis_thread_id, HIMAX_SPI_THREAD_STACKSIZE, himax_spis_thread_proc, NULL, NULL, NULL,
		HIMAX_SPI_THREAD_PRIORITY, 0, -1);
static K_EVENT_DEFINE(spis_transfer_completed_event);
static K_EVENT_DEFINE(spis_transfer_upd_cmd_event);

int himax_spis_init(himax_spi_data_proc_t data_proc)
{
	if (!device_is_ready(himax_spi_dev))
    {
        LOG_ERR("SPI is not ready");
        return -1;
	}

    himax_spi_data_proc = data_proc;
    k_thread_start(himax_spis_thread_id);
    return 0;
}

void himax_spis_set_ctrl(uint8_t ctrl_bits, bool clear_old)
{
    (clear_old ? k_event_set : k_event_post)(&spis_transfer_upd_cmd_event, ctrl_bits);
}

static void himax_spis_callback(const struct device *dev, int result, void *data)
{
    k_event_set(&spis_transfer_completed_event, 1);
    k_thread_resume(himax_spis_thread_id);
}

static int himax_spis_init_transfer()
{
    int err = spi_transceive_cb(himax_spi_dev, &himax_spi_cfg, &tx_buf_set, &rx_buf_set, himax_spis_callback, NULL);
    if (err) {
        LOG_ERR("SPI cb transfer failed");
    }
    return err;
}

static bool himax_spis_check_header()
{
    if (rx_data[0] != (rx_data[1] ^ 0xFF)) {
        // LOG_WRN("Wrong SPI header (%02x,%02x)", rx_data[0], rx_data[1]);
        return false;
    }
    return true;    
}

static void himax_spis_thread_proc()
{
    himax_spis_init_transfer();

    while (1)
    {
        if (k_event_wait(&spis_transfer_completed_event, 1, false, K_FOREVER))
        {
            if (himax_spis_check_header() && himax_spi_data_proc) {
                himax_ctrl_spi_notify();
                himax_spi_data_proc(rx_data, SPI_TRANS_LEN);
            }
            tx_data[0] = tx_data[1] = 0;
            uint32_t event = k_event_test(&spis_transfer_upd_cmd_event, 0x7f);
            if (event) {
                tx_data[0] = (uint8_t)event;
                tx_data[1] = ~tx_data[0];
                k_event_clear(&spis_transfer_upd_cmd_event, event);
            }
            memset(rx_data, 0, sizeof(rx_data));
            k_event_clear(&spis_transfer_completed_event, 1);
            himax_spis_init_transfer();
        }
    }
}
