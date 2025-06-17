/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include "himax_comdef.h"
#include "himax_spi_data_proc.h"
#include "list_expected_objects.h"
#include "list_recognized_objects.h"
#include "amode_proc.h"
#include "image_proc.h"

#define LOG_MODULE_NAME himax_spi_data_proc
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

static uint8_t buffer[SPI_PAYLOAD_LEN];

static void himax_spi_data_img_proc(const uint8_t* data, unsigned dlen)
{
    memcpy(buffer, data + SPI_HDR_SIZE, SPI_PAYLOAD_LEN);

    uint16_t param = ((uint16_t)data[2] << 8) | data[3];

    if (image_update(buffer, param) == false)
    {
        LOG_ERR("Image update failed");
    }
}

static void himax_spi_data_obj_proc(const uint8_t* data, unsigned dlen)
{
    memcpy(buffer, data + SPI_HDR_SIZE, SPI_PAYLOAD_LEN);

    uint8_t* p_obj = buffer;
    bool found = false, permission = false;

    for (unsigned buf_idx = 0; buf_idx < SPI_BUF_NUMB; buf_idx++)
    {
        unsigned chk_len = strnlen(p_obj, 16);
        if (chk_len == 16) {
            LOG_ERR("Bad length");
            break;
        }

        if (p_obj[0] == '\0') {
            break;
        }
        else {
            found = true;
        }

        if (list_expct_objects_find_item(p_obj) == LPRP_OK) {
            permission = true;
        }

        time_t timestamp =  time(NULL);
        list_recg_objects_error_t err = list_recg_objects_add(p_obj, timestamp, permission);
        if (err != LROBJ_OK) {
            LOG_ERR("Lrrp list error %d", err);
        }

        p_obj += SPI_BUF_PLEN;
    }

    amode_recognize_results_update(found, permission);
}

void himax_spi_data_proc(const uint8_t* data, unsigned dlen)
{
    switch (data[0]) // opcode
    {
        case HIMAX_SPI_DATA_OBJINFO:
            himax_spi_data_obj_proc(data, dlen);
            break;
        case HIMAX_SPI_DATA_IMAGE:
            himax_spi_data_img_proc(data, dlen);
            break;
        default:
            LOG_ERR("Bad opcode");
    }
}
