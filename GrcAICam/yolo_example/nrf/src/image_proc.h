/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IMAGE_PROC_H
#define IMAGE_PROC_H

#include <stdbool.h>
#include <stdint.h>

#include "himax_comdef.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMAGE_MAX_SIZE MAX_JPG_SIZE

#define IMAGE_READ_HEADER_LEN 4 // "#240" or "!240"
#define IMAGE_READ_CHUNK_SIZE 240
#define IMAGE_READ_CHUNK_NUMB (IMAGE_SIZE/IMAGE_READ_CHUNK_SIZE)

#define IMAGE_WRITE_CHUNK_SIZE SPI_PAYLOAD_LEN
#define IMAGE_WRITE_CHUNK_NUMB (IMAGE_SIZE/SPI_PAYLOAD_LEN)

#define IMAGE_READ_TIMEOUT_MS 2000

int image_read_first(char* buf);

int image_read_next(char* buf);

bool image_take_one();

bool image_update(char*buf, uint16_t prm);

typedef void (*image_control_proc_t)(bool start);

void image_reader_init(image_control_proc_t proc);

#ifdef __cplusplus
}
#endif

#endif // IMAGE_PROC_H