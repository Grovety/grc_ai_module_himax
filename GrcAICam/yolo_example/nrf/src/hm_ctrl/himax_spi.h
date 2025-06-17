/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef HIMAX_SPI_H
#define HIMAX_SPI_H

#include <stdint.h>

typedef void (*himax_spi_data_proc_t)(const uint8_t* data, unsigned dlen);

int himax_spis_init(himax_spi_data_proc_t data_proc);

void himax_spis_set_ctrl(uint8_t ctrl, bool clear_old);

#endif // HIMAX_SPI_H
