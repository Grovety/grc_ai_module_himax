#pragma once

#include "hx_drv_spi.h"
#include "hx_drv_timer.h"

#include "nrf_comdef.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SPIM_FREQ 10000000

int spim_init();

void spim_init_buffers(uint8_t opcode);

int spim_add_result(const uint8_t* buf, unsigned len, unsigned off);

int spim_add_data_chunk(const uint8_t* data, unsigned len, uint16_t param);

void spim_send();

int spim_is_busy();

void spim_wait();

uint8_t spim_get_ctrl_bits();

#ifdef __cplusplus
}
#endif
