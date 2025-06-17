#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void(*spi_dma_user_callback)();

void spi_control_init();

uint8_t spi_transmit_byte(uint8_t data);
void spi_transmit_byte_fast(uint8_t data);
void spi_transmit_many_fast(uint8_t* data, int len);
void spi_transmit_align(int align);
void spi_transmit_flush();

int spi_receive(int len);
unsigned char* get_spi_rx_buf();

void spi_transmit_dma(void* data, int len, spi_dma_user_callback callback);
void spi_wait();

#ifdef __cplusplus
}
#endif
