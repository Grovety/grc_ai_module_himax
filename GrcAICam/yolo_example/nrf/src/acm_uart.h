/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ACM_UART_H
#define ACM_UART_H

#include "bt_proc.h"
#include "command_decoder.h"

#define SR_OUT_BUF_SIZE BT_OUT_BUF_SIZE

typedef struct {
    int is_read;
    uint8_t rx_buf[SR_OUT_BUF_SIZE];
    int to_write;
    int is_written;
    uint8_t tx_buf[SR_OUT_BUF_SIZE];
    bool is_command;
} uart_ctx_t;

typedef int (*uart_data_proc_t)(int if_num, const char* inp_buf, int inp_len, char* out_buf, int out_buf_size);

int acm_uart_init(uart_data_proc_t uart_data_proc);

#endif // ACM_UART_H
