/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BT_PROC_H
#define BT_PROC_H

#define BT_OUT_BUF_SIZE (247-3) // set to MTU size

typedef int (*bt_data_proc_t)(int if_num, const char* inp_buf, int inp_len, char* out_buf, int out_buf_size);

int bt_init(bt_data_proc_t proc);

#endif // BT_PROC_H
