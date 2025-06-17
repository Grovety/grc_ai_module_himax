/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef COMMAND_DECODER_H
#define COMMAND_DECODER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

    extern const char* NRF_SW_VER;
    extern const char* NRF_HW_VER;
    extern const char* GRV_SW_VER;
    extern const char* GRV_HW_VER;
    typedef enum {
        CMDDEC_BAD_LENGTH = -300,
        CMDDEC_BAD_PREFIX = -301,
        CMDDEC_BAD_TAIL_CHAR = -302,
        //CMDDEC_BAD_FORMAT = -303,
        //CMDDEC_BAD_PARAM_0 = -304,
        //CMDDEC_BAD_PARAM_1 = -305,
        //CMDDEC_BAD_PARAM_2 = -306,
        CMDDEC_UNKNOWN_CMD = -307
    } cmddec_error_t;

    typedef void (*command_decoder_notify_proc_t)(void);

    int prepare_and_add_lf(char* dst, const char* src);

    void command_decoder_init(command_decoder_notify_proc_t proc);

    int command_decoder_run(int used_interface, const char* inp_buf, int inp_len, char* out_buf, int out_buf_size);

#ifdef __cplusplus
}
#endif

#endif // COMMAND_DECODER_H
