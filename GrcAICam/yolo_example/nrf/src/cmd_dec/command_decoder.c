/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "common_cmd.h"
#include "himax_cmd.h"
#include "list_expct_obj_cmd.h"
#include "list_recg_obj_cmd.h"
#include "date_time_cmd.h"
#include "tof_sensor_cmd.h"
#include "image_read_cmd.h"

#include "mutex_wrapper.h"
#include "command_decoder.h"

#define LOG_MODULE_NAME command_decoder
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

// #define CMD_LOG

#ifdef CMD_LOG
    static void cmd_log(const char* pref, const char* data, unsigned len)
    {
        #define CHUNK_LEN 20
        #define FMT(idx) ((idx >= 4) && ((data[0] == '#') || (data[0] == '!')) ? " %02x" : "%c")
        if (pref) {
            printk("%s", pref);
        }
        if (len <= CHUNK_LEN*2) {
            for (unsigned i = 0; i < len; i++)
                printk(FMT(i), data[i]);
        }
        else {
            for (unsigned i = 0; i < CHUNK_LEN; i++) {
                printk(FMT(i), data[i]);
            }
            printk("..");
            for (unsigned i = len-CHUNK_LEN; i < len; i++) {
                printk(FMT(i), data[i]);
            }
        }
        printk("\n");
    }
#endif // CMD_LOG

K_MUTEX_DEF(cmddec_mutex)

static command_decoder_notify_proc_t notify_proc;

int prepare_and_add_lf(char* dst, const char* src)
{
    int len = (int)strlen(src);
    if (dst != src) // else just add lf
        memcpy(dst, src, len);
    dst[len] = '\n';
    return len + 1;
}

void command_decoder_init(command_decoder_notify_proc_t proc)
{
    K_MUTEX_INIT(cmddec_mutex)
    notify_proc = proc;
}

int command_decoder_run(int used_interface, const char* inp_buf, int inp_len, char* out_buf, int out_buf_size)
{
    K_MUTEX_LOCK(cmddec_mutex)

#ifdef CMD_LOG
   cmd_log(">> ", inp_buf, inp_len);
#endif

    if (notify_proc) {
        notify_proc();
    }

    char* p_inp = (char*)inp_buf;
    char* p_out = out_buf;
    *p_out = '\0';
    int ret;
 
    if (inp_len < 3) {                              // check for minimal length
        ret = CMDDEC_BAD_LENGTH;
        goto exit;
    }
    if (*(p_inp + inp_len - 1) != '\n') {           // check for tailing '\n' and replace to zero
        ret = CMDDEC_BAD_TAIL_CHAR;
        goto exit;
    }
    *(p_inp + inp_len - 1) = '\0';
    if ((*p_inp != 'A') || (*(p_inp + 1) != 'T')) { // check for AT prefix
        ret = CMDDEC_BAD_PREFIX;
        goto exit;
    }
    p_inp += 2;
    if (*p_inp == '\0') {                           // "AT\n"
        ret = prepare_and_add_lf(p_out, "AT");
        goto exit;
    }
    if (*p_inp == '^') {
        ret = common_cmd(p_inp + 1, p_out);
        if (ret == CMDDEC_UNKNOWN_CMD) {
            ret = himax_cmd(p_inp + 1, p_out);
            if (ret == CMDDEC_UNKNOWN_CMD) {
                ret = himax_debug_cmd(p_inp + 1, p_out);
            }
        }
        goto exit;
    }
    if (*p_inp == '+') {
        ret = lprp_cmd(p_inp + 1, p_out, out_buf_size);
        if (ret == CMDDEC_UNKNOWN_CMD) {
            ret = list_recg_obj_cmd(p_inp + 1, p_out, out_buf_size, used_interface);
        }
        goto exit;
    }
    if (*p_inp == '%') {
        ret = date_time_cmd(p_inp + 1, p_out, out_buf_size);
        goto exit;
    }
    if (*p_inp == '&') {
        ret = image_read_cmd(p_inp + 1, p_out, out_buf_size);
        if (ret == CMDDEC_UNKNOWN_CMD) {
            ret = tof_sensor_cmd(p_inp + 1, p_out, out_buf_size);
        }
        goto exit;
    }
    ret = CMDDEC_UNKNOWN_CMD;

exit:

    // if (ret == CMDDEC_UNKNOWN_CMD) {
    //     ret = prepare_and_add_lf(p_out, "UNKNOWN_CMD");
    // }

#ifdef CMD_LOG
    if (ret > 0) {
        cmd_log("<< ", out_buf, ret);
    }
    else {
        printk("<< error %d\n", ret);
    }
#endif

    K_MUTEX_UNLOCK(cmddec_mutex)

    return ret;
}
