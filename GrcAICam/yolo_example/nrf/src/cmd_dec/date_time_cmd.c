/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include "time_conv.h"
#include "command_decoder.h"
#include "common_cmd.h"

int date_time_cmd(const char* p_inp, char* p_out, int out_size)
{
    if (!strcmp(p_inp, "TIME")) {               // AT%TIME\n
        time_t cur_time = time(NULL);
        return prepare_and_add_lf(p_out, time_to_str(&cur_time));
    }
    else if (!strncmp(p_inp, "TIME", 4)) {      // AT%TIME=<date>\n
        if (*(p_inp + 4) != '=') {
            return prepare_and_add_lf(p_out, "BAD_FORMAT");
        }
        const char* p_dt = p_inp + 5;
        time_t new_time;
        if (str_to_time(p_dt, &new_time)) {
            if (set_time(&new_time) != 0) {
                return prepare_and_add_lf(p_out, "ERROR");
            }
            else {
                return prepare_and_add_lf(p_out, "OK");
            }
        }
        else {
            return prepare_and_add_lf(p_out, "BAD_PARAM");
        }
    }
    return CMDDEC_UNKNOWN_CMD;
}
