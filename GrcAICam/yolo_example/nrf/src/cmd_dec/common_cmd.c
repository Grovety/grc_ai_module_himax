/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include "version.h"
#include "sys_reset.h"
#include "command_decoder.h"
#include "common_cmd.h"

int common_cmd(const char* p_inp, char* p_out)
{
    if (!strncmp(p_inp, "ECHO", 4)) {              // AT^ECHO=<string>\n
        if (*(p_inp + 4) != '=') {
            return prepare_and_add_lf(p_out, "BAD_FORMAT");
        }
        char* echo_str = (char*)p_inp + 5;
        return prepare_and_add_lf(p_out, echo_str);
    }
    else if (!strcmp(p_inp, "REBOOT")) {            // AT^REBOOT\n
        init_sys_reset(2000);
        return prepare_and_add_lf(p_out, "OK");
    }
    else if (!strcmp(p_inp, "SWNVER")) {            // AT^SWNVER\n
        return prepare_and_add_lf(p_out, NRF_SW_VER);
    }
    else if (!strcmp(p_inp, "HWNVER")) {            // AT^HWNVER\n
        return prepare_and_add_lf(p_out, NRF_HW_VER);
    }
    else if (!strcmp(p_inp, "SWHVER")) {            // AT^SWHVER\n
        return prepare_and_add_lf(p_out, GRV_SW_VER);
    }
    else if (!strcmp(p_inp, "HWHVER")) {            // AT^HWHVER\n
        return prepare_and_add_lf(p_out, GRV_HW_VER);
    }
    else if (!strcmp(p_inp, "DTVER")) {             // AT^DTVER\n
        return prepare_and_add_lf(p_out, NRF_DT_VER);
    }
    return CMDDEC_UNKNOWN_CMD;                      // non-positive value means don't send answer
}
