/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include "himax_comdef.h"
#include "himax_control.h"
#include "command_decoder.h"
#include "himax_cmd.h"

int himax_cmd(const char* p_inp, char* p_out)
{
    if (!strncmp(p_inp, "MODE", 4)) {          // AT^MODE or AT^MODE=<mode>
        p_inp += 4;
        if (*p_inp == '\0') {
            himax_next_mode_t mode = himax_get_mode();
            if (mode == HIMAX_MODE_CONT_WORK) {
                return prepare_and_add_lf(p_out, "WORK");
            }
#ifdef USE_POWER_DOWN_MODES
            else if (mode == HIMAX_MODE_POWER_DOWN) {
                return prepare_and_add_lf(p_out, "PDOWN");
            }
            else if (mode == HIMAX_MODE_DPOWER_DOWN) {
                return prepare_and_add_lf(p_out, "DPD");
            }
#endif // USE_POWER_DOWN_MODES
            else if (mode == HIMAX_MODE_POWER_OFF) {
                return prepare_and_add_lf(p_out, "PWROFF");
            }
            else {
                return prepare_and_add_lf(p_out, "UNKNOWN");
            }
        }
        if (*p_inp != '=') {
            return prepare_and_add_lf(p_out, "BAD_FORMAT");
        }
        p_inp += 1;
        himax_next_mode_t mode;
        if (!strcmp(p_inp, "WORK")) {           // work mode
            mode = HIMAX_MODE_CONT_WORK;
        }
#ifdef USE_POWER_DOWN_MODES
        else if (!strcmp(p_inp, "PDOWN")) {     // power down mode
            mode = HIMAX_MODE_POWER_DOWN;
        }
        else if (!strcmp(p_inp, "DPD")) {       // deep power down mode
            mode = HIMAX_MODE_DPOWER_DOWN;
        }
#endif // USE_POWER_DOWN_MODES
        else if (!strcmp(p_inp, "PWROFF")) {    // power off by switch
            mode = HIMAX_MODE_POWER_OFF;
        }
        else if (!strcmp(p_inp, "ONESHOT")) {   // one iteration and deep sleep
            return prepare_and_add_lf(p_out, "NOT_SUPPORT");
        }
        else {                                  // wrong parameter
            return prepare_and_add_lf(p_out, "BAD_PARAM");
        }
        bool res = himax_set_mode(mode, HIMAX_ADV_DO_NOTHING);
        return prepare_and_add_lf(p_out, res ? "OK" : "BAD_MODE");
    }
    return CMDDEC_UNKNOWN_CMD;
}

int himax_debug_cmd(const char* p_inp, char* p_out)
{
    if (!strncmp(p_inp, "RCGNIMIT", 8)) {
        if (*(p_inp + 8) != '=') {
            return prepare_and_add_lf(p_out, "BAD_FORMAT");
        }
        p_inp += 9;
        if (((*p_inp != '0') && (*p_inp != '1')) || (*(p_inp+1) != '\0')) {
            return prepare_and_add_lf(p_out, "BAD_PARAM");
        }
        himax_adv_action_t adv;
        if (*p_inp == '0') {
            adv = HIMAX_ADV_RCGN_IMIT_OFF;
        }
        else {
            adv = HIMAX_ADV_RCGN_IMIT_ON;
        }
        bool res = himax_set_mode(HIMAX_MODE_CONT_WORK, adv);
        return prepare_and_add_lf(p_out, res ? "OK" : "BAD_MODE");
    }
    return CMDDEC_UNKNOWN_CMD;
}
