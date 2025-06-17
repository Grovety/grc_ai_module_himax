/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include "tof_sensor_comdef.h"
#include "tof_sensor_core.h"
#include "command_decoder.h"

int tof_sensor_cmd(const char* p_inp, char* p_out, int out_size)
{
    if (!strcmp(p_inp, "RDDIST")) {             // AT&RDDIST
        tof_sens_results_bigw_get_distance(p_out);
        return prepare_and_add_lf(p_out, p_out);
    }
    else if (!strcmp(p_inp, "RDZONE")) {        // AT&RDZONE
        ;return prepare_and_add_lf(p_out, "NOT_SUPPORTED");
    }
    else if (!strncmp(p_inp, "SMODE", 5)) {     // AT&SMODE=<mode>
        p_inp += 5;
        if (*p_inp == '\0') {
            tof_mode_t mode = tof_sensor_get_mode();
            if (mode == TOF_MODE_AUTO_WAIT_FOR_OBJECTS) {
                return prepare_and_add_lf(p_out, "AUTO+");
            }
            else if (mode == TOF_MODE_AUTO_WAIT_DISAPPERANCE) {
                return prepare_and_add_lf(p_out, "AUTO-");
            }
            else if (mode == TOF_MODE_MEAS_DISTANCE_TIMER) {
                return prepare_and_add_lf(p_out, "TIMER");
            }
            else if ((mode == TOF_MODE_DISABLED_BY_USER_CMD) || (mode == TOF_MODE_DISABLED_WHILE_RECOGN)) {
                return prepare_and_add_lf(p_out, "OFF");
            }
            else {
                return prepare_and_add_lf(p_out, "UNKNOWN");
            }
        }
        else if (*p_inp == '=') {
            p_inp += 1;
            
            if (!strcmp(p_inp, "AUTO+")) {
                tof_sensor_set_user_mode(TOF_MODE_AUTO_WAIT_FOR_OBJECTS);
                return prepare_and_add_lf(p_out, "OK");
            }
            else if (!strcmp(p_inp, "AUTO-")) {
                tof_sensor_set_user_mode(TOF_MODE_AUTO_WAIT_DISAPPERANCE);
                return prepare_and_add_lf(p_out, "OK");
            }
            else if (!strcmp(p_inp, "TIMER")) {
                tof_sensor_set_user_mode(TOF_MODE_MEAS_DISTANCE_TIMER);
                return prepare_and_add_lf(p_out, "OK");
            }
            else if (!strcmp(p_inp, "OFF")) {
                tof_sensor_set_user_mode(TOF_MODE_DISABLED_BY_USER_CMD);
                return prepare_and_add_lf(p_out, "OK");
            }
            else {
                return prepare_and_add_lf(p_out, "BAD_PARAM");
            }
        }
        else {
            return prepare_and_add_lf(p_out, "BAD_FORMAT");
        }
    }
    else if (!strcmp(p_inp, "SPARAM")) {       // AT&SPARAM
        tof_sensor_get_measure_param(p_out);
        return prepare_and_add_lf(p_out, p_out);
    }
    else if (!strncmp(p_inp, "DIST", 4))       // AT&DIST=<low_dist>,<high_dist>
    {
        p_inp += 4;
        if (*p_inp != '=') {
            return prepare_and_add_lf(p_out, "BAD_FORMAT");
        }
        const char* p0 = p_inp + 1;
        char* p1 = strchr(p0, ',');
        if (p1 == NULL) {
            return prepare_and_add_lf(p_out, "BAD_FORMAT");
        }
        *p1++ = '\0';
        if (tof_sensor_set_distance(atoi(p0), atoi(p1))) {
            return prepare_and_add_lf(p_out, "OK");
        }
        else {
            return prepare_and_add_lf(p_out, "BAD_PARAM");
        }
    }
    else if (!strncmp(p_inp, "ROI", 3)) {       // AT&ROI=<top_left_x>,<top_left_y>,<bottom_rigth_x>,<bottom_rigth_y>
        p_inp += 3;
        if (*p_inp != '=') {
            return prepare_and_add_lf(p_out, "BAD_FORMAT");
        }
        const char* p0 = p_inp + 1;
        char* p1 = strchr(p0, ',');
        char* p2 = p1 ? strchr(p1 + 1, ',') : NULL;
        char* p3 = p2 ? strchr(p2 + 1, ',') : NULL;
        if (!p1 || !p2 || !p3) {
            return prepare_and_add_lf(p_out, "BAD_FORMAT");
        }
        *p1++ = '\0';
        *p2++ = '\0';
        *p3++ = '\0';
        if (tof_sensor_set_amode_roi(atoi(p0), atoi(p1), atoi(p2), atoi(p3))) {
            return prepare_and_add_lf(p_out, "OK");
        }
        else {
            return prepare_and_add_lf(p_out, "BAD_PARAM");
        }
    }
    else if (!strncmp(p_inp, "TMBD", 4)) {
        p_inp += 4;
        if (*p_inp != '=') {
            return prepare_and_add_lf(p_out, "BAD_FORMAT");
        }
        const char* p0 = p_inp + 1;
        char* p1 = strchr(p0, ',');
        *p1++ = '\0';
        if (!p1) {
            return prepare_and_add_lf(p_out, "BAD_FORMAT");
        }
        if (tof_sensor_set_timing_budgets(atoi(p0), atoi(p1))) {
            return prepare_and_add_lf(p_out, "OK");
        }
        else {
            return prepare_and_add_lf(p_out, "BAD_PARAM");
        }
    }
    return CMDDEC_UNKNOWN_CMD;
}
