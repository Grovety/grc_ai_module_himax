/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdlib.h>

#include "list_recognized_objects.h"
#include "time_conv.h"
#include "command_decoder.h"
#include "list_recg_obj_cmd.h"

int list_recg_obj_cmd(const char* p_inp, char* p_out, int out_size, list_recg_objects_history_interface_t used_interface)
{
    if (!strncmp(p_inp, "LRREAD", 6)) {             // AT+LRREAD=<data>,<count>\n,AT^LRREAD=<count>\n
        if (*(p_inp + 6) != '=') {
            return prepare_and_add_lf(p_out, "BAD_FORMAT");
        }
        const char* p_param = p_inp + 7;
        if (strlen(p_inp) < 1) {
            return prepare_and_add_lf(p_out, "BAD_FORMAT");
        }
        time_t timestamp = 0;
        char* p_comma = strchr(p_param, ',');       // search for comma
        if (p_comma != NULL) {
            if (str_to_time(p_comma+1, &timestamp) == false) {
                return prepare_and_add_lf(p_out, "BAD_PARAM_1");
            }
            *p_comma = '\0';
        }
        int count = atoi(p_param);                  // strtol?
        if ((count <= 0) || (count > MAX_LROBJ_READ_OP_NUM)) {
            return prepare_and_add_lf(p_out, "BAD_PARAM_0");
        }
        list_recg_objects_error_t err = list_recg_objects_read_first_ts(used_interface, count, timestamp, p_out, out_size);
        if (err == LROBJ_OK) {
            return prepare_and_add_lf(p_out, p_out);
        }
        else if (err == LROBJ_NOT_FOUND) {
            return prepare_and_add_lf(p_out, p_out/*"NOT_FOUND"*/);
        }
        else if (err == LROBJ_BUFFER_TOO_SMALL) {    // no standard situation
            return prepare_and_add_lf(p_out, "BUFFER_TOO_SMALL");
        }
        else if (err == LROBJ_BAD_CRC) {             // list was damaged
            return prepare_and_add_lf(p_out, "BAD_CRC");
        }
        else {                                      // in case not all was noticed
            return prepare_and_add_lf(p_out, "OTHER_ERROR");
        }
    }
    else if (!strncmp(p_inp, "LRRDNEXT", 8)) {      // AT+LRRDNEXT=<count>\n
        if (*(p_inp + 8) != '=') {
            return prepare_and_add_lf(p_out, "BAD_FORMAT");
        }
        const char* p_param = p_inp + 9;
        if (strlen(p_inp) < 1) {
            return prepare_and_add_lf(p_out, "BAD_FORMAT");
        }
        int count = atoi(p_param);                  // strtol?
        if ((count <= 0) || (count > MAX_LROBJ_READ_OP_NUM)) {
            return prepare_and_add_lf(p_out, "BAD_PARAM_0");
        }
        list_recg_objects_error_t err = list_recg_objects_read_next(used_interface, count, p_out, out_size);
        if (err == LROBJ_OK) {
            return prepare_and_add_lf(p_out, p_out);
        }
        if (err == LROBJ_NOT_FOUND) {
            return prepare_and_add_lf(p_out, "NOT_FOUND");
        }
        if (err == LROBJ_BUFFER_TOO_SMALL) {         // no standard situation
            return prepare_and_add_lf(p_out, "BUFFER_TOO_SMALL");
        }
        else {                                      // in case not all was noticed
            return prepare_and_add_lf(p_out, "OTHER_ERROR");
        }
    }
    else if (!strncmp(p_inp, "LRCLEAR", 7)) {       // AT+LPCLEAR\n
        list_recg_objects_clear();
        return prepare_and_add_lf(p_out, "OK");
    }
    else if (!strncmp(p_inp, "LRADD", 5)) {         // AT+<number>,<data>,<perm>\n
        if (*(p_inp + 5) != '=') {
            return prepare_and_add_lf(p_out, "BAD_FORMAT");
        }
        const char* p_plate = p_inp + 6;
        char* p_comma0 = strchr(p_plate, ',');
        if (!p_comma0) {
            return prepare_and_add_lf(p_out, "BAD_FORMAT");
        }
        char* p_comma1 = strchr(p_comma0 + 1, ',');
        if (!p_comma1) {
            return prepare_and_add_lf(p_out, "BAD_FORMAT");
        }
        *(p_comma0) = '\0';
        *(p_comma1) = '\0';

        list_recg_objects_error_t err = list_recg_objects_user_add(p_plate, p_comma0+1, *(p_comma1+1));
        if (err == LROBJ_OK) {
            return prepare_and_add_lf(p_out, "OK");
        }
        else if (err == LROBJ_BAD_OBJECT_FORMAT) {
            return prepare_and_add_lf(p_out, "BAD_PARAM_0");
        }
        else if (err == LROBJ_BAD_DATE_FORMAT) {
            return prepare_and_add_lf(p_out, "BAD_PARAM_1");
        }
        else if (err == LROBJ_BAD_PERM_FORMAT) {
            return prepare_and_add_lf(p_out, "BAD_PARAM_2");
        }
        else if (err == LROBJ_BAD_CRC) {             // list was damaged
            return prepare_and_add_lf(p_out, "BAD_CRC");
        }
        else {                                      // in case not all was noticed
            return prepare_and_add_lf(p_out, "OTHER_ERROR");
        }
    }
    return CMDDEC_UNKNOWN_CMD;
}
