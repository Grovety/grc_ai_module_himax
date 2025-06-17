/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include "list_expected_objects.h"
#include "command_decoder.h"
#include "list_expct_obj_cmd.h"

int lprp_cmd(const char* p_inp, char* p_out, int out_size)
{
    if (!strncmp(p_inp, "LPADD", 5)) {              // AT+LPADD=<number>\n
        if (*(p_inp + 5) != '=') {
            return prepare_and_add_lf(p_out, "BAD_FORMAT");
        }
        list_expct_objects_error_t err = list_expct_objects_add_item(p_inp + 6);
        if (err == LPRP_OK) {                       // it is ok
            return prepare_and_add_lf(p_out, "OK");
        }
        else if (err == LPRP_FULL) {                // no free space
            return prepare_and_add_lf(p_out, "LIST_FULL");
        }
        else if (err == LPRP_BAD_FORMAT) {          // wrong number
            return prepare_and_add_lf(p_out, "BAD_PARAM");
        }
        else if(err == LPRP_ALREADY_PUT) {          // number has already put
            return prepare_and_add_lf(p_out, "ALREADY_PUT");
        }
        else if (err == LPRP_BAD_CRC) {             // list has errors
            return prepare_and_add_lf(p_out, "BAD_CRC");
        }
        else {                                      // in case not all was noticed
            return prepare_and_add_lf(p_out, "OTHER_ERROR");
        }
    }
    else if (!strncmp(p_inp, "LPDEL", 5)) {         // AT+LPDEL=<number>\n
        if (*(p_inp + 5) != '=') {
            return prepare_and_add_lf(p_out, "BAD_FORMAT");
        }
        list_expct_objects_error_t err = list_expct_objects_del_item(p_inp + 6);
        if (err == LPRP_OK) {                       // it is ok
            return prepare_and_add_lf(p_out, "OK");
        }
        else if (err == LPRP_BAD_FORMAT) {          // wrong number
            return prepare_and_add_lf(p_out, "BAD_PARAM");
        }
        else if (err == LPRP_NOT_FOUND) {           // number isn't exist
            return prepare_and_add_lf(p_out, "NOT_FOUND");
        }
        else if (err == LPRP_BAD_CRC) {             // list has errors
            return prepare_and_add_lf(p_out, "BAD_CRC");
        }
        else {                                      // in case not all was noticed
            return prepare_and_add_lf(p_out, "OTHER_ERROR");
        }
    }
    else if (!strncmp(p_inp, "LPFIND", 6)) {        // AT+LPFIND=<number>\n
        if (*(p_inp + 6) != '=') {
            return prepare_and_add_lf(p_out, "BAD_FORMAT");
        }
        list_expct_objects_error_t err = list_expct_objects_find_item(p_inp + 7);
        if (err == LPRP_OK) {                       // it is ok
            return prepare_and_add_lf(p_out, "OK");
        }
        else if (err == LPRP_BAD_FORMAT) {          // wrong number
            return prepare_and_add_lf(p_out, "BAD_PARAM");
        }
        else if (err == LPRP_NOT_FOUND) {           // number isn't exist
            return prepare_and_add_lf(p_out, "NOT_FOUND");
        }
        else if (err == LPRP_BAD_CRC) {             // list has errors
            return prepare_and_add_lf(p_out, "BAD_CRC");
        }
        else {                                      // in case not all was noticed
            return prepare_and_add_lf(p_out, "OTHER_ERROR");
        }
    }
    else if (!strncmp(p_inp, "LPREAD", 6)) {        // AT+LPREAD\n
        list_expct_objects_error_t err = list_expct_objects_read_first(p_out, out_size);
        if (err == LPRP_OK) {
            return prepare_and_add_lf(p_out, p_out);
        }
        if (err == LPRB_BUFFER_TOO_SMALL) {         // no standard situation
            return prepare_and_add_lf(p_out, "BUFFER_TOO_SMALL");
        }
        else if (err == LPRP_BAD_CRC) {             // list has errors
            return prepare_and_add_lf(p_out, "BAD_CRC");
        }
        else {                                      // in case not all was noticed
            return prepare_and_add_lf(p_out, "OTHER_ERROR");
        }
    }
    else if (!strncmp(p_inp, "LPRDNEXT", 8)) {      // AT+LPRDNEXT\n
        list_expct_objects_error_t err = list_expct_objects_read_next(p_out, out_size);
        if (err == LPRP_OK) {
            return prepare_and_add_lf(p_out, p_out);
        }
        if (err == LPRB_BUFFER_TOO_SMALL) {         // no standard situation
            return prepare_and_add_lf(p_out, "BUFFER_TOO_SMALL");
        }
        else if (err == LPRP_BAD_CRC) {             // list has errors
            return prepare_and_add_lf(p_out, "BAD_CRC");
        }
        else {                                      // in case not all was noticed
            return prepare_and_add_lf(p_out, "OTHER_ERROR");
        }
    }
    else if (!strncmp(p_inp, "LPCLEAR", 7)) {       // AT+LPCLEAR\n
        list_expct_objects_clear();
        return prepare_and_add_lf(p_out, "OK");
    }
    return CMDDEC_UNKNOWN_CMD;                      // non-positive value means don't send answer
}
