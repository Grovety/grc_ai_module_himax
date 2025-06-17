/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include "image_proc.h"
#include "command_decoder.h"

int image_read_cmd(const char* p_inp, char* p_out, int out_size)
{
    if (!strcmp(p_inp, "IMAGE")) {              // AT&IMAGE
        if (image_take_one()) {
            return prepare_and_add_lf(p_out, "OK");
        }
        else {
            return prepare_and_add_lf(p_out, "BAD_MODE");
        }
    }
    else if (!strcmp(p_inp, "RDIMG")) {         // AT&RDIMG
        if (out_size < IMAGE_READ_HEADER_LEN + IMAGE_READ_CHUNK_SIZE) {
            return prepare_and_add_lf(p_out, "BUFFER_TOO_SMALL");
        }
        return image_read_first(p_out);
    }
    else if (!strcmp(p_inp, "RINEXT")) {        // AT&RINEXT
        if (out_size < IMAGE_READ_HEADER_LEN + IMAGE_READ_CHUNK_SIZE) {
            return prepare_and_add_lf(p_out, "BUFFER_TOO_SMALL");
        }
        return image_read_next(p_out);
    }
    return CMDDEC_UNKNOWN_CMD;
}
