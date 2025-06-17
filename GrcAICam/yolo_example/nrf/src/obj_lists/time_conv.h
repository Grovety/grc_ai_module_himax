/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TIME_CONV_H
#define TIME_CONV_H

#include <stdbool.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

const char* time_to_str(const time_t* t); // "yyyy-mm-dd hh-mm-ss"

bool str_to_time(const char* buf, time_t* t);

int set_time(const time_t* new_time);

void init_time();

#ifdef __cplusplus
}
#endif

#endif // TIME_CONV_H
