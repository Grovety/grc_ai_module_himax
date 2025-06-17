/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LIST_EXPECTED_OBJECTS_H
#define LIST_EXPECTED_OBJECTS_H

#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>

#include "objects_validator.h"
#include "lists_common_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LPRP_OK = 0,
    LPRP_FULL = -100,               // the list is full
    LPRP_BAD_FORMAT = -101,         // the wrong number
    LPRP_NOT_FOUND = -102,          // the number is abcent
    LPRB_BUFFER_TOO_SMALL = -103,   // the data size is more then the buffer size
    LPRP_ALREADY_PUT = -104,        // the number, that we are trying put,has already put
    LPRP_BAD_CRC = -105,            // list has errors
    LPRP_OTHER_ERROR = -106         // some other problem
} list_expct_objects_error_t;

typedef struct {
    object_info_t items[LPRP_ITEMS_MAX_NUMBER];       // fixed size list
    uint32_t crc;
} list_expected_objects_t;

void list_expct_objects_init(const list_expected_objects_t* list_expected_objects_saved);

void list_expct_objects_clear(void);

void list_expct_objects_copy(list_expected_objects_t* dst);

list_expct_objects_error_t list_expct_objects_find_item(const char* object);

list_expct_objects_error_t list_expct_objects_add_item(const char* object);

list_expct_objects_error_t list_expct_objects_del_item(const char* object);

list_expct_objects_error_t list_expct_objects_read_first(char* buffer, size_t buffer_size);

list_expct_objects_error_t list_expct_objects_read_next(char* buffer, size_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif // LIST_EXPECTED_OBJECTS_H
