/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LIST_RECOGNIZED_OBJECTS_H
#define LIST_RECOGNIZED_OBJECTS_H

#include <time.h>
#include <stdbool.h>
#include <stdint.h>

#include "lists_common_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LROBJ_HISTORY_NUMBER_OF_INTERFACES 2

//#define CHECK_LROBJ_ITEM_FOR_BAD_FORMAT		// don't add item if it has wrong format
#define CHECK_LROBJ_ITEM_FOR_THE_SAME		// don't add item if it has already added
#define MIN_ALLOWABLE_TIME_DIFFERENCE_SEC 1

// Get the needed output buffer size:
// MAX_LROBJ_ONE_REC_LEN * <maximal requsted number of records>

#define MAX_LROBJ_ONE_REC_LEN 32	// "x000xx000,2024-12-12 11:22:33,Y;",for last record ';' will be changed to eof
#define MIN_LROBJ_OUT_BUF_LEN (MAX_LROBJ_READ_OP_NUM*) // check it..

// It depends on MTU size. 244/32=7
#define MAX_LROBJ_READ_OP_NUM 7	// one read operation can read no more than
typedef enum {
    LROBJ_OK = 0,
    LROBJ_BAD_OBJECT_FORMAT = -200, // wrong number
	LROBJ_BAD_DATE_FORMAT = -201,	// wrong date or time
	LROBJ_BAD_PERM_FORMAT = -202,	// not 'Y' or 'N' in the received command
	LROBJ_BAD_COUNT_FORMAT = -203,	// zero or to many data in request
	LROBJ_NOT_FOUND = -204,			// records after this date were not found
	LROBJ_BUFFER_TOO_SMALL = -205,	// don't have enough free place
	LROBJ_BAD_INTERFACE = -206,		// wrong number of interface to read history
	LROBJ_BAD_CRC = -207,			// list has errors
	LROBJ_OTHER_ERROR = -208		// some other problem
} list_recg_objects_error_t;

typedef struct {
	object_info_t object;			// object description
	time_t timestamp;				// data and time of the recognition
	bool is_expected;				// true if the object was gived rigths
} list_recognized_objects_item_t;

typedef list_recognized_objects_item_t list_recognized_objects_t[LROBJ_ITEMS_MAX_NUMBER];

typedef struct {
	list_recognized_objects_t list;
	int read_index;
	int write_index;
	int items_number;
	uint32_t crc;
} rcgn_obj_struct_current_t; // used for log

typedef struct {
	list_recognized_objects_t list;
	int read_index;
	int is_read;
	int items_number;
	time_t base_timestamp;
} rcgn_obj_struct_history_t; // used to save log when first and next read operations

void list_recg_objects_init(const rcgn_obj_struct_current_t* crps_saved);

void list_recg_objects_clear(void);

void list_recg_objects_copy(rcgn_obj_struct_current_t* dst);

list_recg_objects_error_t list_recg_objects_add(const object_info_t object, time_t timestamp, bool is_expected);

list_recg_objects_error_t list_recg_objects_user_add(const char* object, const char* date_time, char is_expected);

list_recg_objects_error_t list_recg_objects_read_first_ts(list_recg_objects_history_interface_t history_interface, int count, time_t timestamp, char* buffer, size_t buffer_size);

list_recg_objects_error_t list_recg_objects_read_first_dt(list_recg_objects_history_interface_t history_interface, int count, const char* date_time, char* buffer, size_t buffer_size);

list_recg_objects_error_t list_recg_objects_read_next(list_recg_objects_history_interface_t history_interface, int count, char* buffer, size_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif // LIST_RECOGNIZED_OBJECTS_H
