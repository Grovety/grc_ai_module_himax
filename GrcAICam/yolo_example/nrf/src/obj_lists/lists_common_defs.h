/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LISTS_COMMON_DEFS_H
#define LISTS_COMMON_DEFS_H

#define OBJECTS_INFO_MAX_LENGTH 16 // it can be reduced to real plate length

// Now the list of allowed registration marks is sent to the host in one chank.
// But it is limited by MTU size, it is only 247 byte.
// It defines the size of the list 247/10=24.

#define LPRP_ITEMS_MAX_NUMBER 32

// How many plates will be returned by read first and read next operations.
#define LPRP_ITEMS_MAX_RDNUMB 24

// Now the list of recognized registration plates is send to the host in several chunks.
// There are a group of commands for it. So, the size of the list das not link with MTU size.

#define LROBJ_ITEMS_MAX_NUMBER 64

typedef char object_info_t[OBJECTS_INFO_MAX_LENGTH]; // k123ae98,p345ey777
typedef enum {
	BLUETOOTH_INTERFACE = 0,
	USBUART_INTERFACE
} list_recg_objects_history_interface_t;

#endif // LISTS_COMMON_DEFS_H
