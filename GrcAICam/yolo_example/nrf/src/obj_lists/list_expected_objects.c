/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "list_expected_objects.h"
#include "mutex_wrapper.h"
#include "crc_proc.h"

#define LOG_MODULE_NAME list_expct_objects
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

static list_expected_objects_t list_expected_objects;

K_MUTEX_DEF(lprp_mutex)

static void update_crc(list_expected_objects_t* cppl)
{ // add item,delete item,clear list
    uint32_t new_crc = crc32_plists((const uint8_t*)cppl->items, OBJECTS_INFO_MAX_LENGTH*LPRP_ITEMS_MAX_NUMBER);
    cppl->crc = new_crc;
}

static bool check_crc(const list_expected_objects_t* cppl)
{
    if (cppl == NULL) {
        return false;
    }
    uint32_t chk_crc = crc32_plists((const uint8_t*)cppl->items, OBJECTS_INFO_MAX_LENGTH * LPRP_ITEMS_MAX_NUMBER);
    return cppl->crc == chk_crc ? true : false;
}

static list_expct_objects_error_t find_item(const object_info_t object)
{
    if (check_object_format(object) != true) {                 // first to check format
        return LPRP_BAD_FORMAT;
    }

    for (unsigned i = 0; i < LPRP_ITEMS_MAX_NUMBER; i++) {      // search for item
        if (!strcmp(list_expected_objects.items[i], object)) {
            return LPRP_OK;
        }
    }

    return LPRP_NOT_FOUND;
}

list_expct_objects_error_t list_expct_objects_find_item(const char* object)
{
    list_expct_objects_error_t err;

    K_MUTEX_LOCK(lprp_mutex)

    if (check_crc(&list_expected_objects) != true) {
        err = LPRP_BAD_CRC;
        goto exit;
    }

    err = find_item(object);

exit:

    K_MUTEX_UNLOCK(lprp_mutex)

    return err;
}

list_expct_objects_error_t list_expct_objects_add_item(const char* object)
{
    list_expct_objects_error_t err;

    K_MUTEX_LOCK(lprp_mutex)

    if (check_crc(&list_expected_objects) != true) {
        err = LPRP_BAD_CRC;
        goto exit;
    }

    err = find_item(object);                                    // search for item

    if (err == LPRP_BAD_FORMAT) {
        goto exit;
    }

    if (err == LPRP_OK) {                                       // item has aready written
        err = LPRP_ALREADY_PUT;
        goto exit;
    }

    for (unsigned i = 0; i < LPRP_ITEMS_MAX_NUMBER; i++) {      // find first free item and fill with new data
        if (list_expected_objects.items[i][0] == '\0') {
            strcpy(list_expected_objects.items[i], object);
            update_crc(&list_expected_objects);
            err = LPRP_OK;
            goto exit;
        }
    }

    err = LPRP_FULL;

exit:

    K_MUTEX_UNLOCK(lprp_mutex)

    return err;
}

list_expct_objects_error_t list_expct_objects_del_item(const char* object)
{
    list_expct_objects_error_t err;

    K_MUTEX_LOCK(lprp_mutex)

    if (check_object_format(object) != true) {                 // first to check format
        err = LPRP_BAD_FORMAT;
        goto exit;
    }

    if (check_crc(&list_expected_objects) != true) {
        err = LPRP_BAD_CRC;
        goto exit;
    }

    for (unsigned i = 0; i < LPRP_ITEMS_MAX_NUMBER; i++) {      // search for item and fill with zero
        if (!strcmp(list_expected_objects.items[i], object)) {
            memset(list_expected_objects.items[i], 0, OBJECTS_INFO_MAX_LENGTH);
            update_crc(&list_expected_objects);
            err = LPRP_OK;
            goto exit;
        }
    }

    err = LPRP_NOT_FOUND;

exit:

    K_MUTEX_UNLOCK(lprp_mutex)

    return err;
}

static unsigned lprp_read_index = 0;

static list_expct_objects_error_t list_expct_objects_read(char* buffer, size_t buffer_size)
{
    list_expct_objects_error_t err;
    size_t act_len = 0;
    unsigned act_num = 0;
    bool first = true;

    K_MUTEX_LOCK(lprp_mutex)

    if (check_crc(&list_expected_objects) != true) {
        err = LPRP_BAD_CRC;
        goto exit;
    }

    memset(buffer, 0, buffer_size);

    while (lprp_read_index < LPRP_ITEMS_MAX_NUMBER) {

        if (list_expected_objects.items[lprp_read_index][0] != '\0') {        // for each actual item
            if (!first) {
                act_len += 1; // delimiter ','
            }
            act_len += strlen(list_expected_objects.items[lprp_read_index]);  // check for free space
            if (act_len > (buffer_size-1)) { // tailing '\0' will replace last ','
                memset(buffer, 0, buffer_size);
                err = LPRB_BUFFER_TOO_SMALL;
                goto exit;
            }
            if (!first) {
                strcat(buffer, ",");
            }
            first = false;
            strcat(buffer, list_expected_objects.items[lprp_read_index]);
            act_num += 1;
        }
        lprp_read_index += 1;
        if (act_num >= LPRP_ITEMS_MAX_RDNUMB) {                             // have reached maximum number of records
            break;
        }
    }

    err = LPRP_OK;

exit:

    K_MUTEX_UNLOCK(lprp_mutex)

    return err;
}

list_expct_objects_error_t list_expct_objects_read_first(char* buffer, size_t buffer_size)
{
    lprp_read_index = 0;
    list_expct_objects_error_t err = list_expct_objects_read(buffer, buffer_size);
    return err;
}

list_expct_objects_error_t list_expct_objects_read_next(char* buffer, size_t buffer_size)
{
    list_expct_objects_error_t err = list_expct_objects_read(buffer, buffer_size);
    return err;
}

void list_expct_objects_clear(void)
{
    K_MUTEX_LOCK(lprp_mutex)

    for (unsigned i = 0; i < LPRP_ITEMS_MAX_NUMBER; i++) {          // go through all records
        memset(list_expected_objects.items[i], 0, OBJECTS_INFO_MAX_LENGTH);   // fill with zero whole string
    }

    update_crc(&list_expected_objects);

    K_MUTEX_UNLOCK(lprp_mutex)
}

void list_expct_objects_init(const list_expected_objects_t* list_expected_objects_saved)
{
    K_MUTEX_INIT(lprp_mutex)

    if (list_expected_objects_saved) {
        if (check_crc(list_expected_objects_saved)) { // in case we have saved list in flash or retained memory
            memcpy(&list_expected_objects, list_expected_objects_saved, sizeof(list_expected_objects_t));
            return;
        }
        else {
            LOG_ERR("Bad crc");
        }
    }
    list_expct_objects_clear();
}

void list_expct_objects_copy(list_expected_objects_t* dst)
{
    K_MUTEX_LOCK(lprp_mutex)

    memcpy(dst, &list_expected_objects, sizeof(list_expected_objects_t));

    K_MUTEX_UNLOCK(lprp_mutex)
}
