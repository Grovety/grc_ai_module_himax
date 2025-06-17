/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "mutex_wrapper.h"
#include "list_expected_objects.h"
#include "list_recognized_objects.h"
#include "time_conv.h"
#include "crc_proc.h"

#define LOG_MODULE_NAME list_rcgn_objects
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

static rcgn_obj_struct_current_t rcgn_obj_struct_current;

static rcgn_obj_struct_history_t rcgn_obj_struct_history[LROBJ_HISTORY_NUMBER_OF_INTERFACES];

#define LROBJ_INCR(I) {I=(I+1) % LROBJ_ITEMS_MAX_NUMBER;}

K_MUTEX_DEF(lrrp_mutex)

static void update_crc(rcgn_obj_struct_current_t* rosc)
{
    uint32_t new_crc = crc32_plists((uint8_t*)rosc, sizeof(rcgn_obj_struct_current_t) - sizeof(uint32_t));
    rosc->crc = new_crc;
}

static bool check_crc(const rcgn_obj_struct_current_t* rosc)
{
    if (rosc == NULL) {
        return false;
    }
    uint32_t chk_crc = crc32_plists((uint8_t*)rosc, sizeof(rcgn_obj_struct_current_t) - sizeof(uint32_t));
    return rosc->crc == chk_crc ? true : false;
}

void list_recg_objects_init(const rcgn_obj_struct_current_t* rosc_saved)
{
    K_MUTEX_INIT(lrrp_mutex)

    if (rosc_saved) { // in case we have saved list in flash or retained memory
        if (check_crc(rosc_saved)) {
            memcpy(&rcgn_obj_struct_current, rosc_saved, sizeof(rcgn_obj_struct_current_t));
            return;
        }
        else {
            LOG_ERR("Bad crc");
        }
    }
    else {
        list_recg_objects_clear();
    }
}

void list_recg_objects_clear(void)
{
    K_MUTEX_LOCK(lrrp_mutex)

    rcgn_obj_struct_current.write_index = 0;
    rcgn_obj_struct_current.read_index = 0;
    rcgn_obj_struct_current.items_number = 0;

    for (int i = 0; i < LROBJ_ITEMS_MAX_NUMBER; i++) {
        memset(&rcgn_obj_struct_current.list[i], 0, sizeof(list_recognized_objects_item_t));
    }

    update_crc(&rcgn_obj_struct_current);

    for (int i = 0; i < LROBJ_HISTORY_NUMBER_OF_INTERFACES; i++) {
        rcgn_obj_struct_history[i].read_index = 0;
        rcgn_obj_struct_history[i].is_read = 0;
        rcgn_obj_struct_history[i].items_number = 0;
        rcgn_obj_struct_history[i].base_timestamp = 0;

        for (int j = 0; j < LROBJ_ITEMS_MAX_NUMBER; j++) {
            memset(&rcgn_obj_struct_history[i].list[j], 0, sizeof(list_recognized_objects_item_t));
        }
    }

    K_MUTEX_UNLOCK(lrrp_mutex)
}

#ifdef CHECK_LROBJ_ITEM_FOR_THE_SAME
static bool list_recg_objects_check_same(const object_info_t object, time_t timestamp, bool is_expected)
{
    for (int i = 0; i < rcgn_obj_struct_current.items_number; i++) {
        time_t diff_time = timestamp - rcgn_obj_struct_current.list[i].timestamp;
        if ((diff_time >= 0) && (diff_time <= MIN_ALLOWABLE_TIME_DIFFERENCE_SEC) &&
            !strcmp(rcgn_obj_struct_current.list[i].object, object) &&
            (rcgn_obj_struct_current.list[i].is_expected == is_expected)) {
                return false;
        }
    }
    return true;
}
#endif

list_recg_objects_error_t list_recg_objects_add(const object_info_t object, time_t timestamp, bool is_expected)
{
    list_recg_objects_error_t err = LROBJ_OK;

    K_MUTEX_LOCK(lrrp_mutex)

    bool ok = true;

#ifdef CHECK_LROBJ_ITEM_FOR_BAD_FORMAT
    ok = check_object_format(object);
#endif

#ifdef CHECK_LROBJ_ITEM_FOR_THE_SAME
    ok = ok && list_recg_objects_check_same(object, timestamp, is_expected);
#endif

    if (check_crc(&rcgn_obj_struct_current) != true) {
        err = LROBJ_BAD_CRC;
        goto exit;
    }

    if (ok) {
        memcpy(rcgn_obj_struct_current.list[rcgn_obj_struct_current.write_index].object, object, OBJECTS_INFO_MAX_LENGTH);
        rcgn_obj_struct_current.list[rcgn_obj_struct_current.write_index].timestamp = timestamp;
        rcgn_obj_struct_current.list[rcgn_obj_struct_current.write_index].is_expected = is_expected;

        LROBJ_INCR(rcgn_obj_struct_current.write_index)

        if (rcgn_obj_struct_current.items_number < LROBJ_ITEMS_MAX_NUMBER) { // the list is full?
            rcgn_obj_struct_current.items_number += 1;
        }
        else {
            LROBJ_INCR(rcgn_obj_struct_current.read_index)
        }
        update_crc(&rcgn_obj_struct_current);
    }

exit:

    K_MUTEX_UNLOCK(lrrp_mutex)

    return err;
}

list_recg_objects_error_t list_recg_objects_user_add(const char* object, const char* date_time, char is_expected)
{
    if (check_object_format(object) != true) {
        return LROBJ_BAD_OBJECT_FORMAT;
    }

    time_t recg_time;
    if (str_to_time(date_time, &recg_time) != true) {
        return LROBJ_BAD_DATE_FORMAT;
    }

    bool perm;
    if (is_expected == 'Y') {
        perm = true;
    }
    else if (is_expected == 'N') {
        perm = false;
    }
    else {
        return LROBJ_BAD_PERM_FORMAT;
    }

    list_recg_objects_error_t err = list_recg_objects_add(object, recg_time, perm);

    return err;
}

static int find_first_index_by_timestamp(rcgn_obj_struct_history_t* p_rcgn_obj_struct_history, time_t timestamp)
{
    int ret = -1;

    int curr_index = p_rcgn_obj_struct_history->read_index;

    if (timestamp == 0) {
        ret = curr_index;// if timestamp was not given,return the first item
    }
    else {
        for (int i = 0; i < LROBJ_ITEMS_MAX_NUMBER; i++)
        {
            if (i >= p_rcgn_obj_struct_history->items_number) {
                break;
            }
            if (p_rcgn_obj_struct_history->list[curr_index].timestamp >= timestamp) {
                ret = curr_index;
                break;
            }
            LROBJ_INCR(curr_index)
        }
    }

    return ret;
}

static list_recg_objects_error_t read_chunk(rcgn_obj_struct_history_t* p_rcgn_obj_struct_history, int count, time_t timestamp, bool first_read, char* buffer, size_t buffer_size)
{
    memset(buffer, 0, buffer_size);

    if (first_read) {
        p_rcgn_obj_struct_history->read_index = find_first_index_by_timestamp(p_rcgn_obj_struct_history, timestamp);
        if (p_rcgn_obj_struct_history->read_index < 0) {
            return LROBJ_NOT_FOUND;
        }
        p_rcgn_obj_struct_history->is_read = 0;
        p_rcgn_obj_struct_history->base_timestamp = timestamp;
    }

    size_t act_len = 0;
    bool first = true;

    for (int i = 0; i < count; i++)
    {
        if (p_rcgn_obj_struct_history->is_read >= p_rcgn_obj_struct_history->items_number) {
            break;
        }
        if (p_rcgn_obj_struct_history->base_timestamp && // if timestamp was not given,don't check date
            p_rcgn_obj_struct_history->list[p_rcgn_obj_struct_history->read_index].timestamp < p_rcgn_obj_struct_history->base_timestamp) {
            break;
        }

        list_recognized_objects_item_t* curr_item = &p_rcgn_obj_struct_history->list[p_rcgn_obj_struct_history->read_index];

        if (!first) {
            act_len += 1;
        }
        const char* object = curr_item->object;
        act_len += strlen(object);
        act_len += 1;
        const char* stime = time_to_str(&curr_item->timestamp);
        act_len += strlen(stime);
        act_len += 1;
        const char* expected = curr_item->is_expected ? "Y" : "N";
        act_len += 1;

        if (act_len > (buffer_size-1)) {
            //memset(buffer, 0, buffer_size);
            //return LROBJ_BUFFER_TOO_SMALL;
            break;
        }

        if (!first) {
            strcat(buffer, ";");
        }
        first = false;
        strcat(buffer, object);
        strcat(buffer, ",");
        strcat(buffer, stime);
        strcat(buffer, ",");
        strcat(buffer, expected);


        LROBJ_INCR(p_rcgn_obj_struct_history->read_index)
        p_rcgn_obj_struct_history->is_read += 1;
    }

    return LROBJ_OK;
}

static list_recg_objects_error_t update_history(rcgn_obj_struct_history_t* p_rcgn_obj_struct_history, time_t timestamp)
{
    list_recg_objects_error_t err = LROBJ_OK;

    K_MUTEX_LOCK(lrrp_mutex)

    if (check_crc(&rcgn_obj_struct_current) == false) {
        err = LROBJ_BAD_CRC;
        goto exit;
    }

    for (int i = 0; i < LROBJ_ITEMS_MAX_NUMBER; i++) {
        memcpy(&p_rcgn_obj_struct_history->list[i], &rcgn_obj_struct_current.list[i], sizeof(list_recognized_objects_item_t));
    }

    p_rcgn_obj_struct_history->read_index = rcgn_obj_struct_current.read_index;
    p_rcgn_obj_struct_history->is_read = 0;
    p_rcgn_obj_struct_history->items_number = rcgn_obj_struct_current.items_number;
    p_rcgn_obj_struct_history->base_timestamp = timestamp;

exit:

    K_MUTEX_UNLOCK(lrrp_mutex)

    return err;
}

list_recg_objects_error_t list_recg_objects_read_first_ts(list_recg_objects_history_interface_t history_interface, int count, time_t timestamp, char* buffer, size_t buffer_size)
{
    list_recg_objects_error_t err = LROBJ_OK;

    rcgn_obj_struct_history_t* p_rcgn_obj_struct_history = &rcgn_obj_struct_history[history_interface];

    err = update_history(p_rcgn_obj_struct_history, timestamp);

    if (err == LROBJ_OK) {
        err = read_chunk(p_rcgn_obj_struct_history, count, timestamp, true, buffer, buffer_size);
    }

    return err;
}

list_recg_objects_error_t list_recg_objects_read_first_dt(list_recg_objects_history_interface_t history_interface, int count, const char* date_time, char* buffer, size_t buffer_size)
{
    list_recg_objects_error_t err;
    time_t timestamp;

    if (str_to_time(date_time, &timestamp) == false) {
        err = LROBJ_BAD_DATE_FORMAT;
    }
    else {
        err = list_recg_objects_read_first_ts(history_interface, count, timestamp, buffer, buffer_size);
    }

    return err;
}

list_recg_objects_error_t list_recg_objects_read_next(list_recg_objects_history_interface_t history_interface, int count, char* buffer, size_t buffer_size)
{
    list_recg_objects_error_t err = read_chunk(&rcgn_obj_struct_history[history_interface], count, 0, false, buffer, buffer_size);
    return err;
}

void list_recg_objects_copy(rcgn_obj_struct_current_t* dst)
{
    K_MUTEX_LOCK(lrrp_mutex)

    memcpy(dst, &rcgn_obj_struct_current, sizeof(rcgn_obj_struct_current_t));

    K_MUTEX_UNLOCK(lrrp_mutex)
}
