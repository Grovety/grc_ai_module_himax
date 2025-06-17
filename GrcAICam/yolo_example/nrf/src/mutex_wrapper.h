/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MUTEX_WRAPPER
#define MUTEX_WRAPPER

#include <zephyr/kernel.h>

#define K_MUTEX_DEF(MUTEX) \
    static struct k_mutex MUTEX;

#define K_MUTEX_INIT(MUTEX) \
    k_mutex_init(&MUTEX);

#define K_MUTEX_LOCK(MUTEX) { \
    int err = k_mutex_lock(&MUTEX, K_MSEC(2000)); \
    if (err) \
        LOG_ERR("%s(%d): %d)", __FILE__, __LINE__, err); \
}

#define K_MUTEX_UNLOCK(MUTEX) \
    k_mutex_unlock(&MUTEX);

#endif // MUTEX_WRAPPER
