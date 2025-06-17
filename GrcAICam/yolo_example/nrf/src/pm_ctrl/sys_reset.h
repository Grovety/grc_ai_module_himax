/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SYS_RESET_H
#define SYS_RESET_H

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>

void init_sys_reset(int delay_ms);

#endif // SYS_RESET_H
