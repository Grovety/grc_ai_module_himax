/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sys_reset.h"

k_tid_t sys_reset_thread_id;
struct k_thread sys_reset_thread;
K_THREAD_STACK_DEFINE(sys_reset_thread_stack, 512);

static void do_sys_reset(void *, void *, void *)
{
    sys_reboot(SYS_REBOOT_WARM);
}

void init_sys_reset(int delay_ms)
{
        sys_reset_thread_id = k_thread_create(&sys_reset_thread, sys_reset_thread_stack, 512, do_sys_reset,
            NULL, NULL, NULL, -1, K_USER, K_MSEC(delay_ms));
}
