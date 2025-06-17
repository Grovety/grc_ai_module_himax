/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PWR_CTRL_H
#define PWR_CTRL_H

#include <stdbool.h>

void pm_ctrl_init();

void pm_charger_status(char* buf);

void pm_charger_ctrl(bool on);

#endif // PWR_CTRL_H