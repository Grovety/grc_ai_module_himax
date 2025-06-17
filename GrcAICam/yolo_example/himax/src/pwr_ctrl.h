#pragma once

#include <stdint.h>

#include "xprintf.h"
#include "hx_drv_pmu_export.h"
#include "hx_drv_pmu.h"
#include "hx_drv_gpio.h"
#include "hx_drv_swreg_aon.h"
#include "powermode.h"
#include "nrf_comdef.h"

#ifdef __cplusplus
extern "C" {
#endif

int pwr_ctrl_init();

void pwr_dpd_enter();

void pwr_pd_enter();

void pm_sleep();

void pwr_modes_test();

#ifdef __cplusplus
}
#endif
