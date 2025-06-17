
/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */





#ifndef _VL53L1_PLATFORM_INIT_H_
#define _VL53L1_PLATFORM_INIT_H_

#include "vl53l1_platform.h"

#ifdef __cplusplus
extern "C"
{
#endif






VL53L1_Error VL53L1_platform_init(
	VL53L1_Dev_t *pdev,
	uint8_t       i2c_slave_address,
	uint8_t       comms_type,
	uint16_t      comms_speed_khz);




VL53L1_Error VL53L1_platform_terminate(
	VL53L1_Dev_t *pdev);


#ifdef __cplusplus
}
#endif

#endif


