
/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */



#ifndef _VL53L1_PLATFORM_USER_DATA_H_
#define _VL53L1_PLATFORM_USER_DATA_H_

#ifndef __KERNEL__
#include <stdlib.h>
#endif

#include "vl53l1_def.h"

#ifdef __cplusplus
extern "C"
{
#endif





typedef struct {

	VL53L1_DevData_t   Data;


	uint8_t   i2c_slave_address;

	uint8_t   comms_type;

	uint16_t  comms_speed_khz;


	uint32_t  new_data_ready_poll_duration_ms;

} VL53L1_Dev_t;



typedef VL53L1_Dev_t *VL53L1_DEV;


#define VL53L1DevDataGet(Dev, field) (Dev->Data.field)



#define VL53L1DevDataSet(Dev, field, VL53L1_p_002) ((Dev->Data.field) = (VL53L1_p_002))



#define VL53L1DevStructGetLLDriverHandle(Dev) (&Dev->Data.LLData)


#define VL53L1DevStructGetLLResultsHandle(Dev) (&Dev->Data.llresults)



#ifdef __cplusplus
}
#endif

#endif


