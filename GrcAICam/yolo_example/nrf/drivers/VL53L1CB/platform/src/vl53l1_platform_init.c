
// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */





#include "vl53l1_ll_def.h"
#include "vl53l1_platform.h"
#include "vl53l1_platform_init.h"


VL53L1_Error VL53L1_platform_init(
	VL53L1_Dev_t *pdev,
	uint8_t       i2c_slave_address,
	uint8_t       comms_type,
	uint16_t      comms_speed_khz)
{


	VL53L1_Error status = VL53L1_ERROR_NONE;



	pdev->i2c_slave_address = i2c_slave_address;
	pdev->comms_type        = comms_type;
	pdev->comms_speed_khz   = comms_speed_khz;

	if (status == VL53L1_ERROR_NONE)
		status =
			VL53L1_CommsInitialise(
				pdev,
				pdev->comms_type,
				pdev->comms_speed_khz);


	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_GpioXshutdown(0);


	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_GpioPowerEnable(0);


	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_GpioCommsSelect(0);


	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_WaitUs(pdev, 1000);


	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_GpioPowerEnable(1);


	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_WaitUs(pdev, 1000);


	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_GpioXshutdown(1);


	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_WaitUs(pdev, 100);

	return status;
}


VL53L1_Error VL53L1_platform_terminate(
		VL53L1_Dev_t *pdev)
{


	VL53L1_Error status = VL53L1_ERROR_NONE;


	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_GpioXshutdown(0);


	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_GpioPowerEnable(0);


	if (status == VL53L1_ERROR_NONE)
		status = VL53L1_CommsClose(pdev);

	return status;
}



