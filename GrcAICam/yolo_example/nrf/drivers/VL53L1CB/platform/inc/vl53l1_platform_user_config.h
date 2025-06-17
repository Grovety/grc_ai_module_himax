
/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53L1 and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */







#ifndef _VL53L1_PLATFORM_USER_CONFIG_H_
#define _VL53L1_PLATFORM_USER_CONFIG_H_

#define    VL53L1_BYTES_PER_WORD              2
#define    VL53L1_BYTES_PER_DWORD             4


#define VL53L1_BOOT_COMPLETION_POLLING_TIMEOUT_MS     500
#define VL53L1_RANGE_COMPLETION_POLLING_TIMEOUT_MS   2000
#define VL53L1_TEST_COMPLETION_POLLING_TIMEOUT_MS   60000

#define VL53L1_POLLING_DELAY_MS                         1


#define VL53L1_TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS  0x8000
#define VL53L1_TUNINGPARM_PRIVATE_PAGE_BASE_ADDRESS 0xC000

#define VL53L1_GAIN_FACTOR__STANDARD_DEFAULT       0x0800

#define VL53L1_GAIN_FACTOR__HISTOGRAM_DEFAULT      0x0800



#define VL53L1_OFFSET_CAL_MIN_EFFECTIVE_SPADS  0x0500


#define VL53L1_OFFSET_CAL_MAX_PRE_PEAK_RATE_MCPS   0x1900


#define VL53L1_OFFSET_CAL_MAX_SIGMA_MM             0x0040



#define VL53L1_ZONE_CAL_MAX_PRE_PEAK_RATE_MCPS     0x1900


#define VL53L1_ZONE_CAL_MAX_SIGMA_MM               0x0040



#define VL53L1_XTALK_EXTRACT_MAX_SIGMA_MM          0x008C



#define VL53L1_MAX_USER_ZONES                16


#define VL53L1_MAX_RANGE_RESULTS             4



#define VL53L1_MAX_STRING_LENGTH 512

#endif


