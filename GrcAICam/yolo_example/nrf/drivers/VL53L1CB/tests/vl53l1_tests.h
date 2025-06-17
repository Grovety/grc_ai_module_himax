#ifndef VL53L1_TESTS_H
#define VL53L1_TESTS_H

#include <zephyr/types.h>
#include <zephyr/kernel.h>

#include "vl53l1_api.h"

#define HAL_Delay(ms) k_sleep(K_MSEC(ms));

// #define VL53L1CB_RANGING_TEST

// #define VL53L1CB_MULTIZONE_TEST

#ifdef VL53L1CB_RANGING_TEST

void vl53l1cb_ranging_test(VL53L1_Dev_t* Dev);

#endif // VL53L1CB_RANGING_TEST

#ifdef VL53L1CB_MULTIZONE_TEST

void vl53l1cb_multizone_test(VL53L1_Dev_t* Dev);

#endif // VL53L1CB_MULTIZONE_TEST

#endif // VL53L1_TESTS_H
