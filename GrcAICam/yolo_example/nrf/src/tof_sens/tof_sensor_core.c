/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/dt-bindings/gpio/gpio.h>

#include "vl53l1_api.h"
#include "vl53l1_platform.h"

#include "mutex_wrapper.h"
#include "tof_sensor_core.h"
#include "himax_control.h"
#include "amode_proc.h"

static tof_msr_param_t tof_msr_param = {
    .dist_mm = {TOF_LOW_DEF_THRESHOLD_DEF, TOF_HIGH_DEF_THRESHOLD_DEF},
    .amode_row = {{TOF_AMODE_ROW_TOP_LEFT_X_DEF, TOF_AMODE_ROW_TOP_LEFT_Y_DEF},
                  {TOF_AMODE_ROW_BOTTOM_RIGHT_X_DEF, TOF_AMODE_ROW_BOTTOM_RIGHT_Y_DEF}},
    .iter_cnt = 0,
    .timing_budget_common = TOF_TIMING_BUDGET_COMMON_DEF,
    .timing_budget_amode = TOF_TIMING_BUDGET_AMODE_DEF
};

#define LOG_MODULE_NAME tof_sensor_core
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define VL53L1X_DEVICE DT_CHOSEN(tof_vl53l1x)
#define VL53L1X_GPIOS DT_PATH(zephyr_user)
static const struct i2c_dt_spec i2c = I2C_DT_SPEC_GET(VL53L1X_DEVICE);
static const struct gpio_dt_spec vl53l1x_inter_gpio = GPIO_DT_SPEC_GET(VL53L1X_GPIOS, tof_inter_gpios);
static const struct gpio_dt_spec vl53l1x_xshut_gpio = GPIO_DT_SPEC_GET(VL53L1X_GPIOS, tof_xshut_gpios);
static struct gpio_callback tof_sensor_irq_callback;

static VL53L1_Dev_t vl53l1x_device;

static tof_mode_t current_mode = TOF_MODE_UNKNOWN;

static int tof_sensor_update_mode(tof_mode_t new_mode);

static void tof_sensor_thread_proc(void);
static void tof_sensor_int_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
#define TOF_SENSOR_THREAD_STACKSIZE 2048
#define TOF_SENSOR_THREAD_PRIORITY 7
static K_THREAD_DEFINE(tof_sensor_thread_id, TOF_SENSOR_THREAD_STACKSIZE, tof_sensor_thread_proc, NULL, NULL, NULL,
		TOF_SENSOR_THREAD_PRIORITY, 0, -1);

static K_EVENT_DEFINE(tof_get_data_event);
K_MUTEX_DEF(tof_setup_mutex);

static void tof_get_data_timer_handler(struct k_timer *dummy);
static K_TIMER_DEFINE(tof_get_data_timer, tof_get_data_timer_handler, NULL);

int tof_sensor_init()
{
    tof_sens_results_init();

    if ((gpio_pin_configure_dt(&vl53l1x_xshut_gpio, GPIO_OUTPUT) != 0) ||
        (gpio_pin_configure_dt(&vl53l1x_inter_gpio, GPIO_INPUT | GPIO_PULL_UP) != 0)) {
        LOG_ERR("tof sensor gpio: configuration error\n");
        return -1;
    }

    tof_sensor_gpio_int_disable();
    gpio_init_callback(&tof_sensor_irq_callback, tof_sensor_int_handler, BIT(vl53l1x_inter_gpio.pin));
    gpio_add_callback(vl53l1x_inter_gpio.port, &tof_sensor_irq_callback);

    gpio_pin_set_dt(&vl53l1x_xshut_gpio, true);
    k_sleep(K_MSEC(20));
    gpio_pin_set_dt(&vl53l1x_xshut_gpio, false);

    vl53l1x_device.i2c = &i2c;
    vl53l1x_device.new_data_ready_poll_duration_ms = 10;

    VL53L1_Error error = VL53L1_WaitDeviceBooted(&vl53l1x_device);
    if (error != VL53L1_ERROR_NONE) {
        LOG_ERR("VL53L1_WaitDeviceBooted-error %d", error);
        return error;
    }

    error = VL53L1_DataInit(&vl53l1x_device);
    if (error != VL53L1_ERROR_NONE) {
        LOG_ERR("VL53L1_DataInit-error %d", error);
        return error;
    }

   	error = VL53L1_StaticInit(&vl53l1x_device);
    if (error != VL53L1_ERROR_NONE) {
        LOG_ERR("VL53L1_StaticInit-error %d", error);
        return error;
    }

    K_MUTEX_INIT(tof_setup_mutex)

    tof_sensor_update_mode(TOF_INIT_MODE);
    k_thread_start(tof_sensor_thread_id);
    return 0;
}

static int tof_sensor_update_mode(tof_mode_t new_mode)
{
    k_timer_stop(&tof_get_data_timer);
    tof_sensor_gpio_int_disable();
    k_event_clear(&tof_get_data_event, 0x03);

    VL53L1_Error error = VL53L1_StopMeasurement(&vl53l1x_device);
    if (error != VL53L1_ERROR_NONE) {
        LOG_ERR("VL53L1_StopMeasurement-error %d", error);
        return error;
    }

    if ((new_mode == TOF_MODE_DISABLED_BY_USER_CMD) || (new_mode == TOF_MODE_DISABLED_WHILE_RECOGN)) {
        current_mode = new_mode;
        return 0;
    }

    VL53L1_PresetModes PresetMode;
    if ((new_mode == TOF_MODE_AUTO_WAIT_FOR_OBJECTS) || (new_mode == TOF_MODE_AUTO_WAIT_DISAPPERANCE))
        PresetMode = TOF_WORK_AUTO_MODE;
    else
        PresetMode = TOF_WORK_RANG_MODE;
    error = VL53L1_SetPresetMode(&vl53l1x_device, PresetMode);
    if (error != VL53L1_ERROR_NONE) {
        LOG_ERR("VL53L1_SetPresetMode-error %d", error);
        return error;
    }

    error = VL53L1_SetDistanceMode(&vl53l1x_device, TOF_WORK_DISTANCE);
    if (error != VL53L1_ERROR_NONE) {
        LOG_ERR("VL53L1_SetDistanceMode-error %d", error);
        return error;
    }

    error = VL53L1_SetLimitCheckEnable(&vl53l1x_device, VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    if (error != VL53L1_ERROR_NONE) {
        LOG_ERR("VL53L1_SetLimitCheckEnable-error %d", error);
        return error;
    }

    error = VL53L1_SetLimitCheckValue(&vl53l1x_device, VL53L1_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 0.40*65536);
    if (error != VL53L1_ERROR_NONE) {
        LOG_ERR("VL53L1_SetLimitCheckValue-error %d", error);
        return error;
    }

    VL53L1_DetectionConfig_t detectionConfig;
    detectionConfig.DetectionMode = VL53L1_DETECTION_DISTANCE_ONLY;
    detectionConfig.Distance.CrossMode = VL53L1_THRESHOLD_IN_WINDOW;
    detectionConfig.Distance.Low = tof_msr_param.dist_mm.low;
    detectionConfig.Distance.High = tof_msr_param.dist_mm.high;
    detectionConfig.IntrNoTarget = 0;
    detectionConfig.Rate.CrossMode = 0;
    detectionConfig.Rate.Low = 0;
    detectionConfig.Rate.High = 0;

    error = VL53L1_SetThresholdConfig(&vl53l1x_device, &detectionConfig);
    if (error != VL53L1_ERROR_NONE) {
        LOG_ERR("VL53L1_SetThresholdConfig-error %d", error);
        return error;
    }

    uint32_t TimingBudget;
    if ((new_mode == TOF_MODE_AUTO_WAIT_FOR_OBJECTS) || (new_mode == TOF_MODE_AUTO_WAIT_DISAPPERANCE))
        TimingBudget = tof_msr_param.timing_budget_amode;
    else
        TimingBudget = tof_msr_param.timing_budget_common;
    error = VL53L1_SetMeasurementTimingBudgetMicroSeconds(&vl53l1x_device, TimingBudget);
    if (error != VL53L1_ERROR_NONE) {
        LOG_ERR("VL53L1_SetMeasurementTimingBudgetMicroSeconds-error %d", error);
        return error;
    }

    if ((new_mode == TOF_MODE_AUTO_WAIT_FOR_OBJECTS) || (new_mode == TOF_MODE_AUTO_WAIT_DISAPPERANCE)) {
        error = VL53L1_SetInterMeasurementPeriodMilliSeconds(&vl53l1x_device, TOF_AUTO_MODE_MSR_RERIOD_MS);
        if (error != VL53L1_ERROR_NONE) {
            LOG_ERR("VL53L1_SetInterMeasurementPeriodMilliSeconds-error %d", error);
            return error;
        }
    }

    VL53L1_UserRoi_t userRoi;
    userRoi.TopLeftX = tof_msr_param.amode_row.top_left.x;
    userRoi.TopLeftY = tof_msr_param.amode_row.top_left.y;
    userRoi.BotRightX = tof_msr_param.amode_row.bottom_rigth.x;
    userRoi.BotRightY = tof_msr_param.amode_row.bottom_rigth.y;

    VL53L1_RoiConfig_t roiConfig;  
    roiConfig.NumberOfRoi = 1;
    roiConfig.UserRois[0] = userRoi;

    error = VL53L1_SetROI(&vl53l1x_device, &roiConfig);
    if (error != VL53L1_ERROR_NONE) {
        LOG_ERR("VL53L1_SetROI-error %d", error);
        return error;
    }

    if (new_mode == TOF_MODE_MEAS_DISTANCE_TIMER) {
        unsigned tmr_period = TimingBudget/1000/8;
        k_timer_start(&tof_get_data_timer,  K_MSEC(tmr_period), K_MSEC(tmr_period));
    }

    if ((new_mode == TOF_MODE_AUTO_WAIT_FOR_OBJECTS) || (new_mode == TOF_MODE_AUTO_WAIT_DISAPPERANCE)) {
        tof_sensor_gpio_int_enable();
    }

    error = VL53L1_StartMeasurement(&vl53l1x_device);
    if (error != VL53L1_ERROR_NONE) {
        LOG_ERR("VL53L1_StartMeasurement-error %d", error);
		return error;
	}
    current_mode = new_mode;
    led_blue_ctrl(current_mode == TOF_MODE_AUTO_WAIT_FOR_OBJECTS ? LED_OFF :
                 (current_mode == TOF_MODE_DISABLED_BY_USER_CMD || 
                  current_mode == TOF_MODE_DISABLED_WHILE_RECOGN) ? 5 : LED_ON);
    return 0;
}

tof_mode_t tof_sensor_get_mode()
{
    K_MUTEX_LOCK(tof_setup_mutex)
    tof_mode_t ret = current_mode;
    K_MUTEX_UNLOCK(tof_setup_mutex)
    return ret;
}

static int tof_sensor_update_results()
{ // return negative-error,0-no ready,1-data ready,but msr status failed,2-ok
    static VL53L1_RangingMeasurementData_t RangingData;
    int ret = TOF_RES_NO_READY;

    uint8_t DataReady;
    VL53L1_Error error = VL53L1_GetMeasurementDataReady(&vl53l1x_device, &DataReady);
    if (error != VL53L1_ERROR_NONE) {
        LOG_ERR("VL53L1_GetMeasurementDataReady failed %d", error);
        VL53L1_StopMeasurement(&vl53l1x_device);
        ret = error;
        goto exit;
    }
    
    if (DataReady == false) {
        // LOG_WRN("VL53L1-data isn't ready");
        ret = TOF_RES_NO_READY;
        goto exit;
    }

    k_sleep(K_MSEC(1));

    //uint8_t PresetMode;
    //error = VL53L1_GetPresetMode(&vl53l1x_device, &PresetMode);
    //if (error != VL53L1_ERROR_NONE) {
    //    LOG_ERR("VL53L1_GetPresetMode failed %d", error);
    //    VL53L1_StopMeasurement(&vl53l1x_device);
    //    ret = error;
    //    goto exit;
    //}

    error = VL53L1_GetRangingMeasurementData(&vl53l1x_device, &RangingData);
    if (error != VL53L1_ERROR_NONE) {
        // LOG_ERR("VL53L1_GetRangingMeasurementData failed %d", error);
        VL53L1_StopMeasurement(&vl53l1x_device);
        ret = error;
        goto exit;
    }

    ret = RangingData.RangeStatus == 0 ? TOF_RES_DATA_OK : TOF_RES_BAD_STAT;
    tof_sens_results_bigw_update(&RangingData);

    error = VL53L1_ClearInterruptAndStartMeasurement(&vl53l1x_device);   
    if (error != VL53L1_ERROR_NONE) {
        LOG_ERR("VL53L1_ClearInterruptAndStartMeasurement failed %d", error);
        ret = error;
    }

exit:

    return ret;
}

static void tof_get_data_timer_handler(struct k_timer *dummy)
{
    k_event_post(&tof_get_data_event, 1);
}

static void tof_sensor_int_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    tof_sensor_gpio_int_disable();
    k_event_post(&tof_get_data_event, 2);
}

void tof_sensor_set_user_mode(tof_mode_t new_mode)
{ // user setup by command
    K_MUTEX_LOCK(tof_setup_mutex)
    tof_sensor_update_mode(new_mode);
    K_MUTEX_UNLOCK(tof_setup_mutex)
}

void tof_sensor_set_auto_mode(tof_mode_t new_mode)
{ // autonomous actions
    K_MUTEX_LOCK(tof_setup_mutex)
    tof_sensor_update_mode(new_mode);
    K_MUTEX_UNLOCK(tof_setup_mutex)
}

static void tof_sensor_thread_proc()
{
    while(1) {
        uint32_t event = k_event_wait(&tof_get_data_event, 3, false, K_FOREVER);
        K_MUTEX_LOCK(tof_setup_mutex)
        if (event & 2) { // interrupt event
            k_event_clear(&tof_get_data_event, 2);
            if ((current_mode == TOF_MODE_AUTO_WAIT_FOR_OBJECTS) || (current_mode == TOF_MODE_AUTO_WAIT_DISAPPERANCE)) {
                tof_sensor_update_results();
                amode_do_recognize();
            }
        }
        else if (event & 1) { // timer event
            k_event_clear(&tof_get_data_event, 1);
            if (current_mode == TOF_MODE_MEAS_DISTANCE_TIMER) {
                if (tof_sensor_update_results() < 0) {
                    tof_sensor_update_mode(TOF_MODE_MEAS_DISTANCE_TIMER);
                }
            }
        }
        K_MUTEX_UNLOCK(tof_setup_mutex)
    }
}

void tof_sensor_gpio_int_enable()
{
    gpio_pin_interrupt_configure_dt(&vl53l1x_inter_gpio, GPIO_INT_LEVEL_LOW);
}

void tof_sensor_gpio_int_disable()
{
    gpio_pin_interrupt_configure_dt(&vl53l1x_inter_gpio, GPIO_INT_DISABLE);
}

void tof_sensor_get_measure_param(char* buf)
{
    K_MUTEX_LOCK(tof_setup_mutex)
    sprintf(buf, "%u,%u;%u,%u,%u,%u;%u,%u",
        tof_msr_param.dist_mm.low, tof_msr_param.dist_mm.high,
        tof_msr_param.amode_row.top_left.x, tof_msr_param.amode_row.top_left.y,
        tof_msr_param.amode_row.bottom_rigth.x, tof_msr_param.amode_row.bottom_rigth.y,
        tof_msr_param.timing_budget_common, tof_msr_param.timing_budget_amode);
    K_MUTEX_UNLOCK(tof_setup_mutex)
}

bool tof_sensor_set_distance(int low, int high)
{
    bool ret = false;
    K_MUTEX_LOCK(tof_setup_mutex)
    if (CHECK_RANGE(low, 0, TOF_PARAM_MAX_DISTANCE) && CHECK_RANGE(high, 0, TOF_PARAM_MAX_DISTANCE) && (low < high)) {
        tof_msr_param.dist_mm.low = (uint16_t)low;
        tof_msr_param.dist_mm.high = (uint16_t)high;
        ret = true;
    }
    K_MUTEX_UNLOCK(tof_setup_mutex)
    return ret;
}

bool tof_sensor_set_amode_roi(int top_left_x, int top_left_y, int bot_rigth_x, int bot_rigth_y)
{
    bool ret = false;
    K_MUTEX_LOCK(tof_setup_mutex)
    if (CHECK_RANGE(top_left_x, 0, 15) && CHECK_RANGE(top_left_y, 0, 15) && 
        CHECK_RANGE(bot_rigth_x, 0, 15) && CHECK_RANGE(bot_rigth_y, 0, 15) &&
        (bot_rigth_x - top_left_x >= 3) && (top_left_y - bot_rigth_y >= 3)) {
        tof_msr_param.amode_row.top_left.x = (uint8_t)top_left_x;
        tof_msr_param.amode_row.top_left.y = (uint8_t)top_left_y;
        tof_msr_param.amode_row.bottom_rigth.x = (uint8_t)bot_rigth_x;
        tof_msr_param.amode_row.bottom_rigth.y = (uint8_t)bot_rigth_y;
        ret = true;
    }
    K_MUTEX_UNLOCK(tof_setup_mutex)
    return ret;
}

bool tof_sensor_set_timing_budgets(int timing_budgets_common, int timing_budgets_amode)
{
    bool ret = false;
    K_MUTEX_LOCK(tof_setup_mutex)
    if (CHECK_RANGE(timing_budgets_common, 20000, 1000000) &&
        CHECK_RANGE(timing_budgets_amode, 20000, 1000000)) {
        tof_msr_param.timing_budget_common = (uint32_t)timing_budgets_common;
        tof_msr_param.timing_budget_amode = (uint32_t)timing_budgets_amode;
        ret = true;
    }
    K_MUTEX_UNLOCK(tof_setup_mutex)
    return ret;
}
