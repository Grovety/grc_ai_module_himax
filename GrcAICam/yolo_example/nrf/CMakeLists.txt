cmake_minimum_required(VERSION 3.20.0)

add_definitions(-DNRF_SDK_BUILD)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

set(CONF_FILE prj.conf)
set(DTC_OVERLAY_FILE app.overlay)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(yolo_example)

######################### You should to choose appropriate scenario here #########################
#########################       (refer to readme.md file for details)    #########################
## Scenario 1 - external TFT Display is used,nRF does nothing
## Scenario 2 - external TFT display is used, nRF turns on Himax when the TOF sensor is triggered
## Scenario 3 - a smartfon with Bluetooth connection is used,Himax sends data to nRF
add_compile_definitions(APPL_SCENARIO=3)
###################################################################################################

target_include_directories(app PRIVATE
  src/
  src/obj_lists/
  src/cmd_dec/
  src/hm_ctrl
  src/pm_ctrl
  src/tof_sens
)

target_sources(app PRIVATE
  src/obj_lists/objects_validator.c
  src/obj_lists/list_expected_objects.c
  src/obj_lists/list_recognized_objects.c
  src/obj_lists/time_conv.c
  src/obj_lists/crc_proc.c
  src/cmd_dec/common_cmd.c
  src/cmd_dec/himax_cmd.c
  src/cmd_dec/list_expct_obj_cmd.c
  src/cmd_dec/list_recg_obj_cmd.c
  src/cmd_dec/date_time_cmd.c
  src/cmd_dec/tof_sensor_cmd.c
  src/cmd_dec/image_read_cmd.c
  src/cmd_dec/command_decoder.c
  src/hm_ctrl/himax_control.c
  src/hm_ctrl/himax_spi.c
  src/hm_ctrl/himax_spi_data_proc.c
  src/pm_ctrl/pwr_ctrl.c
  src/pm_ctrl/sys_reset.c
  src/tof_sens/tof_sensor_core.c
  src/tof_sens/tof_sensor_results.c
  src/amode_proc.c
  src/detect_handler.c
  src/bt_proc.c
  src/leds_ctrl.c
  src/image_proc.c
  src/acm_uart.c
  src/version.c
  src/main.c
)

zephyr_library_include_directories(.)

set(VL53L1CB_PLAT_DIR ./drivers/VL53L1CB/platform)
set(VL53L1CB_CORE_DIR ./drivers/VL53L1CB/core)

target_include_directories(app PRIVATE 
  ${VL53L1CB_PLAT_DIR}/inc
  ${VL53L1CB_CORE_DIR}/inc
)

target_sources(app PRIVATE
  ${VL53L1CB_PLAT_DIR}/src/vl53l1_platform.c
  ${VL53L1CB_PLAT_DIR}/src/vl53l1_platform_ipp.c
  ${VL53L1CB_CORE_DIR}/src/vl53l1_api.c
  ${VL53L1CB_CORE_DIR}/src/vl53l1_api_calibration.c
  ${VL53L1CB_CORE_DIR}/src/vl53l1_api_core.c
  ${VL53L1CB_CORE_DIR}/src/vl53l1_api_debug.c
  ${VL53L1CB_CORE_DIR}/src/vl53l1_api_preset_modes.c
  ${VL53L1CB_CORE_DIR}/src/vl53l1_api_strings.c
  ${VL53L1CB_CORE_DIR}/src/vl53l1_core.c
  ${VL53L1CB_CORE_DIR}/src/vl53l1_core_support.c
  ${VL53L1CB_CORE_DIR}/src/vl53l1_dmax.c
  ${VL53L1CB_CORE_DIR}/src/vl53l1_error_strings.c
  ${VL53L1CB_CORE_DIR}/src/vl53l1_hist_algos_gen3.c
  ${VL53L1CB_CORE_DIR}/src/vl53l1_hist_algos_gen4.c
  ${VL53L1CB_CORE_DIR}/src/vl53l1_hist_char.c
  ${VL53L1CB_CORE_DIR}/src/vl53l1_hist_core.c
  ${VL53L1CB_CORE_DIR}/src/vl53l1_hist_funcs.c
  ${VL53L1CB_CORE_DIR}/src/vl53l1_nvm.c
  ${VL53L1CB_CORE_DIR}/src/vl53l1_nvm_debug.c
  ${VL53L1CB_CORE_DIR}/src/vl53l1_register_funcs.c
  ${VL53L1CB_CORE_DIR}/src/vl53l1_sigma_estimate.c
  ${VL53L1CB_CORE_DIR}/src/vl53l1_silicon_core.c
  ${VL53L1CB_CORE_DIR}/src/vl53l1_wait.c
  ${VL53L1CB_CORE_DIR}/src/vl53l1_xtalk.c
  ${VL53L1CB_CORE_DIR}/src/vl53l1_zone_presets.c
)
