#pragma once
#include "cisdp_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/************************ IMG PROVIDER ****************************/

// set 1 to rewrite rgb buf (detector_input.cc)
#define PUT_DEBUG_FRAME 0

/************************ DETECTOR ****************************/
// set 1 to rewrite detector output with test data(detector_output.cc)
#define SIM_DETECTOR_OUTPUT 0

#define DT_OUTPUT_PRINT 0

#define DETECTOR_THRESHOLD 0.1

#define DT_INPUT_W 320
#define DT_INPUT_H 240
#define DT_INPUT_CH 3
#define DT_INPUT_SIZE (DT_INPUT_W * DT_INPUT_H * DT_INPUT_CH)

#define OUTPUT_BOXES_SIZE 40
#define OUTPUT_CLASSES_SIZE 10
#define OUTPUT_SCORES_SIZE 10
#define OUTPUT_COUNT_SIZE 1


#define WE2_CHIP_VERSION_C 0x8538000c
#define FRAME_CHECK_DEBUG 1
#define SPI_JPG_SEND_DIVIDER 10

#ifdef TRUSTZONE_SEC
#ifdef FREERTOS
/* Trustzone config. */
//
/* FreeRTOS includes. */
// #include "secure_port_macros.h"
#else
#if (__ARM_FEATURE_CMSE & 1) == 0
#error "Need ARMv8-M security extensions"
#elif (__ARM_FEATURE_CMSE & 2) == 0
#error "Compile with --cmse"
#endif
#include "arm_cmse.h"
// #include "veneer_table.h"
//
#endif
#endif


#ifdef EPII_FPGA
#define DBG_APP_LOG (1)
#else
#define DBG_APP_LOG (1)
#endif
#if DBG_APP_LOG
#define dbg_app_log(fmt, ...) xprintf(fmt, ##__VA_ARGS__)
#else
#define dbg_app_log(fmt, ...)
#endif

#define MAX_STRING 100
#define DEBUG_SPIMST_SENDPICS (0x01) // 0x00: off/ 0x01: JPEG/0x02: YUV422/0x03: YUV420/0x04: YUV400/0x05: RGB


#ifdef TRUSTZONE_SEC
#define U55_BASE BASE_ADDR_APB_U55_CTRL_ALIAS
#else
#ifndef TRUSTZONE
#define U55_BASE BASE_ADDR_APB_U55_CTRL_ALIAS
#else
#define U55_BASE BASE_ADDR_APB_U55_CTRL
#endif
#endif

#define YOLO_TFLITE_ADDR ((void*)(BASE_ADDR_FLASH1_R_ALIAS + 0x00B10000))

#define TENSOR_ARENA_BUFSIZE (1474560)
#define YOLOX_NMS_THRESH  0.35f // nms threshold
#define YOLOX_CONF_THRESH 0.25f // threshold of bounding box prob
#define YOLOX_TARGET_SIZE 224  // target image size after resize

#define IMG_WIDTH DP_HW5X5_OUT_WIDTH
#define IMG_HEIGHT DP_HW5X5_OUT_HEIGHT
#define IMG_CH 3

#ifdef __cplusplus
}
#endif
