#include "powermode_export.h"

#include <cinttypes>
#include <string>
#include <cstdint>
#include <list>
#include <vector>
#include <cstdio>

#include <assert.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

#include "WE2_device.h"
#include "board.h"
#include "WE2_core.h"

#include "hx_drv_spi.h"
#include "spi_eeprom_comm.h"
#include "spi_master_protocol.h"
#include "hx_drv_scu.h"
#include "hx_drv_swreg_aon.h"
#include <hx_drv_pmu.h>
#include <hx_drv_pmu_export.h>
#include "pwr_ctrl.h"

#ifdef IP_sensorctrl
#include "hx_drv_sensorctrl.h"
#endif
#ifdef IP_xdma
#include "hx_drv_xdma.h"
#include "sensor_dp_lib.h"
#endif
#ifdef IP_cdm
#include "hx_drv_cdm.h"
#endif
#ifdef IP_gpio
#include "hx_drv_gpio.h"
#endif

#include "BITOPS.h"
#include "powermode.h"

#include "cisdp_sensor.h"

#include "event_handler.h"

#ifdef __cplusplus
}
#endif  // __cplusplus

#include "main.hpp"
#include "xprintf.h"
#include "config.h"
#include "yolo_model.hpp"
#include "time_calculation.h"
#include "img_provider.hpp"
#include "ethosu_driver.h"
#include "spi_control.h"
#include "display/display.hpp"
#include "spi_control_nrf.h"


/*volatile*/ uint32_t g_img_data = 0;
static uint8_t g_xdma_abnormal, g_md_detect, g_cdm_fifoerror, g_wdt1_timeout, g_wdt2_timeout, g_wdt3_timeout;
static uint8_t g_hxautoi2c_error, g_inp1bitparer_abnormal;
static uint32_t g_dp_event;
static uint8_t g_frame_ready;
static uint32_t g_cur_jpegenc_frame;
static uint8_t g_time;
/*volatile*/ uint32_t jpeg_addr, jpeg_sz;
struct ethosu_driver ethosu_drv; /* Default Ethos-U device driver */

__attribute__((section(".bss.NoInit"))) uint8_t tensor_arena_buf[TENSOR_ARENA_BUFSIZE] __ALIGNED(32);
YoloModel* yolo_model = nullptr;


void main_pipeline_run(int g_cur_jpegenc_frame);


static void init_globals()
{
    g_xdma_abnormal = 0;
    g_md_detect = 0;
    g_cdm_fifoerror = 0;
    g_wdt1_timeout = 0;
    g_wdt2_timeout = 0;
    g_wdt3_timeout = 0;
    g_inp1bitparer_abnormal = 0;
    g_dp_event = 0;
    g_frame_ready = 0;
    g_time = 0;
    g_cur_jpegenc_frame = 0;
    g_hxautoi2c_error = 0;
}


static void dp_app_cv_eventhdl_cb(EVT_INDEX_E event)
{
    uint16_t err;

    // dbg_printf(DBG_LESS_INFO, "EVT event = %d\n", event);
    g_dp_event = event;

    switch (event) {
    case EVT_INDEX_1BITPARSER_ERR: /*reg_inpparser_fs_cycle_error*/
        hx_drv_inp1bitparser_get_errstatus(&err);
        dbg_printf(DBG_LESS_INFO, "EVT_INDEX_1BITPARSER_ERR err=0x%x\r\n", err);
        hx_drv_inp1bitparser_clear_int();
        hx_drv_inp1bitparser_set_enable(0);
        g_inp1bitparer_abnormal = 1;
        break;
    case EVT_INDEX_EDM_WDT1_TIMEOUT:
        dbg_printf(DBG_LESS_INFO, "EVT_INDEX_EDM_WDT1_TlenIMEOUT\r\n");
        g_wdt1_timeout = 1;
        break;
    case EVT_INDEX_EDM_WDT2_TIMEOUT:
        dbg_printf(DBG_LESS_INFO, "EVT_INDEX_EDM_WDT2_TIMEOUT\r\n");
        g_wdt2_timeout = 1;
        break;
    case EVT_INDEX_EDM_WDT3_TIMEOUT:
        dbg_printf(DBG_LESS_INFO, "EVT_INDEX_EDM_WDT3_TIMEOUT\r\n");
        g_wdt3_timeout = 1;
        break;

    case EVT_INDEX_CDM_FIFO_ERR:
        /*
         * error happen need CDM timing & TPG setting
         * 1. SWRESET Datapath
         * 2. restart streaming flow
         */
        dbg_printf(DBG_LESS_INFO, "EVT_INDEX_CDM_FIFO_ERR\r\n");
        g_cdm_fifoerror = 1;

        break;

    case EVT_INDEX_XDMA_WDMA1_ABNORMAL:
    case EVT_INDEX_XDMA_WDMA2_ABNORMAL:
    case EVT_INDEX_XDMA_WDMA3_ABNORMAL:
    case EVT_INDEX_XDMA_RDMA_ABNORMAL:
        /*
         * error happen need
         * 1. SWRESET Datapath
         * 2. restart streaming flow
         */
        dbg_printf(DBG_LESS_INFO, "EVT_INDEX_XDMA_WDMA123_ABNORMAL or EVT_INDEX_XDMA_RDMA_ABNORMAL\r\n");
        g_xdma_abnormal = 1;
        break;

    case EVT_INDEX_CDM_MOTION:
        /*
         * app anything want to do
         * */
        dbg_printf(DBG_LESS_INFO, "Motion Detect\n");
        g_md_detect = 1;
        break;
    case EVT_INDEX_XDMA_FRAME_READY:
        g_cur_jpegenc_frame++;
        g_frame_ready = 1;
        // dbg_printf(DBG_LESS_INFO, "SENSORDPLIB_STATUS_XDMA_FRAME_READY %d \n", g_cur_jpegenc_frame);
        break;

    case EVT_INDEX_SENSOR_RTC_FIRE:
        g_time++;
        break;
    case EVT_INDEX_HXAUTOI2C_ERR:
        dbg_printf(DBG_LESS_INFO, "EVT_INDEX_HXAUTOI2C_ERR\r\n");
        g_hxautoi2c_error = 1;
        break;
    default:
        dbg_printf(DBG_LESS_INFO, "Other Event %d\n", event);
        break;
    }

    if (g_frame_ready == 1) {
        g_frame_ready = 0;
        main_pipeline_run(g_cur_jpegenc_frame);

        // recapture image
        sensordplib_retrigger_capture();
    }

    if (g_md_detect == 1) {
        g_md_detect = 0;
    }

    if (g_inp1bitparer_abnormal == 1 || g_wdt1_timeout == 1 || g_wdt2_timeout == 1 || g_wdt3_timeout == 1 || g_cdm_fifoerror == 1 || g_xdma_abnormal == 1 || g_hxautoi2c_error == 1) {
        cisdp_sensor_stop();
    }
}

void app_start_state(APP_STATE_E state)
{
    if (state == APP_STATE_ALLON) {
        if (cisdp_sensor_init() < 0) {
            xprintf("\r\nCIS Init fail\r\n");
            APP_BLOCK_FUNC();
        }

        init_globals();

        // if wdma variable is zero when not init yet, then this step is a must be to retrieve wdma address
        if (cisdp_dp_init(true, SENSORDPLIB_PATH_INT_INP_HW5X5_JPEG, dp_app_cv_eventhdl_cb, g_img_data, APP_DP_RES_YUV640x480_INP_SUBSAMPLE_2X) < 0) {
            xprintf("\r\nDATAPATH Init fail\r\n");
            APP_BLOCK_FUNC();
        }

        event_handler_init();
        cisdp_sensor_start();
        event_handler_start();
    }
}
/*******************************************************************************
 * Code
 ******************************************************************************/



static void _arm_npu_irq_handler(void)
{
    /* Call the default interrupt handler from the NPU driver */
    ethosu_irq_handler(&ethosu_drv);
}

/**
 * @brief  Initialises the NPU IRQ
 **/
static void _arm_npu_irq_init(void)
{
    const IRQn_Type ethosu_irqnum = (IRQn_Type)U55_IRQn;

    /* Register the EthosU IRQ handler in our vector table.
     * Note, this handler comes from the EthosU driver */
    EPII_NVIC_SetVector(ethosu_irqnum, (uint32_t)_arm_npu_irq_handler);

    /* Enable the IRQ */
    NVIC_EnableIRQ(ethosu_irqnum);
}

static int _arm_npu_init(bool security_enable, bool privilege_enable)
{
    int err = 0;

    /* Initialise the IRQ */
    _arm_npu_irq_init();

    /* Initialise Ethos-U55 device */
    const void* ethosu_base_address = (void*)(U55_BASE);

    if (0 != (err = ethosu_init(&ethosu_drv, /* Ethos-U driver device pointer */
                  ethosu_base_address, /* Ethos-U NPU's base address. */
                  NULL, /* Pointer to fast mem area - NULL for U55. */
                  0, /* Fast mem region size. */
                  security_enable, /* Security enable. */
                  privilege_enable))) { /* Privilege enable. */
        xprintf("failed to initalise Ethos-U device\n");
        return err;
    }

    xprintf("Ethos-U55 device initialised\n");

    return 0;
}

#ifdef SPI_NRF
static bool send_image_flag = false;

static void send_jpeg_image()
{
    Image jpg_image = ImageProvider::getJPEG();
    uint8_t* img_ptr = jpg_image.data;
    unsigned img_cnt = jpg_image.size;
    uint32_t spi_para;
    unsigned spi_send;

    spim_init_buffers(HIMAX_SPI_DATA_IMAGE);

    for (unsigned spi_idx = 0; spi_idx < SPI_MAX_JPG_ITER; spi_idx++) {
        if (img_cnt > SPI_PAYLOAD_LEN) {
            spi_send = SPI_PAYLOAD_LEN;
            spi_para = SPI_JPG_PAR(0, spi_idx, spi_send);
        }
        else {
            spi_send = img_cnt;
            spi_para = SPI_JPG_PAR(1, spi_idx, spi_send);
        }
        xprintf("send %d,%d\n", spi_idx, spi_send); 
        spim_add_data_chunk(img_ptr, spi_send, spi_para);
        spim_send();
        board_delay_ms(6);
        img_cnt -= spi_send;
        img_ptr += spi_send;
        if (img_cnt == 0) {
            break;
        }
    }
}

static void send_rcgn_results(std::string &rcgn_result)
{
    while(spim_is_busy()) { // we don't be here
        xprintf("SPI busy\n");
    }

    spim_init_buffers(HIMAX_SPI_DATA_OBJINFO);
    const char* buf_to_send = rcgn_result.c_str();
    unsigned len_to_send = rcgn_result.length();

    if (len_to_send > SPI_BUF_PLEN-1) {
        xprintf("The result size too big\n");
        len_to_send = SPI_BUF_PLEN-1;
    }

    spim_add_result((const uint8_t*)buf_to_send, len_to_send, 0);
    spim_send();
    board_delay_ms(6);
}

static void ctrl_flag_update(uint8_t ctrl_bits)
{
    if (ctrl_bits & HIMAX_CTL_CONT_WORK) {
        // do nothing
    }
    else if (ctrl_bits & HIMAX_CTL_DPWR_DOWN) {
        xprintf("dpd mode\n");
        pwr_dpd_enter();
    }
    else if (ctrl_bits & HIMAX_CTL_PWR_DOWN) {
        xprintf("pd mode\n");
        pwr_pd_enter();
    }
    if (ctrl_bits & HIMAX_CTL_READ_IMAGE) {
        xprintf("ipeg on\n");
        send_image_flag = true;
    }
}
#endif // SPI_NRF

void main_pipeline_run(int g_cur_jpegenc_frame)
{
    uint32_t pipeline_started_at_ms = get_timestamp_ms();

#ifdef SPI_NRF
    if (send_image_flag) {
        send_jpeg_image();
        send_image_flag = 0;
    }
#endif // SPI_NRF

    // Get the frame from the sensor
    uint32_t start_ms = get_timestamp_ms();
    Image frame = ImageProvider::getRGB();
    // img_rotate_180(frame);
    xprintf("get frame %lu ms\n", get_timestamp_ms() - start_ms);

    start_ms = get_timestamp_ms();
    auto result = yolo_model->invoke(frame);
    xprintf("invoke %lu ms\n", get_timestamp_ms() - start_ms);

    float prob_max = 0.0f;
    Object max_result;
    for (auto& r : result) {
        xprintf("label_id: %s, prob: 0.%03d, (%d, %d, %d, %d)\n",
                yolo_model->get_label(r.label_id).c_str(), (int)(1000 * r.prob),
                r.box.xmin, r.box.ymin, r.box.xmax, r.box.ymax);
        if (r.prob > prob_max) {
            prob_max = r.prob;
            max_result = r;
        }
    }
#ifdef SPI_DISPLAY
    start_ms = get_timestamp_ms();

    Display::wait_ready();
    Display::assign_framebuffer(frame);

    if (prob_max > 0.5f) {
        Display::draw_text(0, yolo_model->get_label(max_result.label_id).c_str());
        for (auto& r : result) {
            Display::draw_box(r.box);
        }
    }

    Display::invalidate_async();
    xprintf("display %lu ms\n", get_timestamp_ms() - start_ms);
#endif // SPI_DISPLAY
#ifdef SPI_NRF
    std::string label("");
    if (prob_max > 0.3f) { // 0.5?
        label = yolo_model->get_label(max_result.label_id);
    }
    send_rcgn_results(label);
    ctrl_flag_update(spim_get_ctrl_bits());
#endif // SPI_NRF
    xprintf("main pipeline loop in %lu ms\n", get_timestamp_ms() - pipeline_started_at_ms);
}


/*!
 * @brief Main function
 */
static int app_main(void)
{
    uint32_t wakeup_event;
    uint32_t wakeup_event1;

    hx_drv_pmu_get_ctrl(PMU_pmu_wakeup_EVT, &wakeup_event);
    hx_drv_pmu_get_ctrl(PMU_pmu_wakeup_EVT1, &wakeup_event1);
    xprintf("wakeup_event=0x%x,WakeupEvt1=0x%x\n", wakeup_event, wakeup_event1);

    hx_lib_spi_eeprom_open(USE_DW_SPI_MST_Q);
    hx_lib_spi_eeprom_enable_XIP(USE_DW_SPI_MST_Q, true, FLASH_QUAD, true);

    // if (_arm_npu_init(security_enable, privilege_enable) != 0)
    if (_arm_npu_init(1, 1) != 0)
        return -1;

    yolo_model = new YoloModel(YOLO_TFLITE_ADDR, tensor_arena_buf, TENSOR_ARENA_BUFSIZE);
    yolo_model->init();

#ifdef SPI_DISPLAY
    spi_control_init();
    Display::init();
#endif // SPI_DISPLAY
#ifdef SPI_NRF
    spim_init();
#endif // SPI_NRF

    app_start_state(APP_STATE_ALLON);

    return 0;
}

int main(void)
{
	board_init();
	app_main();
	return 0;
}
