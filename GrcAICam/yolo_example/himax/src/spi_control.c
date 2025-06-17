#include "spi_control.h"

#include "board.h"
#include "hx_drv_gpio.h"
#include "hx_drv_scu.h"
#include "hx_drv_scu_export.h"
#include "hx_drv_spi.h"
#include "xprintf.h"

#define MAX_TX_BUF_SIZE 4092
#define MAX_RX_BUF_SIZE 256
#define SPI_SEN_PIC_CLK (10000000)

DEV_SPI_PTR dev_spi_m;
static volatile uint8_t spi_is_busy = 0;

unsigned char m_tx_buf[MAX_TX_BUF_SIZE] __ALIGNED(__SCB_DCACHE_LINE_SIZE);
unsigned char m_rx_buf[MAX_RX_BUF_SIZE] __ALIGNED(__SCB_DCACHE_LINE_SIZE);

int tx_buf_len = 0;

void sspi_callback_fun_tx(void* status)
{
    spi_is_busy = 0;
}

void sspi_callback_fun_rx(void* status)
{
    spi_is_busy = 0;
}

void spi_control_init()
{
    hx_drv_scu_set_PB2_pinmux(SCU_PB2_PINMUX_SPI_M_DO_1, 1);
    hx_drv_scu_set_PB3_pinmux(SCU_PB3_PINMUX_SPI_M_DI_1, 1);
    hx_drv_scu_set_PB4_pinmux(SCU_PB4_PINMUX_SPI_M_SCLK_1, 1);

    // hx_drv_scu_set_PB11_pinmux(SCU_PB11_PINMUX_GPIO2, 1);
    // hx_drv_gpio_set_output(SPI_M_0, GPIO_OUT_HIGH);

    hx_drv_spi_mst_init(USE_DW_SPI_MST_S, DW_SPI_S_RELBASE);
    dev_spi_m = hx_drv_spi_mst_get_dev(USE_DW_SPI_MST_S);

    dev_spi_m->spi_control(SPI_CMD_SET_RXCB, (SPI_CTRL_PARAM)sspi_callback_fun_rx);
    dev_spi_m->spi_open(DEV_MASTER_MODE, 1000000);
}

void spi_transmit_flush()
{
    if (tx_buf_len == 0) {
        return;
    }
    spi_is_busy = 1;
    dev_spi_m->spi_write_dma(m_tx_buf, tx_buf_len, sspi_callback_fun_tx);
    spi_wait();
    tx_buf_len = 0;
}

unsigned char* get_spi_rx_buf()
{
    return m_rx_buf;
}

int spi_receive(int len)
{
    if (len <= 0) {
        return 0;
    }
    spi_is_busy = 1;
    dev_spi_m->spi_read_dma(m_rx_buf, len, sspi_callback_fun_rx);
    spi_wait();
    return 0;
}

uint8_t spi_transmit_byte(uint8_t data)
{
    spi_is_busy = 1;
    dev_spi_m->spi_write_dma(&data, 1, sspi_callback_fun_tx);
    spi_wait();
    return data;
}

void spi_transmit_byte_fast(uint8_t data)
{
    m_tx_buf[tx_buf_len++] = data;
    if (tx_buf_len == MAX_TX_BUF_SIZE) {
        spi_transmit_flush();
    }
}

void spi_transmit_many_fast(uint8_t* data, int len)
{
    if (len <= 0) {
        return;
    }
    int data_offset = 0;
    int send_cnt = 0;
    while (tx_buf_len + (len - data_offset) > MAX_TX_BUF_SIZE) {
        send_cnt = MAX_TX_BUF_SIZE - tx_buf_len;
        memcpy(&m_tx_buf[tx_buf_len], &data[data_offset], send_cnt);
        tx_buf_len += send_cnt;
        spi_transmit_flush();
        data_offset += send_cnt;
    }
    send_cnt = len - data_offset;
    memcpy(&m_tx_buf[tx_buf_len], &data[data_offset], send_cnt);
    tx_buf_len += send_cnt;
    if (tx_buf_len == MAX_TX_BUF_SIZE) {
        spi_transmit_flush();
    }
}

void spi_transmit_align(int align)
{
    int new_len = tx_buf_len;
    if (new_len % align != 0) {
        new_len = ((new_len / align) + 1) * align;
        tx_buf_len = new_len;
    }
}

static volatile void* dma_transfer_buf = NULL;
static volatile int dma_transfer_buf_len = 0;
static spi_dma_user_callback user_callback = NULL;

static void spi_dma_callback(void* status)
{
    if (dma_transfer_buf_len == 0) {
        spi_is_busy = 0;
        if (user_callback) {
            user_callback();
            user_callback = NULL;
        }
    } else {
        uint8_t* ptr = (uint8_t*)dma_transfer_buf;
        int bytes_to_transfer = dma_transfer_buf_len < MAX_TX_BUF_SIZE ? dma_transfer_buf_len : MAX_TX_BUF_SIZE;

        dma_transfer_buf_len -= bytes_to_transfer;
        dma_transfer_buf = ptr + bytes_to_transfer;
        dev_spi_m->spi_write_dma(ptr, bytes_to_transfer, spi_dma_callback);
    }
}


void spi_transmit_dma(void* data, int len, spi_dma_user_callback callback)
{
    spi_is_busy = 1;
    user_callback = callback;
    dma_transfer_buf_len = len;
    dma_transfer_buf = data;

    spi_dma_callback(NULL);
}

void spi_wait()
{
    while (spi_is_busy == 1) {
        board_delay_ms(1);
    }
}