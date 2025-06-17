#include "board.h"
#include "hx_drv_gpio.h"
#include "hx_drv_scu.h"
#include "hx_drv_scu_export.h"

#include "xprintf.h"

#include "spi_control_nrf.h"

static DEV_SPI_PTR spim_ptr;

static volatile int spim_busy;
static volatile uint8_t spim_ctrl_bits;
static uint16_t spim_tx_cntr;

static uint8_t spim_tx_buf[SPI_TRANS_LEN] __ALIGNED(__SCB_DCACHE_LINE_SIZE);
static uint8_t spim_rx_buf[SPI_TRANS_LEN] __ALIGNED(__SCB_DCACHE_LINE_SIZE);

static void spim_cb(void * status)
{
    hx_drv_gpio_set_out_value(GPIO8, GPIO_OUT_HIGH);

    xprintf("spim_cb (%02x,%02x)\n", spim_rx_buf[0], spim_rx_buf[1]);

    if (spim_rx_buf[0] == (spim_rx_buf[1] ^ 0xFF)) {
        spim_ctrl_bits |= spim_rx_buf[0]; // combine potential actions
    }
    spim_busy = 0;
}

int spim_is_busy()
{
    return spim_busy;
}

void spim_wait()
{
    while (spim_is_busy());
}

uint8_t spim_get_ctrl_bits()
{
    uint8_t ret = spim_ctrl_bits;
    spim_ctrl_bits = 0; // clear mask here
    return ret;
}

void spim_init_buffers(uint8_t opcode)
{
    memset(spim_tx_buf, 0, SPI_TRANS_LEN);
    memset(spim_rx_buf, 0, SPI_TRANS_LEN);
    spim_tx_buf[0] = opcode;
    spim_tx_buf[1] = ~opcode;
}

int spim_add_result(const uint8_t* buf, unsigned len, unsigned off)
{
    if (len > SPI_BUF_PLEN) {
        xprintf("Wrong data number\n");
        return -1;
    }

    if (len+off > SPI_PAYLOAD_LEN) {
        xprintf("Wrong buffer offset\n");
        return -1;
    }

    memcpy(spim_tx_buf+SPI_HDR_SIZE+off, buf, len);

    return 0;
}

int spim_add_data_chunk(const uint8_t* data, unsigned len, uint16_t param)
{
    spim_tx_buf[2] = (uint8_t)(param >> 8);
    spim_tx_buf[3] = (uint8_t)param;

    memcpy(spim_tx_buf+SPI_HDR_SIZE, data, len);

    return 0;
}

void spim_send()
{
    spim_tx_buf[4] = (uint8_t)(spim_tx_cntr >> 8);
    spim_tx_buf[5] = (uint8_t)spim_tx_cntr;
    spim_tx_cntr += 1;

    spim_busy = 1;

    hx_drv_gpio_set_out_value(GPIO8, GPIO_OUT_LOW);

    for (int i = 0; i < 100; i++) {
        asm volatile ("nop");
    }

    spim_ptr->spi_read_write_dma(spim_tx_buf,
                                       SPI_TRANS_LEN,
                                       spim_rx_buf,
                                       SPI_TRANS_LEN,
                                       spim_cb);
    spim_wait();
}

const uint8_t* spim_get_rx_buf()
{
    return (const uint8_t*)spim_rx_buf;
}

int spim_init()
{
    spim_busy = 0;
    spim_ctrl_bits = 0;
    spim_tx_cntr = 0;

    hx_drv_scu_set_SEN_CSW0_pinmux(SCU_SEN_CSW0_PINMUX_GPIO8); \
    hx_drv_gpio_set_output(GPIO8, GPIO_OUT_HIGH);

    hx_drv_scu_set_PB7_pinmux(SCU_PB7_PINMUX_SPI_M_DO, 1);
    hx_drv_scu_set_PB6_pinmux(SCU_PB6_PINMUX_SPI_M_DI, 1);
    hx_drv_scu_set_PB8_pinmux(SCU_PB8_PINMUX_SPI_M_CLK, 1);

    hx_drv_spi_mst_init(USE_DW_SPI_MST_S, DW_SPI_S_RELBASE);
    spim_ptr = hx_drv_spi_mst_get_dev(USE_DW_SPI_MST_S);
    spim_ptr->spi_open(DEV_MASTER_MODE, SPIM_FREQ);

    return 0;
}
