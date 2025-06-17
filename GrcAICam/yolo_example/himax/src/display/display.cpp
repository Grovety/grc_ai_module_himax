#include "display.hpp"

#include "WE2_core.h"
#include "board.h"
#include "hx_drv_gpio.h"
#include "hx_drv_scu.h"
#include "hx_drv_scu_export.h"

#include "font.hpp"
#include "xprintf.h"
#include "spi_control.h"


__attribute__((section(".display.bss.NoInit"))) static uint16_t display_framebuf_565[320 * 240] __ALIGNED(32);

#define CHARS_COLS_LEN 5 // number of columns for chars
#define CHARS_ROWS_LEN 8 // number of rows for chars

void Display::init()
{
    // TODO: move pinmux to the specific place
    hx_drv_scu_set_PB5_pinmux(SCU_PB5_PINMUX_GPIO16, 1);
    hx_drv_gpio_set_output(GPIO16, GPIO_OUT_HIGH); // Chip Select H
}

void Display::wait_ready()
{
    spi_wait();
}

uint16_t Display::rgb_to_565(uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t pixel_565 = ((uint16_t)(r & 0b11111000) << 8) | ((uint16_t)(g & 0b11111100) << 3) | (uint16_t)(b >> 3);
    return pixel_565;
    // return ((pixel_565 & 0xFF00) >> 8) | ((pixel_565 & 0x00FF) << 8);
}

void Display::assign_framebuffer(const Image& img)
{
    for (size_t i = 0; i < img.width * img.height; i++) {
        display_framebuf_565[i] = rgb_to_565(img.data[i * 3 + 0], img.data[i * 3 + 1], img.data[i * 3 + 2]);
    }
}

void Display::draw_box(const Box& box)
{
    const uint16_t color = 0xE0FF;
    if ((box.xmin > 320) || (box.xmax > 320) || (box.ymin > 240) || (box.ymax > 240) || (box.xmin > box.xmax) || (box.ymin > box.ymax)) {
        xprintf("Invalid box: %d, %d, %d, %d\n", box.xmin, box.ymin, box.xmax, box.ymax);
        return;
    }

    for (uint16_t x = box.xmin; x <= box.xmax; x++) {
        display_framebuf_565[box.ymin * 320 + x] = color;
        display_framebuf_565[box.ymax * 320 + x] = color;
    }
    for (uint16_t y = box.ymin; y <= box.ymax; y++) {
        display_framebuf_565[y * 320 + box.xmin] = color;
        display_framebuf_565[y * 320 + box.xmax] = color;
    }
}

void Display::draw_text(int line, const std::string& str)
{
    if (str.size() < 1)
        return;

    const uint16_t color = 0x00F8;
    const uint16_t letter_interval = 5;
    const uint16_t margin = 2;
    const uint16_t letter_width = 10;
    const uint16_t letter_height = 16;

    uint16_t letter_x = std::max(0, (320 - (int)(str.size() - 1) * (letter_width + letter_interval)) / 2);
    uint16_t letter_y = 6 + line * (letter_height + margin * 2);


    uint16_t bg_x = letter_x - margin;
    uint16_t bg_y = letter_y - margin;
    uint16_t bg_width = (str.size() - 1) * (letter_width + letter_interval) + letter_width + margin * 2;
    uint16_t bg_height = letter_height + margin * 2;

    for (uint16_t y = bg_y; y < bg_y + bg_height; y++) {
        for (uint16_t x = bg_x; x < bg_x + bg_width; x++) {
            draw_pixel_565(0xFFFF, x, y);
        }
    }

    for (size_t i = 0; i < str.size(); i++) {
        draw_char_565(str[i], color, letter_x, letter_y);
        letter_x += letter_interval + letter_width;
    }
}

static void on_spi_done()
{
    hx_drv_gpio_set_out_value(GPIO16, GPIO_OUT_HIGH);
}

void Display::invalidate()
{

    // for (int group = 0; group < 240 / 6; group++) {
    //     hx_drv_gpio_set_out_value(GPIO16, GPIO_OUT_LOW);
    //     spi_transmit_byte(0x5A);
    //     spi_transmit_byte(0x00);
    //     spi_transmit_byte((uint16_t)(group * 6) >> 8);
    //     spi_transmit_byte((uint16_t)(group * 6) & 0xFF);
    //     spi_transmit_byte(0x00);
    //     spi_transmit_byte(0x00);
    //     spi_transmit_byte(0x00);
    //     spi_transmit_byte(0x00);

    //     for (int row = group * 6; row < (group + 1) * 6; row++) {
    //         for (int col = 0; col < 320; col++) {
    //             spi_transmit_byte_fast((uint8_t)display_framebuf_565[row * 320 + col]); // transfer low Byte
    //             spi_transmit_byte_fast((uint8_t)(display_framebuf_565[row * 320 + col] >> 8)); // transfer High Byte
    //         }
    //     }
    //     spi_transmit_flush();
    //     hx_drv_gpio_set_out_value(GPIO16, GPIO_OUT_HIGH);
    // }

    hx_drv_gpio_set_out_value(GPIO16, GPIO_OUT_LOW);
    spi_transmit_dma((uint8_t*)display_framebuf_565, 320 * 240 * 2, nullptr);
    spi_wait();
    hx_drv_gpio_set_out_value(GPIO16, GPIO_OUT_HIGH);

}

void Display::invalidate_async()
{
    hx_drv_gpio_set_out_value(GPIO16, GPIO_OUT_LOW);
    spi_transmit_dma((uint8_t*)display_framebuf_565, 320 * 240 * 2, on_spi_done);
}

void Display::draw_pixel_565(uint16_t color, uint16_t x, uint16_t y, uint8_t size)
{
    if ((x + size > 320) || (y + size > 240))
        return;

    for (uint16_t y_ = y; y_ < y + size; y_++) {
        for (uint16_t x_ = x; x_ < x + size; x_++) {
            display_framebuf_565[y_ * 320 + x_] = color;
        }
    }
}

void Display::draw_char_565(char symbol, uint16_t color, uint16_t x, uint16_t y)
{
    const uint16_t scale = 2;

    if ((symbol < 0x20) && (symbol > 0x7f)) {
        symbol = '?';
    }

    for (size_t col = 0; col < CHARS_COLS_LEN; col++) {
        uint8_t pixels = FONTS[symbol - 32][col];
        for (size_t row = 0; row < CHARS_ROWS_LEN; row++) {
            if (pixels & (1 << row)) {
                uint16_t x_ = x + col * scale;
                uint16_t y_ = y + row * scale;
                draw_pixel_565(color, x_, y_, scale);
            }
        }
    }
}
