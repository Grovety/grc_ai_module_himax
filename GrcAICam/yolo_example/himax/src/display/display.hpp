#pragma once

#include <cstdint>
#include <string>

#include "utils.hpp"

class Display {
public:
    static uint16_t rgb_to_565(uint8_t r, uint8_t g, uint8_t b);
    static void draw_pixel_565(uint16_t color, uint16_t x, uint16_t y, uint8_t size = 1);
    static void draw_char_565(char symbol, uint16_t color, uint16_t x, uint16_t y);

    static void init();
    static void wait_ready();
    static void assign_framebuffer(const Image& img);
    static void draw_box(const Box& box);
    static void draw_text(int line, const std::string& str);
    static void invalidate();
    static void invalidate_async();
};
