#include <algorithm>

#include "img_provider.hpp"
#include "WE2_core.h"
#include "cisdp_sensor.h"
#include "config.h"


__attribute__((section(".bss.NoInit")))
static uint8_t rgb_buf[IMG_WIDTH * IMG_HEIGHT * IMG_CH] __ALIGNED(32);


Image ImageProvider::getJPEG()
{
    uint32_t filesize;
    uint32_t addr;
    cisdp_get_jpginfo(&filesize, &addr);

    return Image(
        IMG_WIDTH,
        IMG_HEIGHT,
        reinterpret_cast<uint8_t*>(addr),
        ImageColorScheme::JPEG,
        filesize
    );
}

Image ImageProvider::getRAW()
{
    return Image(
        IMG_WIDTH,
        IMG_HEIGHT,
        reinterpret_cast<uint8_t*>(app_get_raw_addr()),
        ImageColorScheme::YUV,
        app_get_raw_sz()
    );
}

uint8_t clamp_u8(float val)
{
    if (val < 0)
        return 0;
    if (val > 255)
        return 255;
    return (uint8_t)val;
}

inline void swap(uint8_t* x, uint8_t* y)
{
    uint8_t tmp = *x;
    *x = *y;
    *y = tmp;
}

void img_rotate_180(Image& img)
{
    const size_t pixels = img.width * img.height;
    for (size_t p1 = 0; p1 < pixels / 2; p1++) {
        size_t p2 = pixels - 1 - p1;
        swap(img.data + p1 * 3 + 0, img.data + p2 * 3  + 0);
        swap(img.data + p1 * 3 + 1, img.data + p2 * 3 + 1);
        swap(img.data + p1 * 3 + 2, img.data + p2 * 3 + 2);
    }
}

void yuv420toRgb(uint8_t* yuv, uint16_t width, uint16_t height)
{
    const uint32_t frameSize = width * height;
    const uint32_t halfWidth = width / 2;
    const uint32_t uStart = frameSize;
    const uint32_t vStart = frameSize + frameSize / 4;

    int rgb_cnt = 0;
    for (uint16_t y = 0; y < height; y++) {
        for (uint16_t x = 0; x < width; x++) {
            int colorIndex = (y / 2) * halfWidth + (x / 2);
            float yy = yuv[y * width + x];
            float uu = (float)(yuv[uStart + colorIndex]) - 128.0f;
            float vv = (float)(yuv[vStart + colorIndex]) - 128.0f;

            float r = yy + 1.140f * vv;
            float g = yy - 0.395f * uu - 0.581f * vv;
            float b = yy + 2.032f * uu;

            rgb_buf[rgb_cnt++] = clamp_u8(r);
            rgb_buf[rgb_cnt++] = clamp_u8(g);
            rgb_buf[rgb_cnt++] = clamp_u8(b);
        }
    }
}

int img_enchance_brightness(Image& img)
{
    uint8_t* max_ptr = std::max_element(img.data, img.data + img.size);
    float factor = 255.0f / *max_ptr;
    for (size_t i = 0; i < img.size; i++) {
        img.data[i] = (uint8_t)(img.data[i] * factor);
    }
    return img.size;
}

Image ImageProvider::getRGB()
{
    Image yuv = getRAW();
    yuv420toRgb(yuv.data, yuv.width, yuv.height);
    Image rgb(
        IMG_WIDTH,
        IMG_HEIGHT,
        rgb_buf,
        ImageColorScheme::RGB_888
    );
    img_enchance_brightness(rgb);
    return rgb;
}
