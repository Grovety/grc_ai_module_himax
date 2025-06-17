#pragma once

#include <cstdint>
#include <vector>

struct Box {
    uint16_t xmin;
    uint16_t ymin;
    uint16_t xmax;
    uint16_t ymax;

    Box()
        : xmin(0)
        , ymin(0)
        , xmax(0)
        , ymax(0)
    {
    }

    Box(uint16_t xmin, uint16_t ymin, uint16_t xmax, uint16_t ymax)
        : xmin(xmin)
        , ymin(ymin)
        , xmax(xmax)
        , ymax(ymax)
    {
    }
};

typedef std::vector<Box> Boxes;

enum class ImageColorScheme : uint8_t {
    RGB_888,
    RGB_565,
    YUV,
    JPEG,
};

class Image {
public:
    uint16_t width;
    uint16_t height;
    uint8_t* data;
    ImageColorScheme colorScheme;
    size_t size;

    Image(uint16_t width, uint16_t height, uint8_t* data, ImageColorScheme colorScheme, size_t size = 0)
        : width(width)
        , height(height)
        , data(data)
        , colorScheme(colorScheme)
    {
        switch (colorScheme) {
        case ImageColorScheme::RGB_888:
            this->size = width * height * 3;
            break;
        case ImageColorScheme::RGB_565:
            this->size = width * height * 2;
            break;
        default:
            this->size = size;
            break;
        }
    }

    uint8_t rgb_get(uint16_t x, uint16_t y, uint16_t c) const
    {
        return data[3 * (y * width + x) + c];
    }

    static int rescale(const Image& src, const Box& box, Image& dst)
    {
        float x_scale_factor = (box.xmax - box.xmin) / (float)dst.width;
        float y_scale_factor = (box.ymax - box.ymin) / (float)dst.height;

        float src_x, src_y, alpha, beta;
        uint16_t x1, y1, x2, y2;

        int output_idx = 0;

        for (int y = 0; y < dst.height; y++) {
            src_y = (float)y * y_scale_factor + box.ymin;
            y1 = (int)src_y;
            y2 = std::min(y1 + 1, box.ymax - 1);
            beta = src_y - (float)y1;

            for (int x = 0; x < dst.width; x++) {
                src_x = (float)x * x_scale_factor + box.xmin;
                x1 = (int)src_x;
                x2 = std::min(x1 + 1, box.xmax - 1);
                alpha = src_x - (float)x1;

                const int img_channels = 3;
                for (int c = 0; c < img_channels; c++) {
                    float aa = (1.0f - alpha) * (1.0f - beta) * (float)src.rgb_get(x1, y1, c);
                    float bb = alpha * (1.0f - beta) * (float)src.rgb_get(x2, y1, c);
                    float cc = (1.0f - alpha) * beta * (float)src.rgb_get(x1, y2, c);
                    float dd = alpha * beta * (float)src.rgb_get(x2, y2, c);
                    dst.data[output_idx++] = (uint8_t)(aa + bb + cc + dd);
                }
            }
        }
        return output_idx;
    }
};
