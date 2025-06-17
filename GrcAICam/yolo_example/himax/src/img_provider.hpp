#pragma once

#include <cstdint>

#include "utils.hpp"

class ImageProvider {

public:
    static Image getRGB();
    static Image getRAW();
    static Image getJPEG();
};

void img_rotate_180(Image& img);
