#pragma once

#include "tensorflow/lite/micro/micro_interpreter.h"

#include "utils.hpp"

struct Object
{
    Box box;
    size_t label_id;
    float prob;
};

class YoloModel {
    tflite::MicroInterpreter* interpreter;

public:
    YoloModel(const void* model_addr, uint8_t* arena, size_t arena_size);
    ~YoloModel();

    std::string get_label(int idx);

    int init();
    std::vector<Object> invoke(const Image& img);
	
	virtual void yolo_pre_process(const Image& img);
	virtual std::vector<Object> yolo_post_process();

};
