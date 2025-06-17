#include "yolo_model.hpp"

#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"


#include "config.h"
#include "xprintf.h"
#include <math.h>

static const char* yolo_labels[] = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
    "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
    "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush"
};


struct GridAndStride
{
    int grid0;
    int grid1;
    int stride;
};


static void generate_grids_and_stride(const int target_size, std::vector<int>& strides, std::vector<GridAndStride>& grid_strides)
{
    for (int i = 0; i < (int)strides.size(); i++)
    {
        int stride = strides[i];
        int num_grid = target_size / stride;
        for (int g1 = 0; g1 < num_grid; g1++)
        {
            for (int g0 = 0; g0 < num_grid; g0++)
            {
                GridAndStride gs;
                gs.grid0 = g0;
                gs.grid1 = g1;
                gs.stride = stride;
                grid_strides.push_back(gs);
            }
        }
    }
}

inline float dequantize(int8_t val, float scale, int offset)
{
    return scale * ((float)val - offset);
}

static void generate_yolox_proposals(const std::vector<GridAndStride>& grid_strides, TfLiteTensor* tensor, float prob_threshold, std::vector<Object>& objects)
{
    const int num_class = 80; // tensor->dims[2] - 5;
    const int8_t* feat_ptr = tflite::GetTensorData<int8_t>(tensor);
    const float scale = tensor->params.scale;
    const int offset = tensor->params.zero_point;

    for (size_t anchor_idx = 0; anchor_idx < grid_strides.size(); anchor_idx++) {
        const int grid0 = grid_strides[anchor_idx].grid0;
        const int grid1 = grid_strides[anchor_idx].grid1;
        const int stride = grid_strides[anchor_idx].stride;

        float x_center = (dequantize(feat_ptr[0 * grid_strides.size() + anchor_idx], scale, offset) + grid0) * stride;
        float y_center = (dequantize(feat_ptr[1 * grid_strides.size() + anchor_idx], scale, offset) + grid1) * stride;
        float w = exp(dequantize(feat_ptr[2 * grid_strides.size() + anchor_idx], scale, offset)) * stride;
        float h = exp(dequantize(feat_ptr[3 * grid_strides.size() + anchor_idx], scale, offset)) * stride;

        for (size_t class_idx = 0; class_idx < num_class; class_idx++) {
            float box_cls_score = dequantize(feat_ptr[(4 + class_idx) * grid_strides.size() + anchor_idx], scale, offset);
            float box_prob = box_cls_score;
            if (box_prob > prob_threshold) {
                Object obj;
                // adjust offset to original unpadded
                float x0 = (x_center - w * 0.5f) / ((float)YOLOX_TARGET_SIZE / IMG_WIDTH);
                float y0 = (y_center - h * 0.5f) / ((float)YOLOX_TARGET_SIZE / IMG_HEIGHT);
                float x1 = (x_center + w * 0.5f) / ((float)YOLOX_TARGET_SIZE / IMG_WIDTH);
                float y1 = (y_center + h * 0.5f) / ((float)YOLOX_TARGET_SIZE / IMG_HEIGHT);

                // clip
                x0 = std::max(std::min(x0, (float)(IMG_WIDTH - 1)), 0.f);
                y0 = std::max(std::min(y0, (float)(IMG_HEIGHT - 1)), 0.f);
                x1 = std::max(std::min(x1, (float)(IMG_WIDTH - 1)), 0.f);
                y1 = std::max(std::min(y1, (float)(IMG_HEIGHT - 1)), 0.f);

                obj.box = Box((uint16_t)x0, (uint16_t)y0, (uint16_t)x1, (uint16_t)y1);
                obj.label_id = class_idx;
                obj.prob = box_prob;

                objects.push_back(obj);
            }
        }
    }
}

static void qsort_descent_inplace(std::vector<Object>& faceobjects, int left, int right)
{
    int i = left;
    int j = right;
    float p = faceobjects[(left + right) / 2].prob;

    while (i <= j)
    {
        while (faceobjects[i].prob > p)
            i++;

        while (faceobjects[j].prob < p)
            j--;

        if (i <= j)
        {
            // swap
            std::swap(faceobjects[i], faceobjects[j]);

            i++;
            j--;
        }
    }

    if (left < j)
        qsort_descent_inplace(faceobjects, left, j);

    if (i < right)
        qsort_descent_inplace(faceobjects, i, right);
}

inline void qsort_descent_inplace(std::vector<Object>& objects)
{
    if (objects.empty())
        return;

    qsort_descent_inplace(objects, 0, objects.size() - 1);
}

static float intersection_area(const Object& a, const Object& b)
{
    float x0 = std::max(a.box.xmin, b.box.xmin);
    float x1 = std::min(a.box.xmax, b.box.xmax);
    float y0 = std::max(a.box.ymin, b.box.ymin);
    float y1 = std::min(a.box.ymax, b.box.ymax);
    if (x1 > x0 && y1 > y0)
        return (x1 - x0) * (y1 - y0);
    return 0;
}

static void nms_sorted_bboxes(const std::vector<Object>& faceobjects, std::vector<int>& picked, float nms_threshold)
{
    picked.clear();

    const int n = faceobjects.size();

    std::vector<float> areas(n);
    for (int i = 0; i < n; i++) {
        //areas[i] = faceobjects[i].rect.width * faceobjects[i].rect.height;
        areas[i] = (faceobjects[i].box.xmax - faceobjects[i].box.xmin) * (faceobjects[i].box.ymax - faceobjects[i].box.ymin);
    }

    for (int i = 0; i < n; i++) {
        const Object& a = faceobjects[i];

        int keep = 1;
        for (int j = 0; j < (int)picked.size(); j++)
        {
            const Object& b = faceobjects[picked[j]];

            // intersection over union
            float inter_area = intersection_area(a, b);
            float union_area = areas[i] + areas[picked[j]] - inter_area;
            // float IoU = inter_area / union_area
            if (inter_area / union_area > nms_threshold)
                keep = 0;
        }

        if (keep)
            picked.push_back(i);
    }
}

void YoloModel::yolo_pre_process(const Image& img)
{
	Image resized_img(
        interpreter->input(0)->dims->data[1],
        interpreter->input(0)->dims->data[2],
        interpreter->input(0)->data.uint8,
        ImageColorScheme::RGB_888
    );

    Image::rescale(img, Box(0, 0, resized_img.width, resized_img.height), resized_img);
}

std::vector<Object> YoloModel::yolo_post_process()
{
    static const int stride_arr[] = {8, 16, 32}; // might have stride=64 in YOLOX
    std::vector<int> strides(stride_arr, stride_arr + sizeof(stride_arr) / sizeof(stride_arr[0]));
    std::vector<GridAndStride> grid_strides;
    std::vector<Object> proposals;
	TfLiteTensor* tensor = interpreter->output(0);

    generate_grids_and_stride(YOLOX_TARGET_SIZE, strides, grid_strides);
    generate_yolox_proposals(grid_strides, tensor, YOLOX_CONF_THRESH, proposals);

    qsort_descent_inplace(proposals);

    std::vector<int> picked;
    nms_sorted_bboxes(proposals, picked, YOLOX_NMS_THRESH);

    int count = picked.size();

    std::vector<Object> objects;
    objects.resize(count);

    for (int i = 0; i < count; i++) {
        objects[i] = proposals[picked[i]];
    }

    return objects;
}

YoloModel::YoloModel(const void* model_addr, uint8_t* arena, size_t arena_size)
    : interpreter(nullptr)
{
    const tflite::Model* model = tflite::GetModel(model_addr);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        xprintf(
            "[ERROR] %s model's schema version %d is not equal "
            "to supported version %d\n",
            "yolo8n",
            model->version(), TFLITE_SCHEMA_VERSION);
    } else {
        xprintf("%s model's schema version %d\n", "yolo8n", model->version());
    }

    static tflite::MicroMutableOpResolver<2> op_resolver;
    if (kTfLiteOk != op_resolver.AddEthosU()) {
        xprintf("Failed to add Arm NPU support to op_resolver");
        return;
    }
    if (kTfLiteOk != op_resolver.AddTranspose()) {
        xprintf("Failed to AddTranspose to op_resolver");
        return;
    }

    interpreter = new tflite::MicroInterpreter(model, op_resolver, arena, arena_size);
}

YoloModel::~YoloModel()
{
    if (interpreter)
        delete interpreter;
}

std::string YoloModel::get_label(int idx)
{
    return yolo_labels[idx];
}

int YoloModel::init()
{
    if (interpreter->AllocateTensors() != kTfLiteOk)
        return -1;
    return 0;
}

std::vector<Object> YoloModel::invoke(const Image& img)
{
    yolo_pre_process(img);

    if (interpreter->Invoke() != kTfLiteOk)
        return std::vector<Object>();

    return yolo_post_process();
}
