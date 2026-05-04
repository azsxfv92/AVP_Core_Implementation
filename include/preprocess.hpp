#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <cstddef>

namespace avp
{

enum class ColorOrder
{
    BGR = 0,
    RGB = 1
};

enum class TensorLayout
{
    HWC = 0,
    CHW = 1
};

struct PreprocessConfig
{
    int input_width = 1280;
    int input_height = 720;
    int input_channels = 3;

    int output_width = 640;
    int output_height = 640;
    int output_channels = 3;

    ColorOrder input_color_order = ColorOrder::BGR;
    ColorOrder model_color_order = ColorOrder::RGB;
    TensorLayout output_layout = TensorLayout::CHW;

    bool normalize_to_unit = true; // pixel / 255.0
    bool use_mean_std = false;

    float mean[3] = {0.0f, 0.0f, 0.0f};
    float std[3]  = {1.0f, 1.0f, 1.0f};
};

struct ImageView
{
    const uint8_t* data = nullptr;
    int width = 0;
    int height = 0;
    int channels = 0;
    int step = 0; // bytes per row
};

struct TensorView
{
    float* data = nullptr;
    int n = 1;
    int c = 3;
    int h = 0;
    int w = 0;
};

struct CudaPreprocessContext
{
    uint8_t* d_input = nullptr;
    float* d_output = nullptr;
    size_t input_bytes = 0;
    size_t output_bytes = 0;
    int allocated_input_w = 0;
    int allocated_input_h = 0;
    int allocated_input_c = 0;
    bool initialized = false;
};

struct PreprocessTimingBreakdown
{
    double h2d_ms = 0.0;
    double kernel_ms = 0.0;
    double d2h_ms = 0.0;
    double total_ms = 0.0;
};

bool init_cuda_preprocess_context(
    CudaPreprocessContext& ctx, 
    int input_w,
    int input_h,
    int input_c,
    const PreprocessConfig& cfg
);

bool ensure_cuda_preprocess_context(
    CudaPreprocessContext& ctx,
    int input_w,
    int input_h,
    int input_c,
    int input_step,
    const PreprocessConfig& cfg
);

bool preprocess_cuda_reuse(
    const ImageView& input,
    const PreprocessConfig& cfg,
    TensorView& output,
    CudaPreprocessContext& ctx,
    PreprocessTimingBreakdown& timing
);

void release_cuda_preprocess_context(CudaPreprocessContext& ctx);

bool preprocess_cpu(const ImageView& input, const PreprocessConfig& cfg, TensorView& output);

bool preprocess_cuda(const ImageView& input, const PreprocessConfig& cfg, TensorView& output);

float compare_tensors_l1(const float* a, const float* b, size_t count);

std::string preprocess_config_to_string(const PreprocessConfig& cfg);

}