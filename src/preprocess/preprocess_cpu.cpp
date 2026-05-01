#include "preprocess.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <vector>

namespace avp
{
namespace
{
inline int chw_index(int c, int h, int w, int H, int W){
    return c*H*W + h*W+w;
}

inline float normalize_value(float v, const PreprocessConfig& cfg, int c){
    if(cfg.normalize_to_unit){
        v /= 255.0f;
    }

    if(cfg.use_mean_std){
        v = (v - cfg.mean[c]) / cfg.std[c];
    }

    return v;
}
}

bool preprocess_cpu(const ImageView& input, const PreprocessConfig& cfg, TensorView& output){
    if(!input.data || !output.data){
        return false;
    }
    if(input.channels != 3 || cfg.output_channels != 3){
        return false;
    }

    cv::Mat input_mat(input.height, input.width, CV_8UC3, 
    const_cast<uint8_t*>(input.data), static_cast<size_t>(input.step));

    cv::Mat resized;
    cv::resize(input_mat, resized, 
        cv::Size(cfg.output_width, cfg.output_height), 0.0, 0.0, cv::INTER_NEAREST);
    
    cv::Mat color_converted;
    const bool need_bgr_to_rgb = (cfg.input_color_order == ColorOrder::BGR 
        && cfg.model_color_order == ColorOrder::RGB);
    
    const bool need_rgb_to_bgr = (cfg.input_color_order == ColorOrder::RGB 
        && cfg.model_color_order == ColorOrder::BGR);
        
    if(need_bgr_to_rgb || need_rgb_to_bgr){
        cv::cvtColor(resized, color_converted, cv::COLOR_BGR2RGB);
    }
    else{
        color_converted = resized;
    }

    const int H = cfg.output_height;
    const int W = cfg.output_width;

    for (int h = 0; h < H; ++h)
    {
        const cv::Vec3b* row_ptr = color_converted.ptr<cv::Vec3b>(h);
        for (int w = 0; w < W; ++w)
        {
            const uint8_t p0 = row_ptr[w][0];
            const uint8_t p1 = row_ptr[w][1];
            const uint8_t p2 = row_ptr[w][2];

            const float f0 = normalize_value(static_cast<float>(p0), cfg, 0);
            const float f1 = normalize_value(static_cast<float>(p1), cfg, 1);
            const float f2 = normalize_value(static_cast<float>(p2), cfg, 2);

            if (cfg.output_layout == TensorLayout::CHW)
            {
                output.data[chw_index(0, h, w, H, W)] = f0;
                output.data[chw_index(1, h, w, H, W)] = f1;
                output.data[chw_index(2, h, w, H, W)] = f2;
            }
            else
            {
                const int idx = (h * W + w) * 3;
                output.data[idx + 0] = f0;
                output.data[idx + 1] = f1;
                output.data[idx + 2] = f2;
            }
        }
    }

    return true;
}

float compare_tensors_l1(const float* a, const float* b, size_t count){
    if(!a || !b || count == 0){
        return -1.0f;
    }

    double sum = 0.0;
    for(size_t i = 0; i < count; ++i){
        sum += std::abs(static_cast<double>(a[i]) - static_cast<double>(b[i]));
    }
    return static_cast<float>(sum / static_cast<double>(count));
}

std::string preprocess_config_to_string(const PreprocessConfig& cfg){
    std::ostringstream oss;
    oss << "input=" << cfg.input_width << "x" << cfg.input_height
        << "x" << cfg.input_channels
        << ", output=" << cfg.output_width << "x" << cfg.output_height
        << "x" << cfg.output_channels
        << ", normalize_to_unit=" << (cfg.normalize_to_unit ? "true" : "false")
        << ", use_mean_std=" << (cfg.use_mean_std ? "true" : "false");
    return oss.str();
}

}